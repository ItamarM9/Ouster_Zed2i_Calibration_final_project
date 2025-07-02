import os
import glob
import cv2
import numpy as np
import open3d as o3d
import random

# ================== CONFIGURATION ==================
image_dir    = r"C:\Users\Itamar\Downloads\data_090625_final\calib_subset\images"
bin_dir      = r"C:\Users\Itamar\Downloads\data_090625_final\calib_subset\lidar"
pattern_size = (8, 6)    # inner corners per chessboard (width, height)
square_size  = 0.074      # size of one square (meters)
debug_corner_detection        = False       # True → show each corner‐detection result briefly
roi = {'x': (-8.0, -2.4), 'y': (-2.2, 2.5), 'z': (-0.8, 0.3)}
num_couples = 50     # Number of randomly selected image/lidar couples

parent_dir = os.path.dirname(image_dir)
out_dir = os.path.join(parent_dir, 'lidar_projection')
calib_file = os.path.join(parent_dir, 'calibration_data.npz')

# ====================================================

def imread_unicode(path):
    with open(path, 'rb') as f:
        arr = np.frombuffer(f.read(), np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)

def list_files(folder, ext):
    files = glob.glob(os.path.join(folder, f"*.{ext}"))
    files.sort()
    return files

def detect_chessboard(img_paths):
    flags = (cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_EXHAUSTIVE |
             cv2.CALIB_CB_ACCURACY)
    obj_pts, img_pts, valid = [], [], []
    objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
    objp[:, :2] = np.indices(pattern_size).T.reshape(-1,2) * square_size
    image_size = None

    for img_path in img_paths:
        img = imread_unicode(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        if image_size is None:
            image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
        if not found:
            if debug_corner_detection:
                print(f"No corners in {os.path.basename(img_path)}")
            continue

        obj_pts.append(objp.copy())
        img_pts.append(corners.reshape(-1,2).astype(np.float32))
        valid.append(img_path)

        if debug_corner_detection:
            disp = img.copy()
            cv2.drawChessboardCorners(disp, pattern_size, corners, True)
            cv2.imshow('Corners', disp)
            cv2.waitKey(200)

    if debug_corner_detection:
        cv2.destroyAllWindows()
    if not obj_pts:
        raise RuntimeError("No chessboards detected.")
    return obj_pts, img_pts, valid, image_size

def calibrate_camera_intrinsics(obj_pts, img_pts, image_size):
    ret, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, image_size, None, None)
    if not ret:
        raise RuntimeError("Intrinsic calibration failed.")
    print(f"Intrinsic calibration RMS error: {ret:.3f} px")
    print("Camera matrix K:\n", K)
    print("Distortion coeffs:\n", dist.ravel())
    return K, dist

def load_and_crop_lidar(path, roi):
    data = np.fromfile(path, dtype=np.float32).reshape(-1,4)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
    pts = np.asarray(pcd.points)
    mask = (
        (pts[:,0] >= roi['x'][0]) & (pts[:,0] <= roi['x'][1]) &
        (pts[:,1] >= roi['y'][0]) & (pts[:,1] <= roi['y'][1]) &
        (pts[:,2] >= roi['z'][0]) & (pts[:,2] <= roi['z'][1])
    )
    return pcd.select_by_index(np.where(mask)[0])

def estimate_plane(pcd, threshold=0.01, ransac_n=3, iters=1000):
    model, inds = pcd.segment_plane(threshold, ransac_n, iters)
    a,b,c,_ = model
    normal = np.array([a,b,c], np.float64)
    normal /= np.linalg.norm(normal)
    inliers = np.asarray(pcd.select_by_index(inds).points)
    centroid = inliers.mean(axis=0)
    return normal, centroid, inliers

def compute_extrinsic_errors(valid_imgs,
                             Rcb_list, tcb_list,
                             normals_list, centroids_list,
                             obj_pts, img_pts_list,
                             R_l2c, t_l2c,
                             K, dist):

    trans_errors, rot_errors, reproj_centroid_errors = [], [], []
    rvec_l2c, _ = cv2.Rodrigues(R_l2c)
    tvec_l2c = t_l2c.reshape(3,1)

    for i, img_path in enumerate(valid_imgs):
        name = os.path.basename(img_path)

        # 1) Camera-centroid
        obj_centroid = np.mean(obj_pts[i], axis=0)
        cam_centroid = (Rcb_list[i] @ obj_centroid) + tcb_list[i]

        # 2) LiDAR-centroid
        lidar_centroid = centroids_list[i]
        lidar_centroid_cam = (R_l2c @ lidar_centroid) + t_l2c

        # Translation error [m]
        te = np.linalg.norm(cam_centroid - lidar_centroid_cam)
        trans_errors.append(te)

        # Rotation error [°]
        board_normal_cam = (Rcb_list[i] @ np.array([0,0,1]))
        lidar_normal_cam = R_l2c @ normals_list[i]
        dot = abs(board_normal_cam.dot(lidar_normal_cam))
        dot = np.clip(dot, -1.0, 1.0)
        re = np.degrees(np.arccos(dot))
        rot_errors.append(re)

        # Centroid reprojection error [px]
        proj_centroid, _ = cv2.projectPoints(
            lidar_centroid.reshape(1,3),
            rvec_l2c, tvec_l2c,
            K, dist
        )
        uv_proj = proj_centroid.reshape(-1,2)[0]
        uv_true_centroid = np.mean(img_pts_list[i], axis=0)
        pe = np.linalg.norm(uv_proj - uv_true_centroid)
        reproj_centroid_errors.append(pe)

        print(f"{name}: Translation error = {te:.3f} m, "
              f"Rotation error = {re:.3f}°, "
              f"Centroid reproj error = {pe:.3f} px")

    print(f"\nMean translation error: {np.mean(trans_errors):.3f} m")
    print(f"Mean rotation error: {np.mean(rot_errors):.3f}°")
    print(f"Mean centroid reprojection error: {np.mean(reproj_centroid_errors):.3f} px")

def main():
    # 1) Collect files & calibrate intrinsics
    imgs = sorted(list_files(image_dir,'png') + list_files(image_dir,'jpg'))
    bins = sorted(list_files(bin_dir,'bin'))
    if len(imgs) != len(bins):
        raise ValueError('Mismatch images vs bins')

    obj_pts, img_pts_list, valid_imgs, img_size = detect_chessboard(imgs)
    valid_bins = [
        os.path.join(bin_dir,
                     os.path.basename(p).replace('.png','.bin').replace('.jpg','.bin'))
        for p in valid_imgs
    ]

    print(f"Selecting {num_couples} couples randomly from {len(obj_pts)} detected views...")

    pose_vectors = []
    zero_dist = np.zeros((5,))
    dummy_K = np.eye(3)
    for i in range(len(obj_pts)):
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts[i], img_pts_list[i],
            dummy_K, zero_dist,
            flags=cv2.SOLVEPNP_EPNP
        )
        if ok:
            pose_vec = np.concatenate([tvec.flatten(), rvec.flatten()])
        else:
            pose_vec = np.zeros(6)
        pose_vectors.append(pose_vec)
    pose_vectors = np.array(pose_vectors)

    N = min(num_couples, len(obj_pts))
    random.seed(42)  # for reproducibility. comment this for different random sample every run
    selected = random.sample(range(len(obj_pts)), N)

    obj_pts = [obj_pts[i] for i in selected]
    img_pts_list = [img_pts_list[i] for i in selected]
    valid_imgs = [valid_imgs[i] for i in selected]
    valid_bins = [valid_bins[i] for i in selected]

    print(f"Selected {len(obj_pts)} couples for intrinsic calibration.")

    K, dist = calibrate_camera_intrinsics(obj_pts, img_pts_list, img_size)

    # 2) Per-view calibration: camera→board & LiDAR-plane detection
    Rcb_list, tcb_list = [], []
    corners3d_list = []
    normals_list, centroids_list = [], []
    repro_errors = []

    for idx, (im_path, bin_path) in enumerate(zip(valid_imgs, valid_bins)):
        zero_dist = np.zeros_like(dist)

        # Undistort image points
        pts_cam = img_pts_list[idx].reshape(-1,1,2).astype(np.float32)
        pts_und = cv2.undistortPoints(pts_cam, K, dist, P=K)

        # Solve PnP RANSAC camera->board
        ok, rvec_cb, tvec_cb, _ = cv2.solvePnPRansac(
            obj_pts[idx], pts_und.reshape(-1,2), K, zero_dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=2.0,
            iterationsCount=200, confidence=0.99
        )
        if not ok:
            raise RuntimeError("PnP-RANSAC camera→board failed")
        Rcb, _ = cv2.Rodrigues(rvec_cb)
        tcb = tvec_cb.flatten()
        Rcb_list.append(Rcb)
        tcb_list.append(tcb)

        # Compute reprojection error silently
        proj_cb, _ = cv2.projectPoints(obj_pts[idx], rvec_cb, tvec_cb, K, dist)
        repro_errors.append(
            np.linalg.norm(img_pts_list[idx] - proj_cb.reshape(-1,2), axis=1).mean()
        )

        # Load & crop LiDAR, fit plane
        pcd = load_and_crop_lidar(bin_path, roi)
        normal, cen, inliers = estimate_plane(pcd)
        normals_list.append(normal)
        centroids_list.append(cen)

        # Build 3D chessboard corners in LiDAR frame
        X = inliers - cen
        U,_,Vt = np.linalg.svd(X, full_matrices=False)
        axis1, axis2 = (Vt[0],Vt[1]) if pattern_size[0]>=pattern_size[1] else (Vt[1],Vt[0])
        gravity = np.array([0,0,-1])
        gp = gravity - normal*(gravity.dot(normal))
        if np.linalg.norm(gp)>1e-6: gp/=np.linalg.norm(gp)
        if axis2.dot(gp)<0: axis2=-axis2
        if abs(axis1.dot(gp))>abs(axis2.dot(gp)): axis1,axis2=axis2,axis1
        forward = np.array([1,0,0])
        fp = forward - normal*(forward.dot(normal))
        if np.linalg.norm(fp)>1e-6 and axis1.dot(fp)<0: axis1=-axis1
        axis2 = np.cross(normal,axis1); axis2/=np.linalg.norm(axis2)

        coords = np.vstack((X.dot(axis1), X.dot(axis2))).T
        mu,mv = coords.min(axis=0)
        origin = cen + mu*axis1 + mv*axis2
        nc,nr = pattern_size
        corners3d = np.array([
            origin + i*square_size*axis1 + j*square_size*axis2
            for j in range(nr) for i in range(nc)
        ], np.float32)
        corners3d_list.append(corners3d)

    # 3) Orthogonal Procrustes: LiDAR→Camera extrinsics
    all_cam   = np.vstack([(Rcb_list[i] @ obj_pts[i].T + tcb_list[i][:,None]).T
                           for i in range(len(obj_pts))])
    all_lidar = np.vstack(corners3d_list)
    cent_cam = all_cam.mean(axis=0); cent_lid = all_lidar.mean(axis=0)
    Cc = all_cam - cent_cam; Cl = all_lidar - cent_lid
    H = Cl.T @ Cc
    U,_,Vt = np.linalg.svd(H)
    R_l2c = Vt.T @ U.T
    if np.linalg.det(R_l2c) < 0:
        Vt[-1,:] *= -1
        R_l2c = Vt.T @ U.T
    t_l2c = cent_cam - R_l2c @ cent_lid

    print("\nLiDAR → Camera extrinsics:")
    print("Rotation matrix R_l2c:\n", R_l2c)
    print("Translation vector t_l2c:\n", t_l2c)

    # 4) Compute and print translation, rotation & centroid reprojection errors
    compute_extrinsic_errors(valid_imgs,
                             Rcb_list, tcb_list,
                             normals_list, centroids_list,
                             obj_pts, img_pts_list,
                             R_l2c, t_l2c,
                             K, dist)

    # 5) Save calibration results
    np.savez(calib_file,
             K=K,
             dist=dist,
             R=R_l2c,
             t=t_l2c)
    print(f"Saved calibration_data.npz to\n    {calib_file}")

    # 6) Final overlay with depth‐colored points
    os.makedirs(out_dir, exist_ok=True)
    rvec_f, _ = cv2.Rodrigues(R_l2c)
    tvec_f = t_l2c.reshape(3,1)
    for im_path, bin_path in zip(valid_imgs, valid_bins):
        img = imread_unicode(im_path)
        pts = np.asarray(load_and_crop_lidar(bin_path, roi).points)

        pts_cam = (R_l2c @ pts.T + t_l2c[:,None]).T
        depths = pts_cam[:,2]
        dn = (depths - depths.min()) / (depths.max() - depths.min())
        dn = np.clip((dn * 255), 0, 255).astype(np.uint8)
        colors = cv2.applyColorMap(dn.reshape(-1,1), cv2.COLORMAP_JET).reshape(-1,3)

        proj, _ = cv2.projectPoints(pts, rvec_f, tvec_f, K, dist)
        uv = proj.reshape(-1,2).astype(int)
        h,w = img.shape[:2]
        for (u,v), col in zip(uv, colors):
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(img, (u,v), 1, tuple(int(c) for c in col), -1)

        cv2.imwrite(os.path.join(out_dir, os.path.basename(im_path)), img)

    print(f"Saved overlays to {out_dir}")

if __name__ == '__main__':
    main()
