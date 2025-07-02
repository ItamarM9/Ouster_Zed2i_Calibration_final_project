import os
import glob
import cv2
import numpy as np
import open3d as o3d

# ================== CONFIGURATION ==================
#  path to your saved calibration matrix
calib_file   = r'C:\Users\Itamar\Downloads\data_090625_final\calib_subset\calibration_data.npz'

#  your image/LiDAR folders
image_dir = r'C:\Users\Itamar\Downloads\090625_drive\calib_subset\images'
bin_dir   = r'C:\Users\Itamar\Downloads\090625_drive\calib_subset\lidar'

#  Choose a ROI you want to crop
new_roi = {'x': (-25.0, -2.0), 'y': (-12.0, 12.0), 'z': (-2.00, 2.0)}

#  where to write the overlay results
out_dir = os.path.join(os.path.dirname(image_dir), 'projection_with_chosen_roi')
os.makedirs(out_dir, exist_ok=True)
# ====================================================

def imread_unicode(path):
    with open(path, 'rb') as f:
        arr = np.frombuffer(f.read(), np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)

def list_files(folder, ext):
    files = glob.glob(os.path.join(folder, f'*.{ext}'))
    files.sort()
    return files

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
    return np.asarray(pcd.select_by_index(np.where(mask)[0]).points)

def main():
    # --- load calibration ---
    d = np.load(calib_file)
    K    = d['K']
    dist = d['dist']
    R    = d['R']
    t    = d['t']

    # extrinsic as rvec/tvec
    rvec, _ = cv2.Rodrigues(R)
    tvec    = t.reshape(3,1)

    # gather files
    imgs = sorted(list_files(image_dir, 'png') +
                  list_files(image_dir, 'jpg'))
    bins = sorted(list_files(bin_dir, 'bin'))
    if len(imgs) != len(bins):
        raise RuntimeError("Number of images vs. bins mismatch")

    # project each LiDAR crop with depth coloring
    for im_path, bin_path in zip(imgs, bins):
        img = imread_unicode(im_path)

        # load & crop LiDAR points
        pts = load_and_crop_lidar(bin_path, new_roi)  # (N,3)

        # transform to camera frame to get depth
        pts_cam = (R @ pts.T + t[:,None]).T          # (N,3)
        depths = pts_cam[:,2]                        # Z coordinate

        # normalize depths into [0,255]
        dn = (depths - depths.min()) / (depths.max() - depths.min())
        dn = np.clip((dn * 255), 0, 255).astype(np.uint8)

        # map to JET colormap (returns Nx1x3 BGR)
        colors = cv2.applyColorMap(dn.reshape(-1,1), cv2.COLORMAP_JET)
        colors = colors.reshape(-1,3)                # (N,3) BGR

        # project points
        proj, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
        uv = proj.reshape(-1,2).astype(int)

        h, w = img.shape[:2]
        for (u, v), col in zip(uv, colors):
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(img, (u, v), 1, tuple(int(c) for c in col), -1)

        # save overlay
        fn = os.path.basename(im_path)
        cv2.imwrite(os.path.join(out_dir, fn), img)

    print(f"Done! Colored overlays saved to: {out_dir}")

if __name__ == '__main__':
    main()
