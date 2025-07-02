import os
import numpy as np
import open3d as o3d
import cv2

# === CONFIGURATION ===
bin_path = r"C:\Users\Itamar\Downloads\090625_drive\ouster_data_DMY_09_06_2025_HM_16_28\data\data_80.bin"
roi = {'x': (-19.0, -2.4), 'y': (-8.5, 4.5), 'z': (-1.8, 0.3)}

def load_bin_pointcloud(path):
    pts = np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd

def visualize_with_roi_custom_view(pcd, roi_dict, crop=False):
    if crop:
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            (roi_dict['x'][0], roi_dict['y'][0], roi_dict['z'][0]),
            (roi_dict['x'][1], roi_dict['y'][1], roi_dict['z'][1])
        )
        bbox.color = (1.0, 0.0, 0.0)
        pcd = pcd.crop(bbox)
    else:
        bbox = None

    # color‐by‐X code as before…
    pts = np.asarray(pcd.points)
    x_vals = pts[:, 0]
    x_norm = ((x_vals - x_vals.min()) / (pts.ptp()) * 255).astype(np.uint8)
    colors_bgr = cv2.applyColorMap(x_norm.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
    colors_rgb = colors_bgr[:, ::-1].astype(np.float64) / 255.0
    pcd.colors = o3d.utility.Vector3dVector(colors_rgb)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # make points larger
    opt = vis.get_render_option()
    opt.point_size = 3.0

    vis.add_geometry(pcd)
    if bbox:
        vis.add_geometry(bbox)

    ctr = vis.get_view_control()
    ctr.set_front([1.0, 0.0, 0.0])
    ctr.set_lookat([0.0, 0.0, 0.0])
    ctr.set_up([0.0, 0.0, 1.0])
    ctr.set_zoom(0.3)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    if not os.path.exists(bin_path):
        print(f"File not found: {bin_path}")
        exit(1)

    print("Loading point cloud…")
    pcd = load_bin_pointcloud(bin_path)

    visualize_with_roi_custom_view(pcd, roi, crop=True)
