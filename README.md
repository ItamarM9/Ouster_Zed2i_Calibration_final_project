# LiDAR–Camera Calibration Scripts

This folder contains scripts for: creating a subset of synchronized LiDAR–Camera pairs, calibration - finding the intrinsic and extrinsics, visualizing lidar ROIs, and projecting LiDAR points onto camera images with your chosen ROI.

---

## 📄 What Each Script Does

---

### 1️⃣ `calibration_subset_auto_creation.py`
- **Purpose:** 
  - Finds matching LiDAR and camera frames based on timestamps.
  - Optionally applies chessboard detection to filter pairs.
  - Copies matching pairs into a clean `images/` and `lidar/` subset.
- **Before you run:**
  - ✅ Update the paths in the **CONFIGURATION** section:
    - `IMAGE_DIR`, `LIDAR_DIR` — your raw data folders.
    - `CAMERA_TS_FILE`, `LIDAR_TS_FILE` — timestamp text files.
    - `OUTPUT_DIR` — where to write the subset.
  - ✅ Adjust `CAMERA_OFFSET` and `THRESHOLD` if needed. We saw than as of 01/07/25 the camera's timestemps are early by ~0.9 second relative to the ouster.
  - ✅ If using chessboard filtering (`FILTER_CORNERS=True`):
    - **Verify `PATTERN_SIZE`** matches your actual chessboard (e.g., `(8, 6)` means 8 inner corners wide by 6 high).

- **Note:**  
  Even if you want to test your calibration on an environment **without the chessboard**, you still need to run this script to create a properly **time-synchronized subset** of matching LiDAR and image pairs.

---

### 2️⃣ `calibration_lidar_camera.py`
- **Purpose:**
  - Detects chessboard corners in camera images.
  - Calibrates camera intrinsics.
  - Extracts LiDAR planes.
  - Estimates the LiDAR→Camera transformation.
  - Projects LiDAR points onto images for validation.
- **Before you run:**
  - ✅ Update `image_dir` and `bin_dir` to point to your **subset** folders.
  - ✅ Verify the `roi` (region of interest) is suitable for your LiDAR range.
  - ✅ Make sure your chessboard configuration is correct:
    - **Check `pattern_size`** (number of inner corners).
    - **Check `square_size`** (length of one square in meters).

---

### 3️⃣ `inspect_lidar_roi.py`
- **Purpose:**
  - Loads a LiDAR `.bin` file and shows it in Open3D.
  - Lets you check your region of interest visually.
- **Before you run:**
  - ✅ Update `bin_path` to the LiDAR file you want to inspect.
  - ✅ Adjust `roi` if needed.
  - ✅ Set `crop=True` if you want to see only the cropped points.

---

### 4️⃣ `project_with_different_roi.py`
- **Purpose:**
  - Uses your saved calibration (`calibration_data.npz`).
  - Re-projects LiDAR points onto images with a **ROI of your choise**.
  - Saves new overlay images for validation or visualization.
- **Before you run:**
  - ✅ Update `calib_file` path to your saved `.npz` calibration file.
  - ✅ Set `image_dir` and `bin_dir` to the pairs you want to project.
  - ✅ Adjust `new_roi` to your new region of interest.

---

## ⚠️ What You Must Always Do

✅ **Always update file paths** to match your Ubuntu or Windows machine. 
✅ Make sure your input files exist before you run a script.  
✅ Double-check that `pattern_size` and `square_size` match your physical chessboard when relevant.  
✅ Outputs are written to folders defined in each script — check these after you run.

---

## ✔️ That’s It!

Run each script from PyCharm or a terminal, in the order you need for your workflow.  
No extra setup — just fix your paths and go.

**Good luck with your calibration!** 🚗✨
