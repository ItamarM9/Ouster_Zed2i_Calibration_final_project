#!/usr/bin/env python3
import os
import glob
import re
import cv2
import shutil
from datetime import datetime, timedelta

# ======================= USER CONFIGURATION =======================
# 1) Folders:
IMAGE_DIR      = r"C:\Users\Itamar\Downloads\090625_drive\front_camera_rgb_raw__DMY_09_06_2025_HM_16_28\data"      # where your .png/.jpg camera frames live
LIDAR_DIR      = r"C:\Users\Itamar\Downloads\090625_drive\ouster_data_DMY_09_06_2025_HM_16_28\data"       # where your .bin LiDAR frames live

# 2) Timestamp files:
CAMERA_TS_FILE = r"C:\Users\Itamar\Downloads\090625_drive\front_camera_rgb_raw__DMY_09_06_2025_HM_16_28\timestamps.txt"
LIDAR_TS_FILE  = r"C:\Users\Itamar\Downloads\090625_drive\ouster_data_DMY_09_06_2025_HM_16_28\timestamp.txt"


# 3) Output folder (the script will create .../images and .../lidar inside here):
OUTPUT_DIR     = r"C:\Users\Itamar\Downloads\090625_drive\calib_subset"

# 4) Time‐offset & matching threshold:
CAMERA_OFFSET  = 0.9    # seconds to add to each camera timestamp
THRESHOLD      = 0.05   # max |t_lidar - (t_camera + offset)| in seconds

# 5) Chessboard‐filtering:
FILTER_CORNERS = False   # True → drop any matched pair whose image fails chessboard finding / no chessboard
PATTERN_SIZE   = (8, 6)  # inner corners per row/col
DEBUG_DETECT   = False  # True → show each corner‐detection result briefly
# ==================================================================

def _natural_key(path):
    """
    Natural sort key: extract the first integer from the basename, if any.
    Returns (the_integer, basename). If no digit, returns (None, basename).
    """
    base = os.path.basename(path)
    m = re.search(r'\d+', base)
    if m:
        return (int(m.group()), base)
    else:
        return (None, base)

def parse_lidar_file(path):
    """
    Read LiDAR timestamp file: each non‐empty line is
      <index> <DD/MM/YYYY> <HH:MM:SS:ffffff>
    Returns a sorted list of (lidar_idx:int, raw_ts:str, ts:datetime).
    """
    entries = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            # lidar_idx = line number (1-based)
            lidar_idx = int(parts[0])
            raw_ts    = parts[1] + ' ' + parts[2]
            ts = datetime.strptime(raw_ts, '%d/%m/%Y %H:%M:%S:%f')
            entries.append((lidar_idx, raw_ts, ts))
    # Sort by actual datetime, just in case (but nominally the file is already in t‐order)
    return sorted(entries, key=lambda x: x[2])

def parse_camera_file(path, offset_seconds):
    """
    Read camera timestamp file: each non‐empty line is
      <DD/MM/YYYY> <HH:MM:SS:ffffff>
    Applies CAMERA_OFFSET. Returns sorted list of
      (cam_idx:int, raw_ts:str, ts_adjusted:datetime).
    """
    entries = []
    with open(path, 'r') as f:
        for cam_idx, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            raw_ts = parts[0] + ' ' + parts[1]
            ts = datetime.strptime(raw_ts, '%d/%m/%Y %H:%M:%S:%f')
            ts_adj = ts + timedelta(seconds=offset_seconds)
            entries.append((cam_idx, raw_ts, ts_adj))
    return sorted(entries, key=lambda x: x[2])

def find_matches(lidar_list, cam_list, threshold_sec):
    """
    Finds all (lidar_idx, cam_idx, lidar_raw, cam_raw, delta_secs)
    such that |t_lidar - (t_cam+offset)| <= threshold_sec.
    """
    matches = []
    th = timedelta(seconds=threshold_sec)
    j_start = 0
    n_cam = len(cam_list)

    for (lidar_idx, lidar_raw, lidar_ts) in lidar_list:
        # move j_start until cam_ts >= lidar_ts - threshold
        while j_start < n_cam and cam_list[j_start][2] < (lidar_ts - th):
            j_start += 1

        k = j_start
        # collect all camera entries within +threshold
        while k < n_cam and cam_list[k][2] <= (lidar_ts + th):
            cam_idx, cam_raw, cam_ts_adj = cam_list[k]
            delta = abs((lidar_ts - cam_ts_adj).total_seconds())
            matches.append((lidar_idx, cam_idx, lidar_raw, cam_raw, delta))
            k += 1

    return matches

def detect_valid_images_from_list(img_paths, pattern_size, debug=False):
    """
    Runs findChessboardCornersSB(...) on exactly the list img_paths.
    Returns a set of basenames that passed chessboard detection.
    """
    flags = (cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_EXHAUSTIVE   |
             cv2.CALIB_CB_ACCURACY)
    valid = set()

    for img_path in img_paths:
        img = cv2.imread(img_path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
        if found:
            valid.add(os.path.basename(img_path))
            if debug:
                disp = img.copy()
                cv2.drawChessboardCorners(disp, pattern_size, corners, True)
                cv2.imshow("Corner Detect", disp)
                cv2.waitKey(200)

    if debug:
        cv2.destroyAllWindows()
    return valid

def copy_and_rename_pairs(matches,
                          valid_images_set,
                          image_dir, lidar_dir,
                          out_images_dir, out_lidar_dir,
                          filter_corners,
                          log_path):
    """
    For each matched (lidar_idx, cam_idx, lidar_raw, cam_raw, delta),
    copy image_dir/[…cam_idx…] and lidar_dir/[…lidar_idx…] into out_images_dir
    and out_lidar_dir as “0001.png+0001.bin, 0002.png+0002.bin, …”, and log the details.
    """
    # 1) Build our “naturally sorted” lists of all files in each folder:
    cam_files = sorted(
        glob.glob(os.path.join(image_dir, "*.png")) +
        glob.glob(os.path.join(image_dir, "*.jpg")),
        key=_natural_key
    )
    lidar_files = sorted(
        glob.glob(os.path.join(lidar_dir, "*.bin")),
        key=_natural_key
    )

    # 2) Count how many matches survive filtering → to compute zero‐pad width
    if filter_corners:
        filtered = []
        for (lidar_idx, cam_idx, _, _, _) in matches:
            if not (1 <= cam_idx <= len(cam_files)):
                continue
            name = os.path.basename(cam_files[cam_idx - 1])
            if name in valid_images_set:
                filtered.append((lidar_idx, cam_idx))
        total = len(filtered)
    else:
        total = len(matches)

    width = max(len(str(total)), 1)

    # 3) Write header to log
    with open(log_path, 'w') as log_f:
        log_f.write(
            "new_img_name\toriginal_img_name\tcam_raw_ts\t"
            "new_bin_name\toriginal_bin_name\tlidar_raw_ts\tdelta_seconds\n"
        )

        # 4) Copy, rename, and log each pair
        count = 0
        for (lidar_idx, cam_idx, lidar_raw, cam_raw, delta) in matches:
            # --- Bounds check (in case idx is out of range) ---
            if not (1 <= cam_idx <= len(cam_files)):
                log_f.write(f"# Skipped: cam_idx {cam_idx} out-of-range\n")
                continue
            if not (1 <= lidar_idx <= len(lidar_files)):
                log_f.write(f"# Skipped: lidar_idx {lidar_idx} out-of-range\n")
                continue

            cam_path   = cam_files[cam_idx - 1]
            lidar_path = lidar_files[lidar_idx - 1]
            orig_img   = os.path.basename(cam_path)
            orig_bin   = os.path.basename(lidar_path)

            if filter_corners and (orig_img not in valid_images_set):
                log_f.write(f"# Skipped (no corners): {orig_img}\n")
                continue

            count += 1
            idx_str = str(count).zfill(width)

            # preserve original extension (jpg or png)
            _, img_ext = os.path.splitext(orig_img)
            new_img_name = idx_str + img_ext.lower()   # e.g. "0001.jpg"
            new_bin_name = idx_str + ".bin"             # e.g. "0001.bin"

            dst_img = os.path.join(out_images_dir, new_img_name)
            dst_bin = os.path.join(out_lidar_dir, new_bin_name)

            shutil.copy2(cam_path, dst_img)
            shutil.copy2(lidar_path, dst_bin)

            # Write one line to the log:
            log_f.write(
                f"{new_img_name}\t{orig_img}\t{cam_raw}\t"
                f"{new_bin_name}\t{orig_bin}\t{lidar_raw}\t{delta:.6f}\n"
            )


        print(f"Copied & renamed {count} pairs into:\n"
              f"  Images → {out_images_dir}\n"
              f"  LiDAR  → {out_lidar_dir}")
        print(f"Log written to: {log_path}")

def main():
    # ─ Step A: parse timestamps ───────────────────────────────────
    print("Parsing LiDAR timestamps…")
    lidar_list = parse_lidar_file(LIDAR_TS_FILE)
    print(f"  → {len(lidar_list)} LiDAR lines parsed.")

    print("Parsing camera timestamps (with offset)…")
    cam_list = parse_camera_file(CAMERA_TS_FILE, CAMERA_OFFSET)
    print(f"  → {len(cam_list)} camera lines parsed.")

    # ─ Step B: find all time‐matches within THRESHOLD ─────────────
    print(f"Matching LiDAR↔camera with |Δt| ≤ {THRESHOLD}s …")
    matches = find_matches(lidar_list, cam_list, THRESHOLD)
    print(f"  → Found {len(matches)} raw matches.")

    # ─ Step C: pick exactly the camera‐filenames that appear in matches ────
    matched_cam_indices = {cam_idx for (_, cam_idx, _, _, _) in matches}
    all_cam_files = sorted(
        glob.glob(os.path.join(IMAGE_DIR, "*.png")) +
        glob.glob(os.path.join(IMAGE_DIR, "*.jpg")),
        key=_natural_key
    )
    # Build the list of full paths for only those indices:
    matched_cam_files = [
        all_cam_files[idx - 1]
        for idx in sorted(matched_cam_indices)
        if 1 <= idx <= len(all_cam_files)
    ]

    # ─ Step D: run corner‐detection only on matched images ─────────────
    if FILTER_CORNERS:
        print("\nFILTER_CORNERS=True → running corner‐detection on matched images …")
        valid_images = detect_valid_images_from_list(matched_cam_files, PATTERN_SIZE, DEBUG_DETECT)
        print(f"  → {len(valid_images)}/{len(matched_cam_files)} matched images passed corners.")
    else:
        valid_images = set()

    # ─ Step E: create output dirs ────────────────────────────────────
    out_images_dir = os.path.join(OUTPUT_DIR, "images")
    out_lidar_dir  = os.path.join(OUTPUT_DIR, "lidar")
    os.makedirs(out_images_dir, exist_ok=True)
    os.makedirs(out_lidar_dir, exist_ok=True)

    # Path for the log file
    log_path = os.path.join(OUTPUT_DIR, "subset_log.txt")

    # ─ Step F: copy + rename + log ───────────────────────────────────
    copy_and_rename_pairs(
        matches,
        valid_images,
        IMAGE_DIR, LIDAR_DIR,
        out_images_dir, out_lidar_dir,
        FILTER_CORNERS,
        log_path
    )

    print("\nAll done.")

if __name__ == "__main__":
    main()