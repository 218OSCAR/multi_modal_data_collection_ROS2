# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Extract images (RGB + tactile) and CSV data (timestamp + pose)
# from a saved multimodal episode NPZ file.

# Usage:
#     python3 extract_episode_npz.py /path/to/episode_xxxxx.npz
# """

# import os
# import sys
# import csv
# import numpy as np
# import cv2


# def extract_episode(npz_path):
#     if not os.path.exists(npz_path):
#         print(f"❌ File not found: {npz_path}")
#         return

#     print(f"📦 Loading NPZ: {npz_path}")
#     data = np.load(npz_path, allow_pickle=True)

#     # --- derive output folder name ---
#     base_dir = os.path.dirname(npz_path)
#     episode_name = os.path.basename(npz_path).replace(".npz", "")
#     if "_ok" in episode_name:
#         episode_name = episode_name.split("_ok")[0]
#     out_root = os.path.join(base_dir, episode_name)
#     os.makedirs(out_root, exist_ok=True)

#     print(f"📁 Output root: {out_root}")

#     # --- extract and save timestamps ---
#     if "t" in data:
#         t = data["t"]
#         t_csv_path = os.path.join(out_root, "timestamp.csv")
#         np.savetxt(t_csv_path, t, delimiter=",", header="timestamp", comments="")
#         print(f"🕒 Saved timestamps -> {t_csv_path} ({len(t)} entries)")

#     # --- extract and save pose ---
#     if "pose" in data:
#         pose = data["pose"]
#         pose_csv_path = os.path.join(out_root, "tracker_pose.csv")
#         with open(pose_csv_path, "w", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow([
#                 "position_x", "position_y", "position_z",
#                 "orientation_w", "orientation_x", "orientation_y", "orientation_z"
#             ])
#             writer.writerows(pose)
#         print(f"📍 Saved poses -> {pose_csv_path} ({len(pose)} entries)")
#     # --- extract and save RGB images ---
#     if "rgb" in data:
#         rgb_dir = os.path.join(out_root, "realsense_img")
#         os.makedirs(rgb_dir, exist_ok=True)
#         rgb_frames = data["rgb"]
#         for i, img in enumerate(rgb_frames):
#             # 兼容多层 object 嵌套
#             if isinstance(img, np.ndarray) and img.dtype == object:
#                 try:
#                     # 尝试 item() 解包单元素
#                     img = img.item()
#                 except Exception:
#                     # 若是嵌套object数组，则逐元素转uint8
#                     img = np.array(img.tolist(), dtype=np.uint8)
#             if isinstance(img, np.ndarray):
#                 img_uint8 = img.astype(np.uint8)
#                 cv2.imwrite(os.path.join(rgb_dir, f"rgb_{i:04d}.jpg"),
#                             cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR))
#         print(f"📸 Saved {len(rgb_frames)} RGB frames -> {rgb_dir}")

#     # --- extract and save tactile images ---
#     if "tactile" in data:
#         tactile_dir = os.path.join(out_root, "tactile_img")
#         os.makedirs(tactile_dir, exist_ok=True)
#         tactile_frames = data["tactile"]
#         for i, img in enumerate(tactile_frames):
#             if isinstance(img, np.ndarray) and img.dtype == object:
#                 try:
#                     img = img.item()
#                 except Exception:
#                     img = np.array(img.tolist(), dtype=np.uint8)
#             if isinstance(img, np.ndarray):
#                 img_uint8 = img.astype(np.uint8)
#                 cv2.imwrite(os.path.join(tactile_dir, f"tactile_{i:04d}.jpg"),
#                             cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR))
#         print(f"🤚 Saved {len(tactile_frames)} tactile frames -> {tactile_dir}")

#     print("\n✅ Extraction completed successfully.")


# if __name__ == "__main__":
#     if len(sys.argv) < 2:
#         print("Usage: python3 extract_episode_npz.py /path/to/episode_xxxxx.npz")
#         sys.exit(0)
#     extract_episode(sys.argv[1])



#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extract images (RGB + tactile) and CSV data (timestamps + pose + dt)
from a saved multimodal episode NPZ file (new version with timestamps).

Usage:
    python3 extract_episode_npz.py /path/to/episode_xxxxx.npz
"""

import os
import sys
import csv
import numpy as np
import cv2


def save_csv(path, header, array):
    """Utility: save 1D/2D array as CSV."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        if array.ndim == 1:
            for v in array:
                writer.writerow([v])
        else:
            writer.writerows(array)


def extract_episode(npz_path):
    if not os.path.exists(npz_path):
        print(f"❌ File not found: {npz_path}")
        return

    print(f"📦 Loading NPZ: {npz_path}")
    data = np.load(npz_path, allow_pickle=True)

    # --- derive output folder name ---
    base_dir = os.path.dirname(npz_path)
    episode_name = os.path.basename(npz_path).replace(".npz", "")
    if "_ok" in episode_name:
        episode_name = episode_name.split("_ok")[0]

    out_root = os.path.join(base_dir, episode_name)
    os.makedirs(out_root, exist_ok=True)
    print(f"📁 Output root: {out_root}")

    # ============================================================
    # 1) Save timestamps (t_ref, t, *_t, *_dt)
    # ============================================================

    # --- Save t_ref ---
    if "t_ref" in data:
        path = os.path.join(out_root, "t_ref.csv")
        save_csv(path, ["t_ref"], data["t_ref"])
        print(f"🕒 Saved t_ref -> {path} ({len(data['t_ref'])} entries)")

    # --- Save mean t (legacy) ---
    if "t" in data:
        path = os.path.join(out_root, "t_mean.csv")
        save_csv(path, ["t_mean"], data["t"])
        print(f"🕒 Saved t_mean -> {path} ({len(data['t'])} entries)")

    # --- Save per-modality timestamp & dt arrays ---
    for key in data.keys():
        if key.endswith("_t"):
            csv_path = os.path.join(out_root, f"{key}.csv")
            save_csv(csv_path, [key], data[key])
            print(f"⏱ Saved {key} -> {csv_path} ({len(data[key])} entries)")

        if key.endswith("_dt"):
            csv_path = os.path.join(out_root, f"{key}.csv")
            save_csv(csv_path, [key], data[key])
            print(f"⏱ Saved {key} -> {csv_path} ({len(data[key])} entries)")

    # ============================================================
    # 2) Save pose (Vive)
    # ============================================================
    if "pose" in data:
        pose = data["pose"]
        pose_csv_path = os.path.join(out_root, "tracker_pose.csv")
        save_csv(
            pose_csv_path,
            ["px", "py", "pz", "qx", "qy", "qz", "qw"],
            pose
        )
        print(f"📍 Saved poses -> {pose_csv_path} ({len(pose)} entries)")

    # ============================================================
    # 3) Extract and save RGB images
    # ============================================================
    if "rgb" in data:
        rgb_frames = data["rgb"]
        rgb_dir = os.path.join(out_root, "rgb_img")
        os.makedirs(rgb_dir, exist_ok=True)

        for i, img in enumerate(rgb_frames):
            if img.dtype == object:
                try:
                    img = img.item()
                except:
                    img = np.array(img.tolist(), dtype=np.uint8)

            img_uint8 = img.astype(np.uint8)
            cv2.imwrite(
                os.path.join(rgb_dir, f"rgb_{i:04d}.jpg"),
                cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR)
            )

        print(f"📸 Saved {len(rgb_frames)} RGB frames -> {rgb_dir}")

    # ============================================================
    # 4) Extract and save tactile images
    # ============================================================
    if "tactile" in data:
        tactile_frames = data["tactile"]
        tactile_dir = os.path.join(out_root, "tactile_img")
        os.makedirs(tactile_dir, exist_ok=True)

        for i, img in enumerate(tactile_frames):
            if img.dtype == object:
                try:
                    img = img.item()
                except:
                    img = np.array(img.tolist(), dtype=np.uint8)

            img_uint8 = img.astype(np.uint8)
            cv2.imwrite(
                os.path.join(tactile_dir, f"tactile_{i:04d}.jpg"),
                cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR)
            )

        print(f"🤚 Saved {len(tactile_frames)} tactile frames -> {tactile_dir}")

    # ============================================================
    # 5) Save Manus fingertip & glove (optional)
    # ============================================================
    for prefix in ["fingertip_left", "fingertip_right", "glove_left", "glove_right"]:
        if prefix in data:
            arr = data[prefix]
            path = os.path.join(out_root, f"{prefix}.csv")

            # arr may be (N,5,7) or (N,20)
            if arr.ndim == 3:  # fingertip (N, 5, 7)
                flat = arr.reshape(arr.shape[0], -1)
                header = [f"{prefix}_{i}" for i in range(flat.shape[1])]
                save_csv(path, header, flat)
            else:  # glove (N,20)
                header = [f"{prefix}_{i}" for i in range(arr.shape[1])]
                save_csv(path, header, arr)

            print(f"🖐 Saved {prefix} -> {path}")
            
    # ============================================================
    # 3) Extract and save RGB images
    # ============================================================
    if "kinect_rgb" in data:
        rgb_frames = data["kinect_rgb"]
        rgb_dir = os.path.join(out_root, "kinect_rgb_img")
        os.makedirs(rgb_dir, exist_ok=True)

        for i, img in enumerate(rgb_frames):
            if img.dtype == object:
                try:
                    img = img.item()
                except:
                    img = np.array(img.tolist(), dtype=np.uint8)

            img_uint8 = img.astype(np.uint8)
            cv2.imwrite(
                os.path.join(rgb_dir, f"kinect_rgb_{i:04d}.jpg"),
                cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR)
            )

        print(f"📸 Saved {len(rgb_frames)} Kinect RGB frames -> {rgb_dir}")


    print("\n✅ Extraction completed successfully.\n")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_episode_npz.py /path/to/episode_xxxxx.npz")
        sys.exit(0)

    extract_episode(sys.argv[1])
