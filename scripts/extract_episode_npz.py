#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extract images (RGB + tactile) and CSV data (timestamp + pose)
from a saved multimodal episode NPZ file.

Usage:
    python3 extract_episode_npz.py /path/to/episode_xxxxx.npz
"""

import os
import sys
import csv
import numpy as np
import cv2


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

    # --- extract and save timestamps ---
    if "t" in data:
        t = data["t"]
        t_csv_path = os.path.join(out_root, "timestamp.csv")
        np.savetxt(t_csv_path, t, delimiter=",", header="timestamp", comments="")
        print(f"🕒 Saved timestamps -> {t_csv_path} ({len(t)} entries)")

    # --- extract and save pose ---
    if "pose" in data:
        pose = data["pose"]
        pose_csv_path = os.path.join(out_root, "tracker_pose.csv")
        with open(pose_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "position_x", "position_y", "position_z",
                "orientation_w", "orientation_x", "orientation_y", "orientation_z"
            ])
            writer.writerows(pose)
        print(f"📍 Saved poses -> {pose_csv_path} ({len(pose)} entries)")
    # --- extract and save RGB images ---
    if "rgb" in data:
        rgb_dir = os.path.join(out_root, "realsense_img")
        os.makedirs(rgb_dir, exist_ok=True)
        rgb_frames = data["rgb"]
        for i, img in enumerate(rgb_frames):
            # 兼容多层 object 嵌套
            if isinstance(img, np.ndarray) and img.dtype == object:
                try:
                    # 尝试 item() 解包单元素
                    img = img.item()
                except Exception:
                    # 若是嵌套object数组，则逐元素转uint8
                    img = np.array(img.tolist(), dtype=np.uint8)
            if isinstance(img, np.ndarray):
                img_uint8 = img.astype(np.uint8)
                cv2.imwrite(os.path.join(rgb_dir, f"rgb_{i:04d}.jpg"),
                            cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR))
        print(f"📸 Saved {len(rgb_frames)} RGB frames -> {rgb_dir}")

    # --- extract and save tactile images ---
    if "tactile" in data:
        tactile_dir = os.path.join(out_root, "tactile_img")
        os.makedirs(tactile_dir, exist_ok=True)
        tactile_frames = data["tactile"]
        for i, img in enumerate(tactile_frames):
            if isinstance(img, np.ndarray) and img.dtype == object:
                try:
                    img = img.item()
                except Exception:
                    img = np.array(img.tolist(), dtype=np.uint8)
            if isinstance(img, np.ndarray):
                img_uint8 = img.astype(np.uint8)
                cv2.imwrite(os.path.join(tactile_dir, f"tactile_{i:04d}.jpg"),
                            cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR))
        print(f"🤚 Saved {len(tactile_frames)} tactile frames -> {tactile_dir}")

    print("\n✅ Extraction completed successfully.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_episode_npz.py /path/to/episode_xxxxx.npz")
        sys.exit(0)
    extract_episode(sys.argv[1])
