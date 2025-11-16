#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize and inspect NPZ episodes recorded by the ROS2 multimodal data recorder.

Usage:
    python3 visualize_data.py /path/to/episode_xxxxx.npz
"""

import numpy as np
import sys
import os
import json
import cv2

def inspect_npz(npz_path):
    if not os.path.exists(npz_path):
        print(f"❌ File not found: {npz_path}")
        return

    print(f"🔍 Inspecting file: {npz_path}")
    print(f"📦 File size: {os.path.getsize(npz_path)/1e6:.2f} MB\n")

    data = np.load(npz_path, allow_pickle=True)
    keys = list(data.keys())
    print("📑 Keys in file:", keys)
    print()

    # ---- Meta ----
    if "meta_json" in data:
        meta_raw = data["meta_json"].item() if data["meta_json"].dtype == object else str(data["meta_json"])
        meta = json.loads(meta_raw)
        print("🧭 Meta info:")
        for k, v in meta.items():
            print(f"  {k}: {v}")
        print()

    # ---- Time stamps ----
    if "t" in data:
        t = np.asarray(data["t"])
        print(f"⏱  Samples: {len(t)} frames, duration ≈ {t[-1]-t[0]:.2f}s")
    else:
        t = None

    # ---- Pose ----
    if "pose" in data:
        pose = np.asarray(data["pose"])
        print(f"🤖 Pose array: shape={pose.shape}, dtype={pose.dtype}")

    # ---- RGB ----
    if "rgb" in data:
        rgb_arr = data["rgb"]
        print(f"🖼  RGB array: len={len(rgb_arr)}, dtype={rgb_arr.dtype}")
        if len(rgb_arr) > 0:
            first_img = rgb_arr[0]

            # 🧩 如果图像被包成 object 类型或还有嵌套 np.ndarray，则解包
            if isinstance(first_img, np.ndarray) and first_img.dtype == object:
                try:
                    first_img = first_img.item()
                except Exception:
                    first_img = np.asarray(first_img.tolist(), dtype=np.uint8)

            # 🧩 如果此时仍不是标准 uint8 ndarray，尝试转换
            if not isinstance(first_img, np.ndarray):
                print("⚠️ First RGB frame is not ndarray, skipping save.")
            elif first_img.dtype != np.uint8:
                first_img = np.array(first_img, dtype=np.uint8)

            if isinstance(first_img, np.ndarray):
                save_path = os.path.join(os.path.dirname(npz_path), "debug_first_rgb.png")
                try:
                    cv2.imwrite(save_path, cv2.cvtColor(first_img, cv2.COLOR_RGB2BGR))
                    print(f"✅ Saved first RGB frame to: {save_path}")
                except Exception as e:
                    print(f"⚠️ Failed to save RGB image: {e}")
            else:
                print("⚠️ No valid RGB frame found.")





        # ---- 打印结构示例 ----
        if len(rgb_arr) > 0:
            first = rgb_arr[0]
            print(f"   First frame type: {type(first)}, shape={getattr(first, 'shape', None)}, dtype={getattr(first, 'dtype', None)}")

            # 检查是否所有帧的shape一致
            shapes = [getattr(im, "shape", None) for im in rgb_arr]
            uniq_shapes = set(shapes)
            print(f"   Unique shapes among frames: {uniq_shapes}")


        print("\n✅ Inspection completed successfully.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_data.py /path/to/episode_xxxxx.npz")
        sys.exit(0)
    inspect_npz(sys.argv[1])
