#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quickly inspect the structure of a recorded episode (.npz)
to see which modalities are saved and their array shapes.

Usage:
    python3 visualize_episode.py /path/to/episode_xxxxx.npz
"""
import cv2
from io import BytesIO

import numpy as np
import sys
import os
import json
def jpeg_bytes_to_rgb(jpeg_bytes):
    """
    Decode JPEG bytes to RGB numpy array (H, W, 3).
    """
    buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("Failed to decode JPEG bytes")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb

def visualize_npz(npz_path):
    if not os.path.exists(npz_path):
        print(f"❌ File not found: {npz_path}")
        return

    print(f"📦 Inspecting file: {npz_path}")
    print(f"📁 Size: {os.path.getsize(npz_path)/1e6:.2f} MB\n")

    data = np.load(npz_path, allow_pickle=True)
    keys = list(data.keys())
    print(f"🔑 Keys in file: {keys}\n")

    # --- Meta info ---
    if "meta_json" in data:
        meta = json.loads(str(data["meta_json"]))
        print("🧾 Meta information:")
        for k, v in meta.items():
            print(f"  {k}: {v}")
        print()

    # --- Show each modality ---
    for k in keys:
        if k == "meta_json":
            continue
        arr = data[k]
        if isinstance(arr, np.ndarray):
            print(f"📍 '{k}': dtype={arr.dtype}, shape={arr.shape}")
            # If it's an object array (list of frames)
            # if arr.dtype == object and len(arr) > 0:
            #     first = arr[0]
            #     if isinstance(first, np.ndarray):
            #         print(f"    ↳ first element: ndarray, shape={first.shape}, dtype={first.dtype}")
            #     else:
            #         print(f"    ↳ first element type: {type(first)}")
            if arr.dtype == object and len(arr) > 0:
                first = arr[0]

                # Case 1: JPEG bytes (rgb / tactile)
                if isinstance(first, (bytes, bytearray)):
                    try:
                        rgb = jpeg_bytes_to_rgb(first)
                        print(
                            f"    ↳ first element: JPEG bytes → decoded RGB, "
                            f"shape={rgb.shape}, dtype={rgb.dtype}"
                        )
                    except Exception as e:
                        print(f"    ↳ first element: bytes (JPEG), decode failed: {e}")

                # Case 2: already ndarray
                elif isinstance(first, np.ndarray):
                    print(
                        f"    ↳ first element: ndarray, "
                        f"shape={first.shape}, dtype={first.dtype}"
                    )

                else:
                    print(f"    ↳ first element type: {type(first)}")

        else:
            print(f"📍 '{k}': type={type(arr)}")

    print("\n✅ Inspection finished.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_episode.py /path/to/episode_xxxxx.npz")
        sys.exit(0)

    visualize_npz(sys.argv[1])
