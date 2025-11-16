#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quickly inspect the structure of a recorded episode (.npz)
to see which modalities are saved and their array shapes.

Usage:
    python3 visualize_episode.py /path/to/episode_xxxxx.npz
"""

import numpy as np
import sys
import os
import json

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
            if arr.dtype == object and len(arr) > 0:
                first = arr[0]
                if isinstance(first, np.ndarray):
                    print(f"    ↳ first element: ndarray, shape={first.shape}, dtype={first.dtype}")
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
