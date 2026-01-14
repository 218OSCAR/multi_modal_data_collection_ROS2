import numpy as np

# 假设你的文件名是 frame_0001.npz
file_path = "/home/fan/tailai_ws/src/multi_modal_data_collection/data/1767969137.255509.npz"

data = np.load(file_path)

# 查看里面都有什么键
print("Keys in the file:", data.files)
