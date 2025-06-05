import os
import json
import argparse
import numpy as np
from tqdm import tqdm
from pathlib import Path
from bvh_loader import BVHLoader
from scipy.spatial.transform import Rotation as R

# グローバル位置を計算
def update_node_position(node, frame_data, index, parent_position=[0,0,0], parent_rotation=[0,0,0], is_root=False):
    if node.channels:
        pos_order = []
        rot_order = []
        axis_order = ''
        for axis in node.channels:
            # print(axis)
            # print(index)
            if axis == "Xposition" and index < len(frame_data):
                pos_order.append(frame_data[index])
                index += 1
            if axis == "Yposition" and index < len(frame_data):
                pos_order.append(frame_data[index])
                index += 1
            if axis == "Zposition" and index < len(frame_data):
                pos_order.append(frame_data[index])
                index += 1
            if axis == "Xrotation" and index < len(frame_data):
                rot_order.append(frame_data[index])
                index += 1
                axis_order += 'x'
            if axis == "Yrotation" and index < len(frame_data):
                rot_order.append(frame_data[index])
                index += 1
                axis_order += 'y'
            if axis == "Zrotation" and index < len(frame_data):
                rot_order.append(frame_data[index])
                index += 1
                axis_order += 'z'
       
        if is_root:
            x_pos, y_pos, z_pos = pos_order
            node.position = np.array([x_pos, y_pos, z_pos])
            global_rotation = R.from_euler(axis_order[::-1], rot_order[::-1], degrees=True)
        else:
            if len(pos_order) == 3:
                local_position = np.array(pos_order)
            else:
                local_position = np.array(node.offset)
            local_rotation = R.from_euler(axis_order[::-1], rot_order[::-1], degrees=True)
            global_rotation = parent_rotation * local_rotation
            node.position = parent_position + parent_rotation.apply(local_position)
    else:
        global_rotation = parent_rotation
        node.position = parent_position + parent_rotation.apply(np.array(node.offset))
    # print(node.position)
    # print(node.name)
    for child in node.children:
        index = update_node_position(child, frame_data, index, node.position, global_rotation)

    return index

# ジョイント位置をリストに追加
def compute_frame_positions(root, frame_data):
    positions = []
    update_node_position(root, frame_data, 0, is_root=True)
    def collect_positions(node):
        positions.append(node.position)
        for child in node.children:
            collect_positions(child)
    collect_positions(root)
    return positions

# 関節位置を計算し、フレームごとに保存する関数
def compute_and_save_joint_positions(bvh_dir, output_dir):

    # BVHファイルごとに処理
    for bvh_file in tqdm(os.listdir(bvh_dir)):
        if not bvh_file.endswith(".bvh"):
            continue
        
        file_path = os.path.join(bvh_dir, bvh_file)
        loader = BVHLoader(file_path)
        loader.load()

        root = loader.root
        frame_data = loader.frames

        # 各フレームの関節位置を格納するリスト
        joint_positions_per_frame = []

        # フレームごとに関節位置を計算
        for frame_idx, frame in enumerate(frame_data):
            positions = compute_frame_positions(root, frame)
            # print("Frame", frame_data.index(frame))
            # print("Frame",frame, positions)
            # positions をリスト形式に変換
            positions = [pos.tolist() for pos in positions]  
            joint_positions_per_frame.append({"Frame": frame_idx, "Position": positions})

        # JSONファイルとして出力
        output_file = os.path.join(output_dir, f"{Path(bvh_file).stem}.json")
        with open(output_file, 'w') as f:
            json.dump(joint_positions_per_frame, f)

parser = argparse.ArgumentParser()
parser.add_argument('--bvh_dir', type=str, default='bvh/align/Aligned_BVH_Motion')
parser.add_argument('--output_dir', type=str, default='bvh')
args = parser.parse_args()

if __name__ == '__main__':
    output_dir = args.output_dir
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    compute_and_save_joint_positions(args.bvh_dir, output_dir)