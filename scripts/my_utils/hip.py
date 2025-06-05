import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import os

def compute_hip_orientation_quaternion(joint_positions):
    """
    joint_positions: list of 23 joint 3D coordinates in the order:
    0: 腰 (Hip)
    1: 上半身 (Upper Body)
    2: 上半身2 (Upper Body2)
    3: 首 (Neck)
    4: 頭 (Head)
    5: 左目 (Left Eye)
    6: 右目 (Right Eye)
    7: 左足 (Left Foot)
    8: 左膝 (Left Knee)
    9: 左足首 (Left Ankle)
    10: 左爪先 (Left Toe)
    11: 右足 (Right Foot)
    12: 右膝 (Right Knee)
    13: 右足首 (Right Ankle)
    14: 右爪先 (Right Toe)
    15: 左肩 (Left Shoulder)
    16: 左腕 (Left Arm)
    17: 左肘 (Left Elbow)
    18: 左手首 (Left Wrist)
    19: 右肩 (Right Shoulder)
    20: 右腕 (Right Arm)
    21: 右肘 (Right Elbow)
    22: 右手首 (Right Wrist)
    """

    # 0: Hip（腰）
    hip = np.array(joint_positions[0])

    # 2: Upper Body 2（上半身2）
    upper_body2 = np.array(joint_positions[2])

    # 15: Left Shoulder（左肩）
    left_shoulder = np.array(joint_positions[15])

    # 19: Right Shoulder（右肩）
    right_shoulder = np.array(joint_positions[19])

    # 1) Up ベクトル: hip → upper_body2
    v_up = upper_body2 - hip
    norm_up = np.linalg.norm(v_up)
    if norm_up < 1e-6:
        # Degenerate case: return identity quaternion
        return np.array([0.0, 0.0, 0.0, 1.0])
    u_hat = v_up / norm_up

    # 2) Raw right ベクトル: left_shoulder → right_shoulder
    v_right0 = right_shoulder - left_shoulder
    norm_right0 = np.linalg.norm(v_right0)
    if norm_right0 < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])

    # 3) Gram–Schmidt で right を up に直交化
    proj = np.dot(v_right0, u_hat) * u_hat
    r_ortho = v_right0 - proj
    norm_r = np.linalg.norm(r_ortho)
    if norm_r < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    r_hat = r_ortho / norm_r

    # 4) Forward ベクトル = up × right
    f_vec = np.cross(u_hat, r_hat)
    norm_f = np.linalg.norm(f_vec)
    if norm_f < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    f_hat = f_vec / norm_f

    # 5) 回転行列を列ベクトル [r_hat, u_hat, f_hat] で構築
    rot_matrix = np.column_stack((r_hat, u_hat, f_hat))

    # 6) Rotation オブジェクトに変換してクォータニオンを取得
    rot = R.from_matrix(rot_matrix)
    quat = rot.as_quat()  # [qx, qy, qz, qw]
    return quat

def process_motion_json(input_path, output_dir):
    """
    input_path: もとのジョイント Position 情報を持つ JSON ファイルパス
    output_path: 出力先の JSON ファイルパス
    """
    with open(input_path, 'r') as f:
        data = json.load(f)

    output_list = []
    for entry in data:
        frame_idx = entry["Frame"]
        positions = entry["Position"]
        quat = compute_hip_orientation_quaternion(positions).tolist()
        output_list.append({
            "Frame": frame_idx,
            "HipRotationQuaternion": quat
        })
    
    output_path = os.path.join(output_dir, "hip.json")
    with open(output_path, 'w') as f:
        json.dump(output_list, f, ensure_ascii=False)

parser = argparse.ArgumentParser()
parser.add_argument('--json_path', type=str, default='scripts/Yu/Database/new/upper_motion_raw_data')
parser.add_argument('--output_dir', type=str, default='scripts/Yu/Database/new/standardized_upper_motion_raw_data')
args = parser.parse_args()

if __name__ == "__main__":
    # 必要に応じてファイル名またはパスを変更してください
    input_file = args.json_path 
    output_path = args.output_dir # 入力ファイルのパス
    process_motion_json(input_file, output_path)
