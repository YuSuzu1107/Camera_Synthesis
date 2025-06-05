import os
import json
import argparse
import numpy as np
from pathlib import Path
from tqdm import tqdm

def translate_positions(input_dir, output_dir):

    # 入力ディレクトリ内の全てのJSONファイルに対して処理
    for file_name in tqdm(os.listdir(input_dir)):
        if not file_name.endswith(".json"):
            continue

        # 入力ファイルの読み込み
        input_path = os.path.join(input_dir, file_name)
        with open(input_path, 'r') as f:
            data = json.load(f)

        # print(len(data))

        # 各フレームに対してroot位置分の並行移動を行う
        for frame in data:
            root_position = np.array(frame["Position"][0])  # rootの位置
            translated_positions = [np.array(pos) - root_position for pos in frame["Position"]]
            frame["Position"] = [pos.tolist() for pos in translated_positions]  # リスト形式に変換して保存
            # if file_name == "m28.json" and frame["Frame"] == 0:
            #     print(translated_positions)
        
        # 出力ファイルとして保存
        output_path = os.path.join(output_dir, "stand.json")
        with open(output_path, 'w') as f:
            json.dump(data, f)

parser = argparse.ArgumentParser()
parser.add_argument('--json_dir', type=str, default='scripts/Yu/Database/new/upper_motion_raw_data')
parser.add_argument('--output_dir', type=str, default='scripts/Yu/Database/new/standardized_upper_motion_raw_data')
args = parser.parse_args()

if __name__ == '__main__':
    output_dir = args.output_dir
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    translate_positions(args.json_dir, output_dir)