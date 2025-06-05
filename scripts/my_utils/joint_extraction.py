from typing import List
import json
import argparse

# ================================
# ★ ここを書き換えてください ★
# ================================
INPUT_PATH     = 'bvh/m200.json'    # 入力ファイル
OUTPUT_PATH    = 'bvh/align/Aligned/m200.json'   # 出力ファイル
# ================================

def extract_and_reorder_positions(
    input_path: str,
    output_path: str,
):
    # 抽出したい Position のインデックスリスト (0 始まり)
    target_indices = [1,2,3,4,5,6,7,8,9,10,12,13,14,16,17,18,19,20,21,37,38,39,40]   
    # 抜き出した subset の何番目を先頭にするか。0～len(TARGET_INDICES)-1 の順序を指定
    new_order = [0,9,10,11,12,13,14,1,2,3,4,5,6,7,8,15,16,17,18,19,20,21,22]
    # JSON 読み込み
    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    result = []
    for rec in data:
        frame = rec.get('Frame')
        positions = rec.get('Position', [])

        # 1) 指定インデックスで抽出
        subset = [positions[i] for i in target_indices if 0 <= i < len(positions)]

        # 2) 新しい順序で並び替え
        #    new_order の各要素 must be in [0, len(subset)-1]
        reordered = [subset[i] for i in new_order]

        result.append({
            'Frame': frame,
            'Position': reordered
        })

    # 保存
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False)

parser = argparse.ArgumentParser()
parser.add_argument('--json_path', type=str, default='scripts/Yu/Database/new/upper_motion_raw_data')
parser.add_argument('--output_path', type=str, default='scripts/Yu/Database/new/standardized_upper_motion_raw_data')
args = parser.parse_args()

if __name__ == '__main__':
    INPUT_PATH = args.json_path
    OUTPUT_PATH = args.output_path
    extract_and_reorder_positions(
        INPUT_PATH,
        OUTPUT_PATH
    )