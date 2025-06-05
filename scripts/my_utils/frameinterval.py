import json
from pathlib import Path
import argparse


def load_json(filepath: Path):
    """
    JSONファイルを読み込んでPythonオブジェクトを返す
    """
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def calculate_intervals_with_start(beats: list, fps: int = 30):
    """
    position == 1 の拍を抽出し、その3個目以降を2個おきに取り出すロジックは変更せずに保持します。
    すでに以下のように書かれています:

    1. position == 1 のリスト position_1_beats を作る
    2. その 3 個目の start (ms) を times[0] に追加
    3. 3 個目以降を 2 個おきに (step=2) times に追加
    4. times の隣接要素差分を intervals にする
    5. ミリ秒→フレーム数に変換して frame_intervals を返す
    """
    # position == 1 を抽出
    position_1_beats = [beat for beat in beats if beat["position"] == 1]

    # 要素数が足りない場合は空返却
    if len(position_1_beats) < 3:
        return [], 0

    # 3個目の start(ms) を最初に追加
    times = [position_1_beats[2]["start"]]

    # 3個目以降を 2 個おき (step=2) で追加
    for i in range(1, len(position_1_beats) - 2, 2):
        times.append(position_1_beats[i + 2]["start"])

    # times の隣接差分を intervals リスト (ms) にまとめる
    intervals = [times[0]] + [
        times[i + 1] - times[i] for i in range(len(times) - 1)
    ]
    # ミリ秒 → フレーム数に変換 (四捨五入)
    frame_intervals = [round(interval * fps / 1000) for interval in intervals]

    total_frames = sum(frame_intervals)
    return frame_intervals, total_frames


def get_motion_frame_count(motion_list: list):
    """
    motion_list は以下のようなリスト形式を想定:
      [
        {"Frame": 0, "Position": [...]},
        {"Frame": 1, "Position": [...]},
        ...
      ]
    ここから最大の "Frame" を取り、その +1 を motion_frames として返す。
    """
    if not motion_list:
        return 0

    # "Frame" キーの最大値を探す
    max_frame = max(entry.get("Frame", 0) for entry in motion_list)
    return max_frame + 1


def process_single_pair(
    beat_file: Path,
    motion_file: Path,
    output_file: Path,
    fps: int = 30
):
    """
    ・beat_file: ビート JSON ファイルパス（beats リストを含む）
    ・motion_file: モーション JSON ファイルパス（上記 get_motion_frame_count が扱える形式）
    ・output_file: 結果を書き出す JSON ファイルパス
    ・fps: フレームレート (デフォルト 30)

    処理手順:
      1. beat_file を読み込み beats リストを取得
      2. calculate_intervals_with_start で frame_intervals, beat_total_frames を得る
      3. motion_file を読み込み motion_list を取得し、get_motion_frame_count で motion_frames を算出
      4. beat_total_frames と motion_frames を比較して差分を最後の要素に足す or 新規追加
      5. 結果 (frame_intervals, beat_total_frames, motion_frames, difference) を output_file に書き出し
    """
    # 1) beat JSON を読み込む
    beat_data = load_json(beat_file)
    beats = beat_data.get("beats", [])
    if not isinstance(beats, list):
        raise ValueError(f"'{beat_file}' の 'beats' がリストではありません。")

    # 2) 4小節ロジック（変更せず）で interval と合計を取得
    frame_intervals, beat_total_frames = calculate_intervals_with_start(beats, fps=fps)

    # 3) motion JSON を読み込んでフレーム数を取得
    motion_data = load_json(motion_file)
    if not isinstance(motion_data, list):
        raise ValueError(f"'{motion_file}' はリスト形式 (FrameキーとPositionを含む) ではありません。")
    motion_frames = get_motion_frame_count(motion_data)

    # 4) 差分を計算し、frame_intervals を調整
    diff = motion_frames - beat_total_frames
    if frame_intervals:
        if abs(diff) < 90:
            frame_intervals[-1] += diff
        else:
            frame_intervals.append(diff)
    else:
        # frame_intervals が空の場合は差分だけを1要素として持つ
        frame_intervals = [diff]

    # 5) 結果を出力
    result = {
        "frame_intervals": frame_intervals
    }
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False)

    print(f"Processed:\n  Beat= {beat_file.name}\n  Motion= {motion_file.name}\n  → Saved result to {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="beat JSON と motion JSON を入力に、"
                    "position==1 の3個目以降を2個おきで4小節ごとの間隔を算出し、"
                    "motion JSON のフレーム数に合わせて調整して出力します。"
    )
    parser.add_argument(
        "--beat",
        type=str,
        required=True,
        help="beat JSON ファイルのパス (beats リストを含む)"
    )
    parser.add_argument(
        "--motion",
        type=str,
        required=True,
        help="motion JSON ファイルのパス (リスト形式で Frame, Position を含む)"
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="出力先 JSON ファイルのパス"
    )

    args = parser.parse_args()

    beat_path = Path(args.beat)
    motion_path = Path(args.motion)
    output_path = Path(args.output)
    fps = 30

    process_single_pair(beat_path, motion_path, output_path, fps=30)
