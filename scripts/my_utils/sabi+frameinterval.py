import json
from pathlib import Path
import argparse


def load_json(filepath: Path):
    """
    指定されたパスの JSON ファイルを読み込み、Python オブジェクトとして返す。
    """
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(data, filepath: Path):
    """
    data（辞書やリスト）を JSON 形式で指定されたパスに保存する。
    """
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)


def compute_cumulative_frames(frame_intervals: list) -> list:
    """
    frame_intervals: 各区間のフレーム数を並べたリスト
    戻り値: 各インデックス i における「そのインターバルの終了フレーム」リスト
    例: frame_intervals = [10, 20, 15] -> cumulative_frames = [10, 30, 45]
    """
    cum = []
    total = 0
    for interval in frame_intervals:
        total += interval
        cum.append(total)
    return cum


def collect_chorus_frame_ranges(chorus_data: dict, fps: int) -> list:
    """
    chorus_data に含まれる各リピート(repeats) の start/duration(ms) を
    フレーム単位に変換し、[ (start_frame, end_frame), … ] のリストで返す。

    chorus_data は少なくとも以下のキーを持つことを想定:
      - "chorusSegments": [ { "index": int, "isChorus": bool, "duration": ms, 
                              "repeats": [ {"start": ms, "duration": ms, "index": ...}, … ] }, … ]
      - "repeatSegments": 同様の構造…

    ここでは "chorusSegments" と "repeatSegments" の両方を調べ、すべての repeat を収集する。
    """
    ranges = []

    def add_repeats_list(repeats_list):
        for rep in repeats_list:
            start_ms = rep.get("start")
            duration_ms = rep.get("duration")
            if start_ms is None or duration_ms is None:
                continue
            start_frame = round(start_ms * fps / 1000)
            end_frame = round((start_ms + duration_ms) * fps / 1000)
            ranges.append((start_frame, end_frame))

    # chorusSegments
    for seg in chorus_data.get("chorusSegments", []):
        add_repeats_list(seg.get("repeats", []))

    # repeatSegments
    for seg in chorus_data.get("repeatSegments", []):
        add_repeats_list(seg.get("repeats", []))

    return ranges


def find_sabi_indices(cumulative_frames: list, chorus_ranges: list) -> list:
    """
    cumulative_frames: [f0, f1, f2, …] 各インターバルの終了フレーム番号リスト
    chorus_ranges: [(start_frame, end_frame), …] のリスト

    ■ ロジック:
      各インターバル i（終了フレーム = cumulative_frames[i]）が
      chorus_ranges のいずれかの範囲内に入るなら、その i を含める。

    戻り値: 0-based index のソート済みリスト
    """
    sabi_indices = set()
    for start_f, end_f in chorus_ranges:
        for i, end_frame in enumerate(cumulative_frames):
            if start_f <= end_frame <= end_f:
                sabi_indices.add(i)
    return sorted(sabi_indices)


def merge_and_detect(
    chorus_path: Path,
    frame_path: Path,
    output_path: Path,
    fps: int = 30
):
    """
    1. chorus_path の JSON を読み込み (chorusSegments, repeatSegments…)
    2. frame_path の JSON を読み込み (frame_intervals, beat_total_frames, …)
    3. frame_intervals から cumulative_frames を計算
    4. chorus データからフレーム範囲 (start_frame, end_frame) を収集
    5. cumulative_frames と照合して sabi_indices を算出
    6. 元データをマージし、"sabi_indices" を追加して output_path に保存
    """
    chorus_data = load_json(chorus_path)
    frame_data = load_json(frame_path)

    frame_intervals = frame_data.get("frame_intervals", [])
    if not frame_intervals:
        raise ValueError(f"{frame_path} に 'frame_intervals' が見つからないか空です。")

    # 累積フレームを計算
    cumulative_frames = compute_cumulative_frames(frame_intervals)

    # chorus_data からフレーム範囲を収集
    chorus_ranges = collect_chorus_frame_ranges(chorus_data, fps=fps)

    # sabi_indices を算出
    sabi_indices = find_sabi_indices(cumulative_frames, chorus_ranges)

    # マージ結果を生成
    merged = {}

    # frame_data のキーをコピー
    for key, value in frame_data.items():
        merged[key] = value
    # 新たに sabi_indices を追加
    merged["sabi"] = sabi_indices

    save_json(merged, output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="chorus JSON (chorusSegments, repeatSegments) と frame JSON を読み込み、"
                    "累積フレームとサビ区間からサビに該当するフレームインターバルのインデックスを求めて統合します。"
    )
    parser.add_argument(
        "--chorus",
        type=str,
        required=True,
        help="chorus JSON のパス (例: all_sabi.json)"
    )
    parser.add_argument(
        "--frames",
        type=str,
        required=True,
        help="frame JSON のパス (例: frame_intervals.json)"
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="出力先 JSON のパス (例: combined_with_sabi_indices.json)"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="フレームレート (デフォルト: 30)"
    )
    args = parser.parse_args()

    chorus_path = Path(args.chorus)
    frame_path = Path(args.frames)
    output_path = Path(args.output)
    fps = args.fps

    merge_and_detect(chorus_path, frame_path, output_path, fps=fps)
