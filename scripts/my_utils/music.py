import os
import json
from pathlib import Path

import librosa
import numpy as np


FPS = 30
HOP_LENGTH = 512
SR = FPS * HOP_LENGTH


def extract_onset_density(audio_path: Path) -> np.ndarray:
    """
    指定された音声ファイルからオンセット密度を計算し、(n_frames, 1) の NumPy 配列を返す。
    """
    data, _ = librosa.load(str(audio_path), sr=SR)
    envelope = librosa.onset.onset_strength(y=data, sr=SR)  # shape: (n_frames,)
    n_frames = len(envelope)

    onset_frames = librosa.onset.onset_detect(
        y=data, sr=SR, hop_length=HOP_LENGTH
    )

    binary_onset = np.zeros(n_frames, dtype=np.float32)
    for idx in onset_frames:
        if idx < n_frames:
            binary_onset[idx] = 1.0

    window_size = int(SR / HOP_LENGTH)
    if window_size < 1:
        window_size = 1

    onset_density = np.convolve(
        binary_onset,
        np.ones(window_size, dtype=np.float32),
        mode="same",
    )
    onset_density = onset_density[:, None]  # shape: (n_frames, 1)
    return onset_density


def trim_by_motion_frame_count(onset_density: np.ndarray, motion_json_path: Path) -> np.ndarray:
    """
    動作モーション JSON ファイルの中から "Frame" キーの最大値を取得し、
    その最大フレーム + 1 を用いて onset_density を先頭から切り出す。

    motion_json_path には、以下のようなリスト形式の JSON を想定:
    [
      {"Frame": 0, "Position": [...]},
      {"Frame": 1, "Position": [...]},
      ...
    ]
    """
    # JSON を読み込んでリストを取得
    with open(motion_json_path, "r", encoding="utf-8") as f:
        motion_list = json.load(f)

    # 各要素の "Frame" 値を集め、最大値を求める
    frames = [entry.get("Frame", 0) for entry in motion_list]
    if not frames:
        # 空リストなら、返すべきデータがないため空配列を返す
        return np.zeros((0, 1), dtype=np.float32)

    max_frame = max(frames)
    # 切り出す長さは max_frame + 1
    n_keep = max_frame + 1

    trimmed = onset_density[: min(len(onset_density), n_keep), :]
    return trimmed


def save_feature_as_json(feature: np.ndarray, output_path: Path):
    """
    NumPy 配列 feature（shape: (T, D)）をリストに変換し、JSON 形式で保存する。
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(feature.tolist(), f, ensure_ascii=False)
    print(f"Saved trimmed feature to '{output_path}'.")


if __name__ == "__main__":
    """
    使い方:

    python this_script.py \
      --audio       /path/to/song123.mp3 \
      --motion      /path/to/motion.json \
      --output      /path/to/feature.json

    - --audio:  音声ファイル (.wav や .mp3 など) のパス
    - --motion: 動作モーション JSON ファイルのパス (例: m123_gt.json のリスト形式)
    - --output: 切り出した特徴量を保存する JSON ファイルのパス
    """
    import argparse

    parser = argparse.ArgumentParser(
        description="楽曲からオンセット密度を計算し、"
                    "動作モーション JSON の最大 Frame 値＋1 まで切り取って保存します。"
    )
    parser.add_argument(
        "--audio",
        type=str,
        required=True,
        help="楽曲ファイルのパス (.wav/.mp3 など)"
    )
    parser.add_argument(
        "--motion",
        type=str,
        required=True,
        help="動作モーション JSON ファイルのパス（リスト形式で 'Frame' キーを持つ）"
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="出力先 JSON ファイルのパス (切り取った特徴量を保存)"
    )
    args = parser.parse_args()

    audio_path = Path(args.audio)
    motion_json_path = Path(args.motion)
    output_path = Path(args.output)

    # 1) 楽曲からオンセット密度特徴を抽出
    onset_density = extract_onset_density(audio_path)

    # 2) Motion JSON の最大 Frame + 1 に合わせて切り出す
    trimmed_feature = trim_by_motion_frame_count(onset_density, motion_json_path)

    # 3) 結果を JSON で保存
    save_feature_as_json(trimmed_feature, output_path)
