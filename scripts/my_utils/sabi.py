import requests
import json
import os
from pathlib import Path
import argparse


def get_songle_api_url(url, output_dir):
    response = requests.get(url)

    if response.status_code == 200:
        # JSONデータを取得
        data = response.json()
        
        # ローカルファイルに保存
        output_file = os.path.join(output_dir, 'beat.json')
        with open(output_file, "w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False)
        
        print(f"JSONデータを '{output_file}' に保存しました。")
    else:
        print(f"データの取得に失敗しました。ステータスコード: {response.status_code}")

parser = argparse.ArgumentParser()
parser.add_argument('--page', type=str, default='https://songle.jp/songs/songle.jp%2Fuploads%2Fct8m4peiqnulo43nehig.mp3')
parser.add_argument('--output_dir', type=str, default='scripts')
args = parser.parse_args()

if __name__ == "__main__":
    # URLからJSONデータを取得
    page = args.page
    output_dir = args.output_dir
    url = f"https://widget.songle.jp/api/v1/song/chorus.json?url={page}"
    get_songle_api_url(url, output_dir)

    # get_songle_api_url('https://songle.jp/songs/songle.jp%2Fuploads%2Fct8m4peiqnulo43nehig.mp3')
