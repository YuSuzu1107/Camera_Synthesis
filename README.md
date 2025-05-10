# CameraSynthesis

## 必要なファイル

* Boost 1.87.0
* RapidJson

これを`Library/` ディレクトリの下に配置する。

## 手順
1. 以下のgoogle driveのリンクからファイルをダウンロードし、`DataBase/` ディレクトリの下に配置する。

```.bash
https://drive.google.com/drive/folders/1Q9HqYq2YPFhb8IbXfEeantl1hpu3jjaV?usp=share_link
```

2. 以下のコマンドでコンパイルし、実行する。

```.bash
g++ -O3 -march=native -flto -DNDEBUG -std=c++17 -I./Library/rapidjson/include -I./Library/msgpack-c-cpp_master/include -I ./Library/boost_1_87_0 -o camera_synthesis ./main.cpp

./camera_synthesis
```

3. 初期合成なら`initial`と入力する。編集したい場合は`modify`と入力し、()内の選択肢を順番に入力する。

## 可視化
1. 以下のコマンドを用いて`.vmd`形式に変換する。

```.bash
python3 scripts/json2vmd.py \
--json_dir Output/json \
--vmd_dir Output/vmd \
--data_type camera
```
