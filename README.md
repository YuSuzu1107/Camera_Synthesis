# CameraSynthesis
バーチャルCG／実写のダンス映像において、ユーザーが「注目したい身体部位」「寄りか引きか」「動きの速さ」などの好みを指定すると、データベースから当該ダンス・楽曲に適し、かつユーザーの好みに合ったカメラワークを参照し、最適な視点と軌道を自動合成するシステム。

### ユーザーの好みを反映したカメラワーク
<p align="center">
  <img src="media/03.gif" width="800" alt="01.gif" />
</p>
	
### 実写のダンス映像に対して作成したカメラワーク
<p align="center">
  <img src="media/01.gif" width="400" alt="01.gif" />
  <img src="media/02.gif" width="400" alt="02.gif" />
</p>
 
## 必要なファイル

* [Boost 1.87.0](https://www.boost.org/releases/1.87.0/)
* [RapidJson](https://github.com/Tencent/rapidjson/)
* [msgpack for C++](https://github.com/msgpack/msgpack-c/tree/cpp_master)

これを`Library/` ディレクトリの下に配置する。

## 手順
1. 参照先となるデータベースをDCMデータベースを元に作成し、`DataBase/` ディレクトリの下に配置する。(DCMデータセットは[DancaCamera3D](https://github.com/Carmenw1203/DanceCamera3D-Official)の手順に沿ってダウンロードできる。詳しいデータベースの構築方法は追加予定)

2. 以下のコマンドでコンパイルし、実行する。

```.bash
g++ -O3 -march=native -flto -DNDEBUG -std=c++17 -I./Library/rapidjson/include -I./Library/msgpack-c-cpp_master/include -I ./Library/boost_1_87_0 -o camera_synthesis ./main.cpp

./camera_synthesis intermediate/motion intermediate/music {output_json_dir}
```

3. DCMデータセット内のデータに対してカメラワークを生成したければ`Existing`、新しいデータに対してカメラワークを生成したければ`New`と入力する。

4. 初期合成なら`initial`、編集したい場合は`modify`と入力する。

5. 編集モードの場合は質問が順番に表示されるので、()内の選択肢を自分の好む方を選んで入力する。

## 既存データ(バーチャルCG)に対してカメラワーク生成をする場合

1. Raw,Stand_Raw,Hip,Beats,Music_Featuresの中から共通する番号を選んで複製し、以下のようにパスを変更する。

```.bash
Raw内のデータ  　　　　　　  →　　intermediate/motion/raw.msgpack
Stand_Raw内のデータ　　　　 →   intermediate/motion/stand.msgpack
Hip内のデータ　　　　　　　　 →   intermediate/motion/hip.msgpack

Beats内のデータ　　　　　　  →   intermediate/music/beat.msgpack
Music_Features内のデータ  →   intermediate/music/music.msgpack
```

2. ファイルを実行した際に、「番号を入力してください」と出るので、自分が選んだ番号を入れ、カメラワークを出力する。

## 実写データ(ボリュメトリックビデオのデータが必要)に対してカメラワークを生成する場合

### モーションデータの準備

1. Blenderに [MoCapade](https://me.meshcapade.com/from-videos) の出力結果をインポートする。

2. bvh形式でrootをエクスポートし、その結果を再びインポートする。

3. IKボーンを全て削除し、Auto Rig Pro のリマップ機能を用いて元モーションとスケールを合わせ、bvh形式で出力する。

4. `input/`に出力のbvhを配置し、`raw.bvh`とする。

5. 以下のコマンドを用いてjson形式からmessagepack形式に変換するコードをコンパイルする。
   
```.bash
g++ -std=c++17 -O2 \
    -I./Library/rapidjson/include \
    -I./Library/msgpack-c-cpp_master/include \
    -I./Library/boost_1_87_0 \
    ./scripts/json2msgpack.cpp \
    -o ./scripts/json2msgpack
```

6. 以下のコマンドを実行して入力ファイルを出力する。

```.bash
bash scripts/make_motion_input.sh 
```

### 音楽データの準備

1. [Songle](https://songle.jp) のマイページから入力したい楽曲を登録する

2. `scripts/make_motion_input.sh`の以下の部分に楽曲解析ページのURLを入れる。

```.bash
# ここに入力してください
python3 scripts/my_utils/songle_api.py \
--page  [SONGLE_URL]\
--output_dir intermediate/music

# ここに入力してください
python3 scripts/my_utils/sabi.py \
--page  [SONGLE_URL]\
--output_dir input
```

3. 楽曲データを`input/`に配置する。

4. 以下のコマンドを実行して入力ファイルを出力する。

```.bash
bash scripts/make_music_input.sh 
```

## 可視化
1. 以下のコマンドを用いて`.vmd`形式に変換する。

```.bash
python3 scripts/json2vmd.py \
--json_dir {output_json_dir} \
--vmd_dir {output_vmd_dir} \
--data_type camera
```

2. [Saba_Viewer](https://github.com/benikabocha/saba) を用いて可視化を行う。その際、モデルは [符华 from 神帝宇](https://www.aplaybox.com/u/359396473?searchKeyword=符华) に似たものを選ぶと良い。
   また、[Saba_Viewer](https://github.com/benikabocha/saba) の使用にあたって、2点ほどソースコードを変更する必要がある。

1点目
```.c
////saba-master/src/Saba/Model/MMD/VMDFile.h
//uint32_t		m_viewAngle;//for original camera .vmd file
float		m_viewAngle;//for synthesized camera .vmd file
```

2点目
```.c
////saba-master/src/Saba/Model/MMD/VMDCameraAnimation.cpp
//Comment out the following code from line 56~89
// if ((key1.m_time - key0.m_time) > 1)
// {
// 	float timeRange = float(key1.m_time - key0.m_time);
// ...
// 	m_camera.m_distance = key0.m_distance;
// 	m_camera.m_fov = key0.m_fov;
// }

//Add the following code
float timeRange = float(key1.m_time - key0.m_time);
float time = (t - float(key0.m_time)) / timeRange;
float ix_x = key0.m_ixBezier.FindBezierX(time);
float iy_x = key0.m_iyBezier.FindBezierX(time);
float iz_x = key0.m_izBezier.FindBezierX(time);
float rotate_x = key0.m_rotateBezier.FindBezierX(time);
float distance_x = key0.m_distanceBezier.FindBezierX(time);
float fov_x = key0.m_fovBezier.FindBezierX(time);

float ix_y = key0.m_ixBezier.EvalY(ix_x);
float iy_y = key0.m_iyBezier.EvalY(iy_x);
float iz_y = key0.m_izBezier.EvalY(iz_x);
float rotate_y = key0.m_rotateBezier.EvalY(rotate_x);
float distance_y = key0.m_distanceBezier.EvalY(distance_x);
float fov_y = key0.m_fovBezier.EvalY(fov_x);

m_camera.m_interest = glm::mix(key0.m_interest, key1.m_interest, glm::vec3(ix_y, iy_y, iz_y));
m_camera.m_rotate = glm::mix(key0.m_rotate, key1.m_rotate, rotate_y);
m_camera.m_distance = glm::mix(key0.m_distance, key1.m_distance, distance_y);
m_camera.m_fov = glm::mix(key0.m_fov, key1.m_fov, fov_y);
```

3. Blenderで可視化する場合。Blenderのバージョンが3.Xであることを確認し、[google drive](https://drive.google.com/drive/folders/1BaH2MdcObcDVfT_ahRbwboj3qJmf8UQS?usp=sharing)から`mmd_tools`をダウンロードしアドオンに追加する。

## 謝辞
Saba_Viewerを提供してくださったbenikabocha氏、モデルを提供してくださった神帝宇氏、DCMデータセットを提供してくださったWang氏に心から感謝します。

