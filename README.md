# CameraSynthesis

## 必要なファイル

* [Boost 1.87.0](https://www.boost.org/releases/1.87.0/)
* [RapidJson](https://github.com/Tencent/rapidjson/)
* [msgpack for C++](https://github.com/msgpack/msgpack-c/tree/cpp_master)

これを`Library/` ディレクトリの下に配置する。

## 手順
1. [google drive](https://drive.google.com/drive/folders/1Q9HqYq2YPFhb8IbXfEeantl1hpu3jjaV?usp=share_link)からファイルをダウンロードし、`DataBase/` ディレクトリの下に配置する。

2. 以下のコマンドでコンパイルし、実行する。

```.bash
g++ -O3 -march=native -flto -DNDEBUG -std=c++17 -I./Library/rapidjson/include -I./Library/msgpack-c-cpp_master/include -I ./Library/boost_1_87_0 -o camera_synthesis ./main.cpp

./camera_synthesis
```

3. 初期合成なら`initial`、編集したい場合は`modify`と入力する。

4. 質問が順番に表示されるので、()内の選択肢を順番に入力する。

## 可視化
1. 以下のコマンドを用いて`.vmd`形式に変換する。

```.bash
python3 scripts/json2vmd.py \
--json_dir Output/json \
--vmd_dir Output/vmd \
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
