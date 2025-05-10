#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <map>
#include <algorithm>
#include <filesystem>
#include <cassert>

// MessagePack のヘッダ
#include <msgpack.hpp>

// RapidJSON 関連
#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"

using namespace std;
namespace fs = std::filesystem;

// 簡易的な構造体定義
struct FrameData {
    // 各ジョイントの Position
    vector<array<double, 3>> positions;
    // ヒップのクォータニオン（4要素）
    array<double, 4> hipQuaternion;
};

// MessagePack のヘルパー関数
msgpack::object_handle readMsgpack(const string &path) {
    ifstream ifs(path, ios::binary);
    vector<char> buffer((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
    return msgpack::unpack(buffer.data(), buffer.size());
}

const msgpack::object* getMember(const msgpack::object &obj, const string &key) {
    if(obj.type != msgpack::type::MAP)
        return nullptr;
    for (size_t i = 0; i < obj.via.map.size; i++) {
        const msgpack::object &k = obj.via.map.ptr[i].key;
        if (k.type == msgpack::type::STR) {
            string kstr(k.via.str.ptr, k.via.str.size);
            if (kstr == key)
                return &obj.via.map.ptr[i].val;
        }
    }
    return nullptr;
}

// MessagePack を用いた joint_positions の読み込み関数
vector<FrameData> loadJointPositions(const string &msgpackFilePath) {
    vector<FrameData> frames;
    msgpack::object_handle oh = readMsgpack(msgpackFilePath);
    msgpack::object obj = oh.get();
    for (size_t i = 0; i < obj.via.array.size; i++) {
        msgpack::object frameObj = obj.via.array.ptr[i];
        if(frameObj.type != msgpack::type::MAP)
            continue;
        FrameData fd;

        // デバック用
        // const msgpack::object* posObj = getMember(frameObj, "Position");
        // if (!posObj) {
        //     cerr << "デバッグ: フレーム " << i << " でキー 'Position' が見つかりませんでした。" << endl;
        // } else {
        //     if (posObj->type != msgpack::type::ARRAY) {
        //         cerr << "デバッグ: フレーム " << i << " の 'Position' は ARRAY ではありません。型コード: " 
        //              << posObj->type << endl;
        //     } else {
        //         cout << "デバッグ: フレーム " << i << " の 'Position' キーが見つかりました。内容: " 
        //              << *posObj << endl;
        //     }
        // }

        // "Position" キーから各ジョイントの位置を取得
        const msgpack::object* posObj = getMember(frameObj, "Position");
        if (posObj && posObj->type == msgpack::type::ARRAY) {
            for (size_t j = 0; j < posObj->via.array.size; j++) {
                msgpack::object joint = posObj->via.array.ptr[j];
                if (joint.type == msgpack::type::ARRAY && joint.via.array.size >= 3) {
                    array<double, 3> p;
                    p[0] = joint.via.array.ptr[0].as<double>();
                    p[1] = joint.via.array.ptr[1].as<double>();
                    p[2] = joint.via.array.ptr[2].as<double>();
                    fd.positions.push_back(p);
                }
            }
        }
        // "HipRotationQuaternion" キーからヒップ回転（クォータニオン）を取得
        const msgpack::object* hipObj = getMember(frameObj, "HipRotationQuaternion");
        if (hipObj && hipObj->type == msgpack::type::ARRAY && hipObj->via.array.size >= 4) {
            fd.hipQuaternion[0] = hipObj->via.array.ptr[0].as<double>();
            fd.hipQuaternion[1] = hipObj->via.array.ptr[1].as<double>();
            fd.hipQuaternion[2] = hipObj->via.array.ptr[2].as<double>();
            fd.hipQuaternion[3] = hipObj->via.array.ptr[3].as<double>();
        }
        frames.push_back(fd);
    }
    return frames;
}

// JSONに依存しない計算処理
double calculateJointDistanceSparse(const vector<FrameData> &frames1,
                                    const vector<FrameData> &frames2,
                                    int step) {
    double total_distance = 0.0;
    int minLen = min(frames1.size(), frames2.size());
    for (int i = 0; i < minLen; i += step) {
        const auto &joints1 = frames1[i].positions;
        const auto &joints2 = frames2[i].positions;
        int numJoints = min(joints1.size(), joints2.size());
        double frameDistance = 0.0;
        for (int j = 0; j < numJoints; j++) {
            double dx = joints1[j][0] - joints2[j][0];
            double dy = joints1[j][1] - joints2[j][1];
            double dz = joints1[j][2] - joints2[j][2];
            frameDistance += sqrt(dx * dx + dy * dy + dz * dz);
        }
        total_distance += frameDistance;
    }
    return total_distance;
}

double calculateHipVectorDistanceSparse(const vector<FrameData> &frames1,
                                        const vector<FrameData> &frames2,
                                        int step) {
    double total_distance = 0.0;
    int minLen = min(frames1.size(), frames2.size());
    for (int i = 0; i < minLen; i += step) {
        const auto &q1 = frames1[i].hipQuaternion;
        const auto &q2 = frames2[i].hipQuaternion;
        double dx = q1[0] - q2[0];
        double dy = q1[1] - q2[1];
        double dz = q1[2] - q2[2];
        double dw = q1[3] - q2[3];
        total_distance += sqrt(dx * dx + dy * dy + dz * dz + dw * dw);
    }
    return total_distance;
}

vector<array<double, 3>> applyGaussianFilter(const vector<array<double, 3>> &data, double sigma) {
    int kernelSize = max(3, (int)ceil(6.0 * sigma));
    if (kernelSize % 2 == 0)
        kernelSize += 1;
    vector<double> kernel(kernelSize);
    int half = kernelSize / 2;
    double sum = 0.0;
    double invS2 = 1.0 / (2.0 * sigma * sigma);
    for (int i = 0; i < kernelSize; i++) {
        int x = i - half;
        double val = exp(-x * x * invS2);
        kernel[i] = val;
        sum += val;
    }
    for (int i = 0; i < kernelSize; i++) {
        kernel[i] /= sum;
    }
    int n = data.size();
    vector<array<double, 3>> smoothed(n, {0.0, 0.0, 0.0});
    for (int i = 0; i < n; i++) {
        double outx = 0.0, outy = 0.0, outz = 0.0;
        for (int k = 0; k < kernelSize; k++) {
            int index = i + (k - half);
            if (index < 0)
                index = 0;
            if (index >= n)
                index = n - 1;
            outx += data[index][0] * kernel[k];
            outy += data[index][1] * kernel[k];
            outz += data[index][2] * kernel[k];
        }
        smoothed[i] = {outx, outy, outz};
    }
    return smoothed;
}

vector<double> normalizeValues(const vector<double> &vals) {
    vector<double> out;
    if (vals.empty())
        return out;
    double minV = vals[0], maxV = vals[0];
    for (auto v : vals) {
        if (v < minV)
            minV = v;
        if (v > maxV)
            maxV = v;
    }
    double range = maxV - minV;
    out.resize(vals.size());
    if (range == 0.0) {
        fill(out.begin(), out.end(), 0.0);
        return out;
    }
    for (size_t i = 0; i < vals.size(); i++) {
        out[i] = (vals[i] - minV) / range;
    }
    return out;
}

vector<vector<FrameData>> splitByFrameIntervals(const vector<FrameData> &data,
                                                 const vector<int> &frameIntervals) {
    vector<vector<FrameData>> segments;
    int start = 0;
    int n = data.size();
    for (auto interval : frameIntervals) {
        int end = start + interval;
        if (end > n)
            end = n;
        vector<FrameData> segment;
        for (int i = start; i < end; i++) {
            segment.push_back(data[i]);
        }
        segments.push_back(segment);
        start = end;
        if (start >= n)
            break;
    }
    return segments;
}

// 距離の平均値算出
double getDistanceAverageForCandidateMsgpack(const string &candidateFile,
                                               int lengthFrames,
                                               const string &cameraPositionMsgpackDir) {
    string baseName = candidateFile;
    if (baseName.size() > 7 && baseName.substr(baseName.size() - 7) == ".msgpack") {
        baseName = baseName.substr(0, baseName.size() - 7);
    }
    size_t underscorePos = baseName.find('_');
    if (underscorePos == string::npos)
        return 0.0;
    string firstPart = baseName.substr(0, underscorePos);
    string secondPart = baseName.substr(underscorePos + 1);
    string fileNumberStr = firstPart.substr(1);
    if (!secondPart.empty() && secondPart.front() == '(')
        secondPart.erase(0, 1);
    if (!secondPart.empty() && secondPart.back() == ')')
        secondPart.pop_back();
    size_t commaPos = secondPart.find(',');
    if (commaPos == string::npos)
        return 0.0;
    int segStart = stoi(secondPart.substr(0, commaPos));
    int segEnd = stoi(secondPart.substr(commaPos + 1));
    
    string cameraPositionFile = cameraPositionMsgpackDir + "/c" + fileNumberStr + ".msgpack";
    msgpack::object_handle oh = readMsgpack(cameraPositionFile);
    msgpack::object obj = oh.get();
    const msgpack::object* distanceObj = getMember(obj, "Distance");
    if (!distanceObj || distanceObj->type != msgpack::type::ARRAY)
        return 0.0;
    int totalSize = distanceObj->via.array.size;
    int startIndex = segStart;
    int endIndex = segStart + lengthFrames;
    if (startIndex < 0)
        startIndex = 0;
    if (endIndex > totalSize)
        endIndex = totalSize;
    if (endIndex <= startIndex)
        return 0.0;
    double sumDist = 0.0;
    int count = 0;
    for (int i = startIndex; i < endIndex; i++) {
        double d = distanceObj->via.array.ptr[i].as<double>();
        sumDist += d;
        count++;
    }
    return (count == 0) ? 0.0 : sumDist / count;
}

double framesToMilliseconds(int frames, int fps = 30) {
    return (double(frames) / fps) * 1000.0;
}

// BPM の平均値算出
double calculateAverageBpmInIntervalMsgpack(const msgpack::object &beats, int startFrame, int endFrame, int fps = 30) {
    double startMs = framesToMilliseconds(startFrame, fps);
    double endMs = framesToMilliseconds(endFrame, fps);
    double sumBpm = 0.0;
    int cnt = 0;
    if (beats.type != msgpack::type::ARRAY)
        return 0.0;
    for (size_t i = 0; i < beats.via.array.size; i++) {
        msgpack::object beatObj = beats.via.array.ptr[i];
        const msgpack::object* startObj = getMember(beatObj, "start");
        const msgpack::object* bpmObj = getMember(beatObj, "bpm");
        if (!startObj || !bpmObj)
            continue;
        double beatStart = startObj->as<double>();
        if (beatStart >= startMs && beatStart < endMs) {
            double bpmVal = bpmObj->as<double>();
            sumBpm += bpmVal;
            cnt++;
        }
    }
    return (cnt == 0) ? 0.0 : sumBpm / cnt;
}

// ファイル名から (file_number, start_frame, end_frame) を抽出
bool parseSegmentFilename(const string &filename, string &outFileNumber, int &outStart, int &outEnd) {
    string name = filename;
    if (name.size() > 7 && name.substr(name.size() - 7) == ".msgpack") {
        name = name.substr(0, name.size() - 7);
    }
    size_t underscorePos = name.find('_');
    if (underscorePos == string::npos)
        return false;
    string part1 = name.substr(0, underscorePos);
    string part2 = name.substr(underscorePos + 1);
    if (part1.size() <= 1)
        return false;
    outFileNumber = part1.substr(1);
    if (!part2.empty() && part2.front() == '(')
        part2.erase(0, 1);
    if (!part2.empty() && part2.back() == ')')
        part2.pop_back();
    size_t commaPos = part2.find(',');
    if (commaPos == string::npos)
        return false;
    outStart = stoi(part2.substr(0, commaPos));
    outEnd = stoi(part2.substr(commaPos + 1));
    return true;
}

// BPM 値を取得する
double getBpmFromBpmMsgpack(const string &bpmMsgpackPath,
                             const string &fileNumberStr,
                             int startFrame,
                             int endFrame) {
    msgpack::object_handle oh = readMsgpack(bpmMsgpackPath);
    msgpack::object obj = oh.get();
    const msgpack::object* fileObj = getMember(obj, fileNumberStr);
    if (!fileObj || fileObj->type != msgpack::type::ARRAY)
        return 0.0;
    for (size_t i = 0; i < fileObj->via.array.size; i++) {
        msgpack::object intervalObj = fileObj->via.array.ptr[i];
        const msgpack::object* fr = getMember(intervalObj, "interval_frames");
        if (!fr || fr->type != msgpack::type::ARRAY || fr->via.array.size < 2)
            continue;
        int startVal = fr->via.array.ptr[0].as<int>();
        int endVal = fr->via.array.ptr[1].as<int>();
        if (startVal == startFrame && endVal == endFrame) {
            const msgpack::object* avgBpmObj = getMember(intervalObj, "average_bpm");
            if (avgBpmObj)
                return avgBpmObj->as<double>();
        }
    }
    return 0.0;
}

// 指定した msgpack オブジェクトから start ～ end (end は除く) の音楽特徴量シーケンスを抽出する関数
vector<vector<double>> extractMusicFeatureSegment(const msgpack::object &musicObj, int start, int end) {
    vector<vector<double>> segment;
    int total = musicObj.via.array.size;
    for (int i = start; i < end && i < total; i++) {
        const msgpack::object &frameObj = musicObj.via.array.ptr[i];
        // 次元数
        if (frameObj.type == msgpack::type::ARRAY && frameObj.via.array.size == 1) {
            vector<double> vec(1);
            for (int k = 0; k < 1; k++) {
                vec[k] = frameObj.via.array.ptr[k].as<double>();
            }
            segment.push_back(vec);
        } else {
            cerr << "Warning: Frame " << i << " is not a valid 1-dim vector." << endl;
        }
    }
    return segment;
}

// フレームごとに、入力セグメントと候補セグメントの1次元ベクトルの差分を計算する。
// step 間隔でサンプルし、各次元の差分を足し合わせたものを返す（各要素は各次元の総和）。
// 次元数
vector<double> calculateMusicFeatureDistanceSparse(const vector<vector<double>> &inputSegment,
    const vector<vector<double>> &candidateSegment,
    int step) {
int n = min(inputSegment.size(), candidateSegment.size());
vector<double> diffSum(1, 0.0);
for (int i = 0; i < n; i += step) {
for (int k = 0; k < 1; k++) {
diffSum[k] += fabs(inputSegment[i][k] - candidateSegment[i][k]);
}
}
return diffSum;
}

// カメラ位置の平均（移動距離）を取得する
double getPositionAverageForCandidateMsgpack(const string &candidateFile,
                                               int segmentLen,
                                               const string &cameraPositionMsgpackDir) {
    string fileNumberStr;
    int segStart = 0, segEnd = 0;
    if (!parseSegmentFilename(candidateFile, fileNumberStr, segStart, segEnd)) {
        cerr << "parseSegmentFilename失敗: " << candidateFile << endl;
        return 0.0;
    }
    string cameraDataFile = cameraPositionMsgpackDir + "/c" + fileNumberStr + ".msgpack";
    msgpack::object_handle oh = readMsgpack(cameraDataFile);
    msgpack::object obj = oh.get();
    const msgpack::object* cameraEyeObj = getMember(obj, "camera_eye");
    if (!cameraEyeObj || cameraEyeObj->type != msgpack::type::ARRAY) {
        cerr << "camera_eye がありません: " << cameraDataFile << endl;
        return 0.0;
    }
    int totalSize = cameraEyeObj->via.array.size;
    int startIndex = segStart;
    int endIndex = segStart + segmentLen;
    if (startIndex < 0)
        startIndex = 0;
    if (endIndex > totalSize)
        endIndex = totalSize;
    if (endIndex <= startIndex)
        return 0.0;
    array<double, 3> firstPos = {0.0, 0.0, 0.0};
    array<double, 3> lastPos = {0.0, 0.0, 0.0};
    msgpack::object firstFrame = cameraEyeObj->via.array.ptr[startIndex];
    if (firstFrame.type == msgpack::type::ARRAY && firstFrame.via.array.size >= 3) {
        firstPos[0] = firstFrame.via.array.ptr[0].as<double>();
        firstPos[1] = firstFrame.via.array.ptr[1].as<double>();
        firstPos[2] = firstFrame.via.array.ptr[2].as<double>();
    }
    msgpack::object lastFrame = cameraEyeObj->via.array.ptr[endIndex - 1];
    if (lastFrame.type == msgpack::type::ARRAY && lastFrame.via.array.size >= 3) {
        lastPos[0] = lastFrame.via.array.ptr[0].as<double>();
        lastPos[1] = lastFrame.via.array.ptr[1].as<double>();
        lastPos[2] = lastFrame.via.array.ptr[2].as<double>();
    }
    double dx = lastPos[0] - firstPos[0];
    double dy = lastPos[1] - firstPos[1];
    double dz = lastPos[2] - firstPos[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

int getIntervalIndex(int frameNumber, const vector<int> &frameIntervals) {
    int cumulative = 0;
    for (size_t i = 0; i < frameIntervals.size(); i++) {
        cumulative += frameIntervals[i];
        if (frameNumber <= cumulative)
            return i;
    }
    return -1;
}

vector<int> convertFramesToIndices(const vector<int> &frameNumbers, const vector<int> &frameIntervals) {
    vector<int> indices;
    for (auto fn : frameNumbers) {
        int idx = getIntervalIndex(fn, frameIntervals);
        if (idx != -1)
            indices.push_back(idx);
        else
            cerr << "警告: フレーム " << fn << " は全インターバルを超えています" << endl;
    }
    return indices;
}

// メインの類似ファイル検索
struct CalDistance2Result {
    vector<string> closestFiles;   // 各セグメントで選ばれたファイル
    vector<int> lengths;           // 各セグメントの長さ
    string inputNumber;            // 入力モーションの番号
    vector<array<double, 3>> translations; // 全フレーム分の平行移動
};

CalDistance2Result calDistance2Msgpack(const string &bpmMsgpackPath,
                                       const string &musicMsgpackDir, 
                                       const string &musicDatabaseMsgpackDir,
                                       const string &inputBvh, 
                                       const string &standMsgpackDir,
                                       const string &standDatabaseMsgpackDir,
                                       const string &hipDirectionMsgpackDir,
                                       const string &hipDirectionDatabaseMsgpackDir,
                                       const string &rawMsgpackDir,
                                       const string &databaseMsgpackDir,
                                       const vector<int> &frameIntervals,
                                       int step,
                                       const vector<int> &modes,
                                       const string &cameraPositionMsgpackDir,
                                       int mParam) {
    CalDistance2Result result;
    // 入力モーション番号の抽出
    string inputName = inputBvh;
    size_t pos = inputName.find_last_of("/\\");
    if (pos != string::npos)
        inputName = inputName.substr(pos + 1);
    if (inputName.size() > 4 && inputName.substr(inputName.size() - 4) == ".bvh")
        inputName = inputName.substr(0, inputName.size() - 4);
    string inputNumber = (inputName.size() > 1) ? inputName.substr(1) : inputName;
    result.inputNumber = inputNumber;
    
    // 入力モーションの BPM ファイル読み込み
    string beatFileForInput = "scripts/Yu/Database/msg_beats/beat" + inputNumber + ".msgpack";
    msgpack::object_handle beatOh = readMsgpack(beatFileForInput);
    msgpack::object beatObj = beatOh.get();
    const msgpack::object* beatsMember = getMember(beatObj, "beats");

    // --- 入力側音楽特徴量の読み込み ---
    string inputMusicFile = musicMsgpackDir + "/a" + inputNumber + ".msgpack";
    msgpack::object_handle inputMusicOh = readMsgpack(inputMusicFile);
    msgpack::object inputMusicObj = inputMusicOh.get();
    if (inputMusicObj.type != msgpack::type::ARRAY) {
        cerr << "Error: Input music feature file " << inputMusicFile
             << " does not contain an array." << endl;
    }

    // 入力側の音楽特徴量をセグメントごとに抽出する
    vector< vector<vector<double>> > inputMusicSegments;
    vector<double> inputBpmList;

    int segStartFrame = 0;
    for (auto segLen : frameIntervals) {
        int segEndFrame = segStartFrame + segLen;

        double avgBpm = 0.0;
        if (beatsMember && beatsMember->type == msgpack::type::ARRAY) {
            avgBpm = calculateAverageBpmInIntervalMsgpack(*beatsMember, segStartFrame, segEndFrame, 30);
        }
        inputBpmList.push_back(avgBpm);

        vector<vector<double>> seg = extractMusicFeatureSegment(inputMusicObj, segStartFrame, segEndFrame);
        inputMusicSegments.push_back(seg);

        segStartFrame = segEndFrame;
    }
    
    // 入力モーションの標準化データとヒップ方向データの読み込み
    string inputStandMsgpackPath = standMsgpackDir + "/m" + inputNumber + ".msgpack";
    vector<FrameData> inputPositions = loadJointPositions(inputStandMsgpackPath);
    string inputHipMsgpackPath = hipDirectionMsgpackDir + "/m" + inputNumber + ".msgpack";
    vector<FrameData> inputHipDirections = loadJointPositions(inputHipMsgpackPath);
    
    // Raw データの読み込み
    string rawInputPath = rawMsgpackDir + "/m" + inputNumber + ".msgpack";
    vector<FrameData> rawInputFrames = loadJointPositions(rawInputPath);
    vector<vector<FrameData>> rawInputSegments = splitByFrameIntervals(rawInputFrames, frameIntervals);
    vector<vector<FrameData>> inputSegments = splitByFrameIntervals(inputPositions, frameIntervals);
    vector<vector<FrameData>> hipSegments = splitByFrameIntervals(inputHipDirections, frameIntervals);
    
    // 各セグメントごとに類似ファイルを検索
    for (size_t segIndex = 0; segIndex < inputSegments.size(); segIndex++) {
        const auto &inputSegment = inputSegments[segIndex];
        const auto &rawSegment = rawInputSegments[segIndex];
        const auto &hipSegment = hipSegments[segIndex];
        int segmentLen = inputSegment.size();
        double segmentBpmInput = (segIndex < inputBpmList.size()) ? inputBpmList[segIndex] : 0.0;
        // 入力側の音楽特徴量シーケンス（フレームごと3次元ベクトル）
        const vector<vector<double>> &inputMusicSegment = inputMusicSegments[segIndex];

        vector<double> segmentDistances;
        vector<double> hipDistances;
        vector<double> bpmDiffs;
        vector<string> fileNames;
        vector<vector<double>> candidateFeatureDiffs;

        // データベースディレクトリ内の各ファイルを走査
        for (const auto &entry : fs::directory_iterator(standDatabaseMsgpackDir)) {
            if (!entry.is_regular_file())
                continue;
            string fname = entry.path().filename().string(); // 例："m62_(0,550).msgpack"

            if (fname.find("m" + inputNumber + "_") == 0)
                continue;
            if (fname.size() <= 8 || fname.substr(fname.size() - 8) != ".msgpack")
                continue;
            string dbFilePath = entry.path().string();
            vector<FrameData> dbPositions = loadJointPositions(dbFilePath);

            // ここでデバッグ出力：候補ファイルのフレーム数と現在のセグメントの長さを出力
            // cout << "候補ファイル " << fname << " のフレーム数: " 
            // << dbPositions.size() << ", セグメントの長さ: " << segmentLen << "\n";
            
            if (dbPositions.size() < (size_t)segmentLen)
                continue;
            // ヒップ方向データの取得
            string dbFileNumberStr;
            int dbStart = 0, dbEnd = 0;
            if (!parseSegmentFilename(fname, dbFileNumberStr, dbStart, dbEnd))
                continue;
            string dbHipFilePath = hipDirectionDatabaseMsgpackDir + "/m" + dbFileNumberStr + "_(" +
                                    to_string(dbStart) + ", " + to_string(dbEnd) + ").msgpack";
            vector<FrameData> dbHipPositions = loadJointPositions(dbHipFilePath);
            if (dbHipPositions.size() < (size_t)segmentLen)
                continue;
            double segDist = calculateJointDistanceSparse(inputSegment,
                             vector<FrameData>(dbPositions.begin(), dbPositions.begin() + segmentLen), step);
            double hipDist = calculateHipVectorDistanceSparse(hipSegment,
                             vector<FrameData>(dbHipPositions.begin(), dbHipPositions.begin() + segmentLen), step);
            double dbBpmVal = getBpmFromBpmMsgpack(bpmMsgpackPath, dbFileNumberStr, dbStart, dbEnd);
            double bpmDiff = fabs(segmentBpmInput - dbBpmVal);
            segmentDistances.push_back(segDist);
            hipDistances.push_back(hipDist);
            bpmDiffs.push_back(bpmDiff);
            fileNames.push_back(fname);
            // 楽曲特徴量の差分計算
            // 候補側は musicDatabaseDir 内の "a[dbFileNumberStr].msgpack" から、対象区間のシーケンスを抽出
            string candidateMusicFile = musicDatabaseMsgpackDir + "/m" + dbFileNumberStr + "_(" +
                                   to_string(dbStart) + "," + to_string(dbEnd) + ").msgpack";
            msgpack::object_handle candidateMusicOh = readMsgpack(candidateMusicFile);
            msgpack::object candidateMusicObj = candidateMusicOh.get();
            vector<vector<double>> candidateMusicSegment = extractMusicFeatureSegment(candidateMusicObj, dbStart, dbEnd);
            vector<double> diffVec = calculateMusicFeatureDistanceSparse(inputMusicSegment, candidateMusicSegment, step);
            candidateFeatureDiffs.push_back(diffVec);
        }
        vector<double> normSegDist = normalizeValues(segmentDistances);
        vector<double> normHipDist = normalizeValues(hipDistances);
        vector<double> normBpmDiff = normalizeValues(bpmDiffs);

        // 各次元ごとに、各候補の楽曲特徴量差分を正規化
        // 次元数
        int numCandidates = candidateFeatureDiffs.size();
        vector<double> featureScores(numCandidates, 0.0);
        for (int k = 0; k < 1; k++) {
            vector<double> col;
            for (int i = 0; i < numCandidates; i++) {
                col.push_back(candidateFeatureDiffs[i][k]);
            }
            vector<double> normCol = normalizeValues(col);
            for (int i = 0; i < numCandidates; i++) {
                candidateFeatureDiffs[i][k] = normCol[i];
            }
        }
        // 各候補の正規化済み3次元差分を総和してスカラーに
        // 次元数
        for (int i = 0; i < numCandidates; i++) {
            double score = 0.0;
            for (int k = 0; k < 1; k++) {
                score += candidateFeatureDiffs[i][k];
            }
            featureScores[i] = score;
        }
        vector<double> normFeatureScore = normalizeValues(featureScores);
 
        // 類似度の重み付け
        // 変更箇所
        double weight_motion = 1, weight_music = 5;

        vector<pair<string, double>> scores;
        for (size_t i = 0; i < fileNames.size(); i++) {
            double s = weight_motion * (normSegDist[i] + normHipDist[i]) + weight_music * (normFeatureScore[i] + normBpmDiff[i]);
            scores.push_back({fileNames[i], s});
        }
        sort(scores.begin(), scores.end(), [](auto &a, auto &b) { return a.second < b.second; });
        int top_n = scores.size() < 5 ? scores.size() : 5;
        cout << "----- Top 5 candidates for segment " << segIndex << " -----\n";
        for (int i = 0; i < top_n; i++) {
            cout << "   Rank " << (i + 1) << ": " << scores[i].first
                 << " Score=" << scores[i].second << "\n";
        }
        string chosenFile;
        double min_score = 0.0;
        int currentMode = modes[segIndex];  // セグメントごとの mode

        if (currentMode == 1) {
            // Mode 1: 俯瞰視点 (引き)
            // Distance の平均が最も小さい候補を採用
            double min_distance_val = std::numeric_limits<double>::infinity();
            std::string best_file;
            double best_score = 0.0;
            for (int i = 0; i < top_n; i++) {
                std::string candidate_file = scores[i].first;
                double candidate_score = scores[i].second;
                double avg_dist = getDistanceAverageForCandidateMsgpack(candidate_file, segmentLen, cameraPositionMsgpackDir);
                if (avg_dist < min_distance_val) {
                    min_distance_val = avg_dist;
                    best_file = candidate_file;
                    best_score = candidate_score;
                }
            }
            chosenFile = best_file;
            min_score = best_score;
            std::cout << "[Selected file (引き)] " << chosenFile 
                      << " with DistanceAvg = " << min_distance_val 
                      << ", Score = " << min_score << std::endl;

        } else if (currentMode == 2) {
            // Mode 2: 寄り視点
            // Distance の平均が最も大きい候補を採用 (ただし avg_dist < -5 の条件付き)
            double max_distance_val = -std::numeric_limits<double>::infinity();
            std::string best_file;
            double best_score = 0.0;
            for (int i = 0; i < top_n; i++) {
                std::string candidate_file = scores[i].first;
                double candidate_score = scores[i].second;
                double avg_dist = getDistanceAverageForCandidateMsgpack(candidate_file, segmentLen, cameraPositionMsgpackDir);
                if (avg_dist > max_distance_val && avg_dist < -5) {
                    max_distance_val = avg_dist;
                    best_file = candidate_file;
                    best_score = candidate_score;
                }
            }
            chosenFile = best_file;
            min_score = best_score;
            std::cout << "[Selected file (寄り)] " << chosenFile 
                    << " with DistanceAvg = " << max_distance_val 
                    << ", Score = " << min_score << std::endl;
        
        } else if (currentMode == 3) {
            // Mode 3: 動きが多いカメラワーク
            // Camera Movement (カメラ位置の移動距離) が最も大きい候補を採用
            double max_camera_movement = -1.0;
            std::string best_file;
            double best_score = 0.0;
            for (int i = 0; i < top_n; i++) {
                std::string candidate_file = scores[i].first;
                double candidate_score = scores[i].second;
                double movement_distance = getPositionAverageForCandidateMsgpack(candidate_file, segmentLen, cameraPositionMsgpackDir);
                if (movement_distance > max_camera_movement) {
                    max_camera_movement = movement_distance;
                    best_file = candidate_file;
                    best_score = candidate_score;
                }
            }
            chosenFile = best_file;
            min_score = best_score;
            std::cout << "[Selected file (カメラ移動最大)] " << chosenFile 
                      << " with Camera Movement = " << max_camera_movement 
                      << ", Score = " << min_score << std::endl;
        } else if (currentMode == 4) {
            // Mode 4: 動きが少ないカメラワーク
            // Camera Movement が最も小さい候補を採用
            double min_camera_movement = 100.0; // 適切な初期値を設定
            std::string best_file;
            double best_score = 0.0;
            for (int i = 0; i < top_n; i++) {
                std::string candidate_file = scores[i].first;
                double candidate_score = scores[i].second;
                double movement_distance = getPositionAverageForCandidateMsgpack(candidate_file, segmentLen, cameraPositionMsgpackDir);
                if (movement_distance < min_camera_movement) {
                    min_camera_movement = movement_distance;
                    best_file = candidate_file;
                    best_score = candidate_score;
                }
            }
            chosenFile = best_file;
            min_score = best_score;
            std::cout << "[Selected file (カメラ移動最小)] " << chosenFile 
                        << " with Camera Movement = " << min_camera_movement 
                        << ", Score = " << min_score << std::endl;

        } else if (currentMode == 5) {
            // Mode 5: 視点引き (mode==1) と 動き多め (mode==3) の両方を考慮
            std::unordered_map<std::string, int> rank_mode1;
            std::unordered_map<std::string, int> rank_mode3;
            
            // 視点引き (mode==1) の評価: Distance の平均が小さい順にソート
            std::vector<std::pair<std::string, double>> sorted_mode1(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode1.begin(), sorted_mode1.end(), [&](const auto &a, const auto &b) {
                return getDistanceAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir) <
                        getDistanceAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode1.size()); ++rank) {
                rank_mode1[sorted_mode1[rank - 1].first] = rank;
            }
            
            // 動き多め (mode==3) の評価: Camera Movement が大きい順にソート
            std::vector<std::pair<std::string, double>> sorted_mode3(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode3.begin(), sorted_mode3.end(), [&](const auto &a, const auto &b) {
                return getPositionAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir) >
                        getPositionAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode3.size()); ++rank) {
                rank_mode3[sorted_mode3[rank - 1].first] = rank;
            }
            
            // 合計ランクの計算
            std::unordered_map<std::string, int> rank_sum;
            for (const auto &p : rank_mode1) {
                if (rank_mode3.find(p.first) != rank_mode3.end())
                    rank_sum[p.first] = p.second + rank_mode3[p.first];
            }
            int min_rank = std::numeric_limits<int>::max();
            std::string best_file;
            for (const auto &p : rank_sum) {
                if (p.second < min_rank) {
                    min_rank = p.second;
                    best_file = p.first;
                }
            }
            chosenFile = best_file;
            min_score = static_cast<double>(min_rank);
            std::cout << "[Selected file (視点引き + 動き多め)] " << chosenFile 
                        << " with rank sum = " << min_rank << std::endl;

        } else if (currentMode == 6) {
            // Mode 6: 視点寄り (mode==2) と 動き多め (mode==3) の両方を考慮
            std::unordered_map<std::string, int> rank_mode2;
            std::unordered_map<std::string, int> rank_mode3;
            
            // 視点寄り (mode==2) の評価: 特定条件付きソート
            std::vector<std::pair<std::string, double>> sorted_mode2(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode2.begin(), sorted_mode2.end(), [&](const auto &a, const auto &b) {
                double pa = getPositionAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir);
                double pb = getPositionAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
                if ((pa > -5) != (pb > -5)) {
                    return (pa <= -5);  // 値が -5 以下のものを優先
                } else {
                    double da = getDistanceAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir);
                    double db = getDistanceAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
                    return da > db; 
                }
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode2.size()); ++rank) {
                rank_mode2[sorted_mode2[rank - 1].first] = rank;
            }
            
            // 動き多め (mode==3) の評価: Camera Movement が大きい順
            std::vector<std::pair<std::string, double>> sorted_mode3(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode3.begin(), sorted_mode3.end(), [&](const auto &a, const auto &b) {
                return getPositionAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir) >
                        getPositionAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode3.size()); ++rank) {
                rank_mode3[sorted_mode3[rank - 1].first] = rank;
            }
            
            std::unordered_map<std::string, int> rank_sum;
            for (const auto &p : rank_mode2) {
                if (rank_mode3.find(p.first) != rank_mode3.end())
                    rank_sum[p.first] = p.second + rank_mode3[p.first];
            }
            int min_rank = std::numeric_limits<int>::max();
            std::string best_file;
            for (const auto &p : rank_sum) {
                if (p.second < min_rank) {
                    min_rank = p.second;
                    best_file = p.first;
                }
            }
            chosenFile = best_file;
            min_score = static_cast<double>(min_rank);
            std::cout << "[Selected file (視点寄り + 動き多め)] " << chosenFile 
                        << " with rank sum = " << min_rank << std::endl;

        } else if (currentMode == 7) {
            // Mode 7: 視点引き (mode==1) と 動き少なめ (mode==4) の両方を考慮
            std::unordered_map<std::string, int> rank_mode1;
            std::unordered_map<std::string, int> rank_mode4;
            
            std::vector<std::pair<std::string, double>> sorted_mode1(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode1.begin(), sorted_mode1.end(), [&](const auto &a, const auto &b) {
                return getDistanceAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir) <
                        getDistanceAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode1.size()); ++rank) {
                rank_mode1[sorted_mode1[rank - 1].first] = rank;
            }
            
            std::vector<std::pair<std::string, double>> sorted_mode4(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode4.begin(), sorted_mode4.end(), [&](const auto &a, const auto &b) {
                return getPositionAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir) <
                        getPositionAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode4.size()); ++rank) {
                rank_mode4[sorted_mode4[rank - 1].first] = rank;
            }
            
            std::unordered_map<std::string, int> rank_sum;
            for (const auto &p : rank_mode1) {
                if (rank_mode4.find(p.first) != rank_mode4.end())
                    rank_sum[p.first] = p.second + rank_mode4[p.first];
            }
            int min_rank = std::numeric_limits<int>::max();
            std::string best_file;
            for (const auto &p : rank_sum) {
                if (p.second < min_rank) {
                    min_rank = p.second;
                    best_file = p.first;
                }
            }
            chosenFile = best_file;
            min_score = static_cast<double>(min_rank);
            std::cout << "[Selected file (視点引き + 動き少なめ)] " << chosenFile 
                        << " with rank sum = " << min_rank << std::endl;
        } else if (currentMode == 8) {
            // Mode 8: 視点寄り (mode==2) と 動き少なめ (mode==4) の両方を考慮
            std::unordered_map<std::string, int> rank_mode2;
            std::unordered_map<std::string, int> rank_mode4;
            
            std::vector<std::pair<std::string, double>> sorted_mode2(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode2.begin(), sorted_mode2.end(), [&](const auto &a, const auto &b) {
                double pa = getPositionAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir);
                double pb = getPositionAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
                if ((pa > -5) != (pb > -5)) {
                    return (pa <= -5);
                } else {
                    double da = getDistanceAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir);
                    double db = getDistanceAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
                    return da > db;
                }
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode2.size()); ++rank) {
                rank_mode2[sorted_mode2[rank - 1].first] = rank;
            }
            
            std::vector<std::pair<std::string, double>> sorted_mode4(scores.begin(), scores.begin() + top_n);
            std::sort(sorted_mode4.begin(), sorted_mode4.end(), [&](const auto &a, const auto &b) {
                return getPositionAverageForCandidateMsgpack(a.first, segmentLen, cameraPositionMsgpackDir) <
                        getPositionAverageForCandidateMsgpack(b.first, segmentLen, cameraPositionMsgpackDir);
            });
            for (int rank = 1; rank <= static_cast<int>(sorted_mode4.size()); ++rank) {
                rank_mode4[sorted_mode4[rank - 1].first] = rank;
            }
            
            std::unordered_map<std::string, int> rank_sum;
            for (const auto &p : rank_mode2) {
                if (rank_mode4.find(p.first) != rank_mode4.end())
                    rank_sum[p.first] = p.second + rank_mode4[p.first];
            }
            int min_rank = std::numeric_limits<int>::max();
            std::string best_file;
            for (const auto &p : rank_sum) {
                if (p.second < min_rank) {
                    min_rank = p.second;
                    best_file = p.first;
                }
            }
            chosenFile = best_file;
            min_score = static_cast<double>(min_rank);
            std::cout << "[Selected file (視点寄り + 動き少なめ)] " << chosenFile 
                        << " with rank sum = " << min_rank << std::endl;
        } else {
            // その他（ミックス視点）：スコア最小の候補をそのまま採用
            chosenFile = scores[0].first;
            min_score = scores[0].second;
            std::cout << "[Selected file (ミックス: Score最小)] " << chosenFile 
                        << " with score = " << min_score << std::endl;
        }
        cout << "選択ファイル: " << chosenFile << "\n";
        string chosenDbPath = databaseMsgpackDir + "/" + chosenFile;
        vector<FrameData> chosenDbFrames = loadJointPositions(chosenDbPath);
        if (chosenDbFrames.size() > (size_t)segmentLen)
            chosenDbFrames.resize(segmentLen);
        // 各フレームごとの平行移動（root の差分）を計算
        for (int i = 0; i < segmentLen; i++) {
            if (i >= rawSegment.size() || i >= chosenDbFrames.size())
                break;
            array<double, 3> rootInput = rawSegment[i].positions[0];
            array<double, 3> rootChosen = chosenDbFrames[i].positions[0];
            array<double, 3> trans = {rootInput[0] - rootChosen[0],
                                      rootInput[1] - rootChosen[1],
                                      rootInput[2] - rootChosen[2]};
            result.translations.push_back(trans);
        }
        result.closestFiles.push_back(chosenFile);
        result.lengths.push_back(segmentLen);
    }
    // 全フレームの translations にガウスフィルタを適用
    double sigma = 10.0;
    result.translations = applyGaussianFilter(result.translations, sigma);
    return result;
}

// カメラデータの取得
struct CameraRetrievalResult {
    vector<array<double, 3>> position;
    vector<array<double, 3>> rotation;
    vector<double> viewangle;
};

CameraRetrievalResult cameraDataRetrievalMsgpack(const string &cameraPositionMsgpackDir,
                                                   const string &cameraRotationMsgpackDir,
                                                   const vector<string> &closestFiles,
                                                   const vector<int> &lengths,
                                                   const string &inputNumber,
                                                   const vector<array<double, 3>> &translations) {
    CameraRetrievalResult camRes;
    for (size_t segIndex = 0; segIndex < closestFiles.size(); segIndex++) {
        int lengthFrames = lengths[segIndex];
        string fileName = closestFiles[segIndex];
        if (fileName.empty()) {
            for (int i = 0; i < lengthFrames; i++) {
                camRes.position.push_back({0, 0, 0});
                camRes.rotation.push_back({0, 0, 0});
                camRes.viewangle.push_back(60.0);
            }
            continue;
        }
        string fileNumberStr;
        int segStart = 0, segEnd = 0;
        parseSegmentFilename(fileName, fileNumberStr, segStart, segEnd);
        string posFile = cameraPositionMsgpackDir + "/c" + fileNumberStr + ".msgpack";
        string rotFile = cameraRotationMsgpackDir + "/c" + fileNumberStr + ".msgpack";
        msgpack::object_handle posOh = readMsgpack(posFile);
        msgpack::object posObj = posOh.get();
        msgpack::object_handle rotOh = readMsgpack(rotFile);
        msgpack::object rotObj = rotOh.get();
        const msgpack::object* eyeArray = getMember(posObj, "camera_eye");
        const msgpack::object* fovArray = getMember(posObj, "Fov");
        const msgpack::object* rotArray = getMember(rotObj, "Rotation");
        if (!eyeArray || !fovArray || !rotArray) {
            cerr << "カメラデータが不足しています: " << posFile << " または " << rotFile << "\n";
            continue;
        }
        int totalSize_eye = eyeArray->via.array.size;
        int totalSize_rot = rotArray->via.array.size;
        int totalSize_fov = fovArray->via.array.size;
        int startIndex = segStart;
        int endIndex = segStart + lengthFrames;
        if (endIndex > totalSize_eye)
            endIndex = totalSize_eye;
        if (endIndex > totalSize_rot)
            endIndex = min(endIndex, totalSize_rot);
        if (endIndex > totalSize_fov)
            endIndex = min(endIndex, totalSize_fov);
        for (int i = startIndex; i < endIndex; i++) {
            msgpack::object e = eyeArray->via.array.ptr[i];
            array<double, 3> epos = { e.via.array.ptr[0].as<double>(),
                                      e.via.array.ptr[1].as<double>(),
                                      e.via.array.ptr[2].as<double>() };
            msgpack::object r = rotArray->via.array.ptr[i];
            array<double, 3> rrot = { r.via.array.ptr[0].as<double>(),
                                      r.via.array.ptr[1].as<double>(),
                                      r.via.array.ptr[2].as<double>() };
            double fv = fovArray->via.array.ptr[i].as<double>();
            camRes.position.push_back(epos);
            camRes.rotation.push_back(rrot);
            camRes.viewangle.push_back(fv);
        }
    }
    int n = min((int)camRes.position.size(), (int)translations.size());
    for (int i = 0; i < n; i++) {
        camRes.position[i][0] += translations[i][0];
        camRes.position[i][1] += translations[i][1];
        camRes.position[i][2] += translations[i][2];
    }
    return camRes;
}

// JSON 出力（RapidJSON 使用）
// MessagePack から読み込んだカメラデータを JSON 形式で出力
void outputCameraJson(const vector<array<double, 3>> &position,
                      const vector<array<double, 3>> &rotation,
                      const vector<double> &viewangle,
                      const string &outputDir,
                      const string &inputNumber) {
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    int n = position.size();
    doc.AddMember("CameraKeyFrameNumber", n, allocator);
    rapidjson::Value records(rapidjson::kArrayType);
    for (int i = 0; i < n; i++) {
        rapidjson::Value frameData(rapidjson::kObjectType);
        rapidjson::Value curve(rapidjson::kArrayType);
        curve.PushBack(20, allocator)
             .PushBack(107, allocator)
             .PushBack(20, allocator)
             .PushBack(107, allocator);
        frameData.AddMember("Curve", curve, allocator);
        frameData.AddMember("Distance", 0.0, allocator);
        frameData.AddMember("FrameTime", i, allocator);
        frameData.AddMember("Orthographic", 0, allocator);
        rapidjson::Value pos(rapidjson::kObjectType);
        pos.AddMember("x", position[i][0], allocator);
        pos.AddMember("y", position[i][1], allocator);
        pos.AddMember("z", -position[i][2], allocator); // z 座標は -1 倍
        frameData.AddMember("Position", pos, allocator);
        rapidjson::Value rot(rapidjson::kObjectType);
        rot.AddMember("z", rotation[i][2], allocator);
        rot.AddMember("y", rotation[i][1], allocator);
        rot.AddMember("x", rotation[i][0], allocator);
        frameData.AddMember("Rotation", rot, allocator);
        frameData.AddMember("ViewAngle", viewangle[i], allocator);
        records.PushBack(frameData, allocator);
    }
    doc.AddMember("CameraKeyFrameRecord", records, allocator);
    string outPath = outputDir + "/MM" + inputNumber + ".json";
    ofstream ofs(outPath);
    if (!ofs.is_open()) {
        cerr << "出力ファイルを開けません: " << outPath << endl;
        return;
    }
    rapidjson::OStreamWrapper osw(ofs);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);
    doc.Accept(writer);
    cout << "出力ファイル: " << outPath << endl;
}

// main 関数（対話的入力ロジックを追加）
int main(int argc, char* argv[]){
    string camera_mode = "normal";

    string mode;
    cout << "initial or modify\n> ";
    cin >> mode;

    string input_number_str;
    cout << "入力番号をいくつにしますか？\n> ";
    cin >> input_number_str;
    int input_number = stoi(input_number_str);

    int camera_view, cut_number, m;
    string view, view_place, movement, movement_place, continue_view, continue_movement;
    vector<int> partial_views;     // 部分的な視点変更を保持するリスト
    vector<int> partial_movements; // 部分的な動き変更を保持するリスト

    // モードが "initial" の場合
    if (mode == "initial") {
        camera_view = 3;
        cut_number = 4;
        m = 1;
    }
    // モードが "modify" の場合
    else if (mode == "modify") {
        m = 2;
        // カメラ視点位置の入力
        cout << "カメラの視点位置はどうしますか？(引き視点 or 寄り視点 or このまま)\n> ";
        cin >> view;
        if (view != "このまま") {
            cout << "視点位置の適用範囲はどうしますか？(全体 or サビ or サビ以外 or 部分的)\n> ";
            cin >> view_place;
            if (view_place == "部分的") {
                while (true) {
                    string partial_frame_str;
                    cout << "部分的に変更したいカメラワークのフレーム数を入力してください。\n> ";
                    cin >> partial_frame_str;
                    int partial_frame = stoi(partial_frame_str);
                    partial_views.push_back(partial_frame);
                    string another;
                    cout << "さらに部分的な変更を追加しますか？(はい or いいえ)\n> ";
                    cin >> another;
                    // 入力を小文字に変換して比較
                    transform(another.begin(), another.end(), another.begin(), ::tolower);
                    if (another != "はい")
                        break;
                }
            }
            else if (view_place != "全体") {
                cout << "変更しなかった範囲にもう片方の視点位置を適用しますか？(はい or いいえ)\n> ";
                cin >> continue_view;
            }
        }

        // カメラの全体的な動きの入力
        cout << "カメラの全体的な動きはどうしますか？(動き多め or 動き少なめ or このまま)\n> ";
        cin >> movement;
        if (movement != "このまま") {
            cout << "動きの適用範囲はどうしますか？(全体 or サビ or サビ以外 or 部分的)\n> ";
            cin >> movement_place;
            if (movement_place == "部分的") {
                while (true) {
                    string partial_movement_frame_str;
                    cout << "部分的に変更したい動きのフレーム数を入力してください。\n> ";
                    cin >> partial_movement_frame_str;
                    int partial_movement_frame = stoi(partial_movement_frame_str);
                    partial_movements.push_back(partial_movement_frame);
                    string another_m;
                    cout << "さらに部分的な動きの変更を追加しますか？(はい or いいえ)\n> ";
                    cin >> another_m;
                    transform(another_m.begin(), another_m.end(), another_m.begin(), ::tolower);
                    if (another_m != "はい")
                        break;
                }
            }
            else if (movement_place != "全体") {
                cout << "変更しなかった範囲にもう片方の動き方を適用しますか？(はい or いいえ)\n> ";
                cin >> continue_movement;
            }
        }

        // カットの頻度の入力
        string cut;
        cout << "カットの頻度はどうしますか？(低くする or このまま)\n> ";
        cin >> cut;
        string cut_place;
        if (cut == "低くする") {
            cout << "カットの頻度の適用範囲はどうしますか？(全体 or サビ or サビ以外)\n> ";
            cin >> cut_place;
        }

        // ビューによるカメラ視点位置の設定
        if (view == "引き視点")
            camera_view = 1;
        else if (view == "寄り視点")
            camera_view = 2;
        else
            camera_view = 3;
        
        // カット頻度の設定
        if (cut == "低くする") {
            if (cut_place == "全体")
                cut_number = 1;
            else if (cut_place == "サビ")
                cut_number = 3;
            else
                cut_number = 2;
        }
        else {
            cut_number = 4;
        }
    }

    // 入力 BVH ファイルパスを設定（入力番号に基づく）
    string inputBvh = "DataBase/Bvh/m" + input_number_str + ".bvh";
    string frameIntervalsMsgpack = "DataBase/Frame_Intervals/frame_intervals_" + std::to_string(cut_number) + ".msgpack";
    // 全身のデータ(23ジョイント)
    string standMsgpackDir = "Database/Stand_Raw";
    string standDatabaseMsgpackDir = "Database/Stand_Split";
    string rawMsgpackDir = "Database/Raw";
    string databaseMsgpackDir = "Database/Split";
    // ヒップ方向データ
    string hipDirectionMsgpackDir = "Database/Hip_Direction";
    string hipDirectionDatabaseMsgpackDir = "Database/Hip_Direction_Split";
    // 音楽データ
    string musicMsgpackDir = "Database/Music_Features";
    string musicDatabaseMsgpackDir = "Database/Music_Features_Split";
    // カメラデータ
    string cameraPositionMsgpackDir = "DataBaase/CameraCentric";
    string cameraRotationMsgpackDir = "DataBase/CameraInterpolated";
    // BPM データ
    string bpmMsgpack = "Database/BPM/average_bpm.msgpack";
    // 出力ディレクトリ
    string outputDir = "Output/json/";

    // コマンドライン引数で上書き可能な設定
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--input_bvh" && i + 1 < argc) {
            inputBvh = argv[++i];
        } else if (arg == "--output_dir" && i + 1 < argc) {
            outputDir = argv[++i];
        }
    }

    // frame_intervals の読み込み（MessagePack 版）
    msgpack::object_handle intervalsOh = readMsgpack(frameIntervalsMsgpack);
    msgpack::object intervalsObj = intervalsOh.get();
    // 対話入力で得た input_number_str を利用
    string inputNumStr = input_number_str;
    vector<int> frameIntervals;
    vector<int> sabi_indices;
    const msgpack::object* fiMember = getMember(intervalsObj, inputNumStr);
    if (fiMember && fiMember->type == msgpack::type::MAP) {
        const msgpack::object* arr = getMember(*fiMember, "frame_intervals");
        if (arr && arr->type == msgpack::type::ARRAY) {
            for (size_t i = 0; i < arr->via.array.size; i++) {
                frameIntervals.push_back(arr->via.array.ptr[i].as<int>());
            }
        } else {
            cerr << "Error: frame_intervals_msgpack に " << inputNumStr << " が含まれていません\n";
            return 1;
        }   
        // sabi の読み込み（オプショナル）
        const msgpack::object* sabiArr = getMember(*fiMember, "sabi");
        if (sabiArr && sabiArr->type == msgpack::type::ARRAY) {
            for (size_t i = 0; i < sabiArr->via.array.size; i++) {
                sabi_indices.push_back(sabiArr->via.array.ptr[i].as<int>());
            }
        }
    } else {
            cerr << "Error: frame_intervals_msgpack に " << inputNumStr << " が含まれていません\n";
            return 1;
    }

    // modes ベクトルの設定
    // modes ベクトルの設定（すべて 10 で初期化）
    vector<int> modes(frameIntervals.size(), 10);
    int step = 1;

    vector<int> view_indices, movement_indices;
    if (mode == "modify") {
        view_indices = convertFramesToIndices(partial_views, frameIntervals);
        movement_indices = convertFramesToIndices(partial_movements, frameIntervals);
        cout << "view_indices: ";
        for (int idx : view_indices) {
            cout << idx << " ";
        }
        cout << endl;
        cout << "movement_indices: ";
        for (int idx : movement_indices) {
            cout << idx << " ";
        }
        cout << endl;
    }
    // ----- ビューに関する分岐 -----
    if (mode == "modify") {
        if (view == "引き視点") {
            if (view_place == "サビ") {
                // sabi_indices に含まれるインデックスは 1 に設定
                for (int idx : sabi_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        modes[idx] = 1;
                    }
                }
                // continue_view が "はい" の場合、sabi_indices に含まれない部分は 2 に設定
                if (continue_view == "はい") {
                    for (int i = 0; i < modes.size(); i++) {
                        if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                            modes[i] = 2;
                        }    
                    }
                }
            }
            else if (view_place == "サビ以外") {
                // sabi_indices に含まれない部分を 1 に設定
                for (int i = 0; i < modes.size(); i++) {
                    if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                        modes[i] = 1;
                    }
                }
                // continue_view が "はい" の場合、sabi_indices に含まれる部分は 2 に設定
                if (continue_view == "はい") {
                    for (int idx : sabi_indices) {
                        if (idx >= 0 && idx < modes.size()) {
                            modes[idx] = 2;                      
                        }
                    }
                }
            }
            else if (view_place == "部分的") {
                // view_indices に含まれるインデックスを 1 に設定
                for (int idx : view_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        modes[idx] = 1;
                    }
                }
            }
            else { // view_place が "全体" などの場合
                for (int i = 0; i < modes.size(); i++) {
                    modes[i] = 1;
                }
            }
        }
        else if (view == "寄り視点") {
            if (view_place == "サビ") {
                for (int idx : sabi_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        modes[idx] = 2;
                    }
                }
                if (continue_view == "はい") {
                    for (int i = 0; i < modes.size(); i++) {
                        if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                            modes[i] = 1;
                        }
                    }
                }
            }
            else if (view_place == "サビ以外") {
                for (int i = 0; i < modes.size(); i++) {
                    if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                        modes[i] = 2;
                    }
                }
                if (continue_view == "はい") {
                    for (int idx : sabi_indices) {
                        if (idx >= 0 && idx < modes.size()) {
                            modes[idx] = 1;
                        }
                    }
                }
            }
            else if (view_place == "部分的") {
                for (int idx : view_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        modes[idx] = 2;
                    }
                }
            }
            else { // 全体など
                for (int i = 0; i < modes.size(); i++) {
                    modes[i] = 2;
                }
            }
        }
    // ----- 動きに関する分岐 -----
        if (movement == "動き多め") {
            if (movement_place == "サビ") {
                for (int idx : sabi_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        if (modes[idx] == 1) {
                            modes[idx] = 5;
                        } else if (modes[idx] == 2) {
                            modes[idx] = 6;
                        } else {
                            modes[idx] = 3;
                        }
                    }
                }
                if (continue_movement == "はい") {
                    for (int i = 0; i < modes.size(); i++) {
                        if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                            if (modes[i] == 1) {
                                modes[i] = 7;
                            } else if (modes[i] == 2) {
                                modes[i] = 8;
                            } else {
                                modes[i] = 4;
                            }
                        }
                    }
                }
            }
            else if (movement_place == "サビ以外") {
                for (int i = 0; i < modes.size(); i++) {
                    if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                        if (modes[i] == 1) {
                            modes[i] = 5;
                        } else if (modes[i] == 2) {
                            modes[i] = 6;
                        } else {
                            modes[i] = 3;
                        }
                    }
                }
                if (continue_movement == "はい") {
                    for (int idx : sabi_indices) {
                        if (idx >= 0 && idx < modes.size()) {
                            if (modes[idx] == 1) {
                                modes[idx] = 7;
                            } else if (modes[idx] == 2) {
                                modes[idx] = 8;
                            } else {
                                modes[idx] = 4;
                            }   
                        }
                    }
                }
            }
            else if (movement_place == "部分的") {
                for (int idx : movement_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        if (modes[idx] == 1) {
                            modes[idx] = 5;
                        } else if (modes[idx] == 2) {
                            modes[idx] = 6;
                        } else {
                            modes[idx] = 3;
                        }
                    }
                }
            }
            else {
                for (int i = 0; i < modes.size(); i++) {
                    if (modes[i] == 1) {
                        modes[i] = 5;
                    } else if (modes[i] == 2) {
                        modes[i] = 6;
                    } else {
                        modes[i] = 3;
                    }
                }
            }
        }
        else if (movement == "動き少なめ") {
            if (movement_place == "サビ") {
                for (int idx : sabi_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        if (modes[idx] == 1) {
                            modes[idx] = 7;
                        } else if (modes[idx] == 2) {
                            modes[idx] = 8;
                        } else {
                            modes[idx] = 4;
                        }
                    }
                }
                if (continue_movement == "はい") {
                    for (int i = 0; i < modes.size(); i++) {
                        if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                            if (modes[i] == 1) {
                                modes[i] = 5;
                            } else if (modes[i] == 2) {
                                modes[i] = 6;
                            } else {
                                modes[i] = 3;
                            }   
                        }
                    }
                }
            }
            else if (movement_place == "サビ以外") {
                for (int i = 0; i < modes.size(); i++) {
                    if (find(sabi_indices.begin(), sabi_indices.end(), i) == sabi_indices.end()) {
                        if (modes[i] == 1) {
                            modes[i] = 7;
                        } else if (modes[i] == 2) {
                            modes[i] = 8;
                        } else {
                            modes[i] = 4;
                        }
                    }
                }
                if (continue_movement == "はい") {
                    for (int idx : sabi_indices) {
                        if (idx >= 0 && idx < modes.size()) {
                            if (modes[idx] == 1) {
                                modes[idx] = 5;
                            } else if (modes[idx] == 2) {
                                modes[idx] = 6;
                            } else {
                                modes[idx] = 3;
                            }
                        }   
                    }
                }
            }
            else if (movement_place == "部分的") {
                for (int idx : movement_indices) {
                    if (idx >= 0 && idx < modes.size()) {
                        if (modes[idx] == 1) {
                            modes[idx] = 7;
                        } else if (modes[idx] == 2) {
                            modes[idx] = 8;
                        } else {
                            modes[idx] = 4;
                        }
                    }
                }
            }
            else {
                for (int i = 0; i < modes.size(); i++) {
                    if (modes[i] == 1) {
                        modes[i] = 7;
                    } else if (modes[i] == 2) {
                        modes[i] = 6;
                    } else {
                        modes[i] = 4;
                    }
                }
            }
        }
    }
    
    // vector<int> を出力する例
    for (const auto& val : frameIntervals) {
        cout << val << " ";
    }
    cout << endl;

    for (const auto& val : sabi_indices) {
        cout << val << " ";
    }
    cout << endl;

    for (const auto& val : modes) {
        cout << val << " ";
    }
    cout << endl;


    // 類似ファイル検索
    CalDistance2Result cd2Res = calDistance2Msgpack(bpmMsgpack, musicMsgpackDir, musicDatabaseMsgpackDir, inputBvh, standMsgpackDir, standDatabaseMsgpackDir,
                                                     hipDirectionMsgpackDir, hipDirectionDatabaseMsgpackDir,
                                                     rawMsgpackDir, databaseMsgpackDir, frameIntervals, step,
                                                     modes, cameraPositionMsgpackDir, m);
    
    // カメラデータ組み立て
    CameraRetrievalResult camRes = cameraDataRetrievalMsgpack(cameraPositionMsgpackDir, cameraRotationMsgpackDir,
                                                               cd2Res.closestFiles, cd2Res.lengths,
                                                               cd2Res.inputNumber, cd2Res.translations);
    
    // JSON 出力
    outputCameraJson(camRes.position, camRes.rotation, camRes.viewangle, outputDir, cd2Res.inputNumber);
    
    return 0;
}
