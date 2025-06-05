#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <filesystem>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <msgpack.hpp>  // msgpack-c のヘッダ

namespace fs = std::filesystem;
using namespace std;
using namespace rapidjson;

// RapidJSON の Value を再帰的に MessagePack に変換する関数
void packJsonValue(const Value &val, msgpack::packer<msgpack::sbuffer>& pk) {
    if (val.IsNull()) {
        pk.pack_nil();
    } else if (val.IsBool()) {
        pk.pack(val.GetBool());
    } else if (val.IsInt()) {
        pk.pack(val.GetInt());
    } else if (val.IsUint()) {
        pk.pack(val.GetUint());
    } else if (val.IsInt64()) {
        pk.pack(val.GetInt64());
    } else if (val.IsUint64()) {
        pk.pack(val.GetUint64());
    } else if (val.IsDouble()) {
        pk.pack(val.GetDouble());
    } else if (val.IsString()) {
        pk.pack(std::string(val.GetString(), val.GetStringLength()));
    } else if (val.IsArray()) {
        pk.pack_array(val.Size());
        for (auto& v : val.GetArray()) {
            packJsonValue(v, pk);
        }
    } else if (val.IsObject()) {
        pk.pack_map(val.MemberCount());
        for (auto itr = val.MemberBegin(); itr != val.MemberEnd(); ++itr) {
            // キーは文字列として pack
            pk.pack(std::string(itr->name.GetString(), itr->name.GetStringLength()));
            // 値は再帰的に処理
            packJsonValue(itr->value, pk);
        }
    }
}

// 単一の JSON ファイルを MessagePack に変換する関数
bool convertJsonToMsgpack(const fs::path &jsonPath, const fs::path &outputDir) {
    // JSON ファイルを C スタイルでオープン
    FILE* fp = fopen(jsonPath.string().c_str(), "rb");
    if (!fp) {
        cerr << "Error: cannot open input file: " << jsonPath << "\n";
        return false;
    }
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document doc;
    doc.ParseStream(is);
    fclose(fp);
    
    if (doc.HasParseError()) {
        cerr << "JSON parse error in file: " << jsonPath << "\n";
        return false;
    }
    
    // MessagePack 用のバッファとパッカーを用意
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    packJsonValue(doc, pk);
    
    // 出力ファイル名を決定（拡張子を .msgpack に変更）
    fs::path outputFile = outputDir / jsonPath.filename();
    outputFile.replace_extension(".msgpack");
    
    // 出力ファイルへ書き込み
    ofstream ofs(outputFile, ios::binary);
    if (!ofs) {
        cerr << "Error: cannot open output file: " << outputFile << "\n";
        return false;
    }
    ofs.write(sbuf.data(), sbuf.size());
    ofs.close();
    
    cout << "Converted " << jsonPath << " -> " << outputFile << "\n";
    return true;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " input_path output_directory\n";
        return 1;
    }
    
    fs::path inputPath = argv[1];
    fs::path outputDir = argv[2];
    
    // 入力パスの存在チェック
    if (!fs::exists(inputPath)) {
        cerr << "Input path does not exist: " << inputPath << "\n";
        return 1;
    }
    
    // 出力ディレクトリが存在しなければ作成
    if (!fs::exists(outputDir)) {
        fs::create_directories(outputDir);
    }
    
    // 入力がファイルの場合
    if (fs::is_regular_file(inputPath)) {
        if (inputPath.extension() == ".json") {
            convertJsonToMsgpack(inputPath, outputDir);
        } else {
            cerr << "Input file is not a .json file: " << inputPath << "\n";
            return 1;
        }
    }
    // 入力がディレクトリの場合、その中のすべての JSON ファイルを処理
    else if (fs::is_directory(inputPath)) {
        for (const auto &entry : fs::directory_iterator(inputPath)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                convertJsonToMsgpack(entry.path(), outputDir);
            }
        }
    }
    else {
        cerr << "Input path is neither a file nor a directory: " << inputPath << "\n";
        return 1;
    }
    
    return 0;
}
