#include "parse_bookshelf.h"
#include "MyPlace/myplace.h"
#include "PlaceData/placedata.h"

#include <iostream>
#include <iomanip>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char** argv) {
    // 获取当前执行目录
    fs::path exe_dir = fs::current_path();     // 当前执行目录
    fs::path tmp_path;
    if (argc > 1) {
        // 如果命令行传入路径参数，例如：./adaptec1/adaptec1.aux
        tmp_path = fs::path(argv[1]);
    } else {
        // 默认查找当前目录下的 adaptec1 文件夹
        tmp_path = exe_dir / "adaptec1" / "adaptec1.aux";
    }
    // 创建 PlaceData 对象
    PlaceData db;
    if (!ParseBookshelfDataset(tmp_path, db)) {
        cerr << "❌ ERROR: Failed to parse bookshelf dataset." << endl;
        return 1;
    }
    PrintDatabaseSummary(&db);
    return 0;
}
