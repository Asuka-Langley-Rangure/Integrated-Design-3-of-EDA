#include "myplace.h"
#include "placedata.h"
#include "plot.h"
#include "parse_bookshelf.h"
#include "NesterovOpt.h"

#include <iostream>
#include <iomanip>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char *argv[])
{
    PlaceData *db = new PlaceData();

    // 获取当前执行目录
    fs::path exe_dir = fs::current_path();     // 当前执行目录
    fs::path tmp_path;
    if (argc > 1) {
        // 如果命令行传入路径参数，例如：./adaptec4/adaptec4.aux
        tmp_path = fs::path(argv[1]);
    } else {
        // 默认查找当前目录下的 adaptec1 文件夹
        tmp_path = exe_dir / "adaptec1" / "adaptec1.aux";
    }

    // 创建 PlaceData 对象
    if (!ParseBookshelfDataset(tmp_path, db)) {
        cerr << "❌ ERROR: Failed to parse bookshelf dataset." << endl;
        return 1;
    }

    MyPlacer *myplacer = new MyPlacer(db);

    std::cout << "-----------任务2------------" << endl;

    //PrintDatabaseSummary(db);

    std::cout << "-----------任务3------------" << endl;

    //printDesignSummary(db, 512, 512, -1.0);

    std::cout << "-----------任务4------------" << endl;

    myplacer->initialPlacement();

    PLOTTING::plotPlacement("gp_result", db);

    //PrintPlaceData(db);

    std::cout << "-----------任务5------------" << endl;

    NesterovOpt* nesterov = new NesterovOpt(myplacer);
    nesterov->NAG_Process();
    PLOTTING::plotPlacement("gp_result_2", nesterov->placer->db);
    
    std::cout << "✅ Parsing and printing completed successfully." << std::endl;
    return 0;
}

