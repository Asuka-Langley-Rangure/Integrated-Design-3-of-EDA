#include "LefParser.h"
#include <iostream>
#include <fstream>

int main() {
    try {
        LefParser parser;
        LefFile lef = parser.parseFile("cn.lef");

        std::ofstream out("summary.txt");  // 输出到 summary.txt
        if (!out) {
            throw std::runtime_error("无法创建 summary.txt 文件");
        }

        // 将原来的打印函数输出到文件
        std::streambuf* coutBuf = std::cout.rdbuf(); // 保存原有缓冲
        std::cout.rdbuf(out.rdbuf());                // 重定向到文件

        lef.printSummary();  // 输出到 summary.txt

        std::cout.rdbuf(coutBuf); // 恢复控制台输出
        std::cout << "解析结果已写入 summary.txt\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }
}
