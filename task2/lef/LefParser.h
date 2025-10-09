#pragma once
#include <string>
#include "LefFile.h"

class LefParser {
public:
    LefParser() = default;

    // 解析文件并返回一个 LefFile 对象
    LefFile parseFile(const std::string& filename);

private:
    // 工具函数
    static std::vector<std::string> tokenize(const std::string& line);
    static double toDouble(const std::string& s, double defaultValue = 0.0);
    static int toInt(const std::string& s, int defaultValue = 0);

    // 解析函数
    void parseVersion(LefFile& lef, const std::vector<std::string>& tokens);
    void parseBusBitChars(LefFile& lef, const std::vector<std::string>& tokens);
    void parseDividerChar(LefFile& lef, const std::vector<std::string>& tokens);
    void parseUnits(LefFile& lef, const std::vector<std::string>& tokens);
    void parseManufacturingGrid(LefFile& lef, const std::vector<std::string>& tokens);
    void parseSite(LefFile& lef, const std::vector<std::string>& tokens);
    void parseLayer(LefFile& lef, const std::vector<std::string>& tokens);
    void parseMacro(LefFile& lef, const std::vector<std::string>& tokens);
    void parsePin(LefFile& lef, const std::vector<std::string>& tokens);
    void parseObs(LefFile& lef, const std::vector<std::string>& tokens);
};
