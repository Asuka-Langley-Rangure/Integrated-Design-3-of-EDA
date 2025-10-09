#include "LefParser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <regex>

// =================== 工具函数 ===================
bool isNumber(const std::string &str)
{
    // 匹配可选符号位 + 数字序列 + 可选小数点和数字序列
    static const std::regex numberRegex(R"(^[-+]?[0-9]*\.?[0-9]+$)");
    return std::regex_match(str, numberRegex);
}

std::vector<std::string> LefParser::tokenize(const std::string &line)
{
    std::vector<std::string> tokens;
    std::istringstream iss(line);
    std::string token;
    while (iss >> token)
    {
        // 去掉末尾分号
        if (!token.empty() && token.back() == ';')
        {
            token.pop_back();
        }
        // 去掉前后引号
        if (!token.empty() && token.front() == '"' && token.back() == '"' && token.size() >= 2)
        {
            token = token.substr(1, token.size() - 2);
        }
        if (!token.empty())
        {
            tokens.push_back(token);
        }
    }
    return tokens;
}

double LefParser::toDouble(const std::string &s, double defaultValue)
{
    try
    {
        return std::stod(s);
    }
    catch (...)
    {
        return defaultValue;
    }
}

int LefParser::toInt(const std::string &s, int defaultValue)
{
    try
    {
        return std::stoi(s);
    }
    catch (...)
    {
        return defaultValue;
    }
}

// =================== 主解析入口 ===================
LefFile LefParser::parseFile(const std::string &filename)
{
    LefFile lef;
    std::ifstream in(filename);
    if (!in)
    {
        throw std::runtime_error("Cannot open LEF file: " + filename);
    }

    std::string line;
    std::string tempLine;
    while (std::getline(in, line))
    {
        auto tokens = tokenize(line);
        if (tokens.empty())
            continue;

        if (tokens[0] == "VERSION")
        {
            parseVersion(lef, tokens);
        }
        else if (tokens[0] == "BUSBITCHARS")
        {
            parseBusBitChars(lef, tokens);
        }
        else if (tokens[0] == "DIVIDERCHAR")
        {
            parseDividerChar(lef, tokens);
        }
        else if (tokens[0] == "MANUFACTURINGGRID")
        {
            parseManufacturingGrid(lef, tokens);
        }
        else if (tokens[0] == "UNITS")
        {
            while (std::getline(in, tempLine))
            {
                auto tempTokens = tokenize(tempLine);
                tokens.insert(tokens.end(), tempTokens.begin(), tempTokens.end());
                if (tempTokens.size()== 0) continue;
                if (tempTokens[0] == "END" && tempTokens[1] == "UNITS")
                    break;
            }
            parseUnits(lef, tokens);
        }
        else if (tokens[0] == "SITE")
        {
            while (std::getline(in, tempLine))
            {
                auto tempTokens = tokenize(tempLine);
                tokens.insert(tokens.end(), tempTokens.begin(), tempTokens.end());
                if (tempTokens.size()== 0) continue;
                if (tempTokens[0] == "END" && tempTokens[1] == tokens[1])
                    break;
            }
            parseSite(lef, tokens);
        }
        else if (tokens[0] == "LAYER")
        {
            while (std::getline(in, tempLine))
            {
                auto tempTokens = tokenize(tempLine);
                tokens.insert(tokens.end(), tempTokens.begin(), tempTokens.end());
                if (tempTokens.size()== 0) continue;
                if (tempTokens[0] == "END" && tempTokens[1] == tokens[1])
                    break;
            }
            parseLayer(lef, tokens);
        }
        else if (tokens[0] == "MACRO")
        {
            while (std::getline(in, tempLine))
            {
                auto tempTokens = tokenize(tempLine);
                tokens.insert(tokens.end(), tempTokens.begin(), tempTokens.end());
                if (tempTokens.size()== 0) continue;
                if (tempTokens[0] == "END" && tempTokens[1] == tokens[1])
                    break;
            }
            parseMacro(lef, tokens);
        }
        // TODO: 后续扩展 MACRO, PIN, OBS ...
    }

    return lef;
}

// =================== 解析函数实现 ===================
void LefParser::parseVersion(LefFile &lef, const std::vector<std::string> &tokens)
{
    if (tokens.size() >= 2)
    {
        lef.version = tokens[1];
    }
}

void LefParser::parseBusBitChars(LefFile &lef, const std::vector<std::string> &tokens)
{
    if (tokens.size() >= 2)
    {
        lef.busBitChars = tokens[1];
    }
}

void LefParser::parseDividerChar(LefFile &lef, const std::vector<std::string> &tokens)
{
    if (tokens.size() >= 2)
    {
        lef.dividerChar = tokens[1];
    }
}

void LefParser::parseUnits(LefFile &lef, const std::vector<std::string> &tokens)
{
    // UNITS DATABASE MICRONS 2000 ;
    if (tokens.size() >= 4 && tokens[1] == "DATABASE" && tokens[2] == "MICRONS")
    {
        lef.units.databaseMicrons = toInt(tokens[3]);
    }
}

void LefParser::parseManufacturingGrid(LefFile &lef, const std::vector<std::string> &tokens)
{
    if (tokens.size() >= 2)
    {
        lef.manufacturingGrid = toDouble(tokens[1]);
    }
}

void LefParser::parseSite(LefFile &lef, const std::vector<std::string> &tokens)
{
    LefSite site;
    site.name = tokens[1];

    // 从tokens的第二个元素开始遍历（跳过第一个命令和site name）
    for (size_t i = 2; i < tokens.size(); ++i)
    {
        const std::string &token = tokens[i];

        if (token == "SIZE" && i + 3 < tokens.size())
        {
            site.size = {toDouble(tokens[i + 1]), toDouble(tokens[i + 3])};
            i += 3; // 跳过已处理的SIZE参数
        }
        else if (token == "CLASS" && i + 1 < tokens.size())
        {
            site.siteClass = tokens[i + 1];
            i += 1; // 跳过CLASS参数
        }
        else if (token == "SYMMETRY" && i + 1 < tokens.size())
        {
            // 收集所有SYMMETRY参数直到下一个关键字或结束
            for (size_t j = i + 1; j < tokens.size(); ++j)
            {
                if (tokens[j] == "SIZE" || tokens[j] == "CLASS" || tokens[j] == "END")
                {
                    break;
                }
                site.symmetry.push_back(tokens[j]);
            }
            i += site.symmetry.size(); // 跳过所有symmetry参数
        }
        else if (token == "END" && tokens[i + 1] == site.name)
        {
            break;
        }
        // 其他未识别的token会被忽略
    }
    lef.addSite(site);
}

void LefParser::parseLayer(LefFile &lef, const std::vector<std::string> &tokens)
{
    if (tokens.size() < 2)
    {
        std::cerr << "Error: Invalid LAYER statement - missing name" << std::endl;
        return;
    }

    LefLayer layer;
    layer.name = tokens[1];

    // 从tokens的第二个元素开始遍历（跳过LAYER命令和layer name）
    for (size_t i = 2; i < tokens.size(); ++i)
    {
        const std::string &token = tokens[i];

        if (token == "TYPE" && i + 1 < tokens.size())
        {
            layer.type = tokens[i + 1];
            i += 1; // 跳过TYPE参数
        }
        else if (token == "DIRECTION" && i + 1 < tokens.size())
        {
            layer.direction = tokens[i + 1];
            i += 1; // 跳过DIRECTION参数
        }
        else if (token == "PITCH" /*&& i + 2 < tokens.size()*/)
        {
            layer.routingFactor.push_back("PITCH");
            int j = i + 1;
            while (j < tokens.size() && isNumber(tokens[j]))
            {
                layer.pitch.push_back(toDouble(tokens[j]));
                j += 1;
            }
            i += layer.pitch.size(); // 跳过PITCH参数
        }
        else if (token == "OFFSET" && i + 2 < tokens.size())
        {
            layer.routingFactor.push_back("OFFSET");
            int j = i + 1;
            while (j < tokens.size() && isNumber(tokens[j]))
            {
                layer.offset.push_back(toDouble(tokens[j]));
                j += 1;
            }
            i += layer.offset.size(); // 跳过OFFSET参数
        }
        else if (token == "THICKNESS" && i + 1 < tokens.size())
        {
            layer.routingFactor.push_back("THICKNESS");
            layer.thickness = toDouble(tokens[i + 1]);
            i += 1;
        }
        else if (token == "HEIGHT" && i + 1 < tokens.size())
        {
            layer.routingFactor.push_back("HEIGHT");
            layer.height = toDouble(tokens[i + 1]);
            i += 1;
        }
        else if (token == "FILLSPACING" && i + 1 < tokens.size())
        {
            layer.routingFactor.push_back("FILLSPACING");
            layer.fillSpacing = toDouble(tokens[i + 1]);
            i += 1; // 跳过FILLSPACING参数
        }
        else if (token == "WIDTH" && i + 1 < tokens.size())
        {
            layer.routingFactor.push_back("WIDTH");
            layer.width = toDouble(tokens[i + 1]);
            i += 1; // 跳过WIDTH参数
        }
        else if (token == "END" && tokens[i + 1] == layer.name)
        {
            break;
        }
        // 其他未识别的token会被忽略
    }
    lef.addLayer(layer);
}

void LefParser::parsePin(LefFile &lef, const std::vector<std::string> &tokens){

}

void LefParser::parseObs(LefFile &lef, const std::vector<std::string> &tokens){

}

void LefParser::parseMacro(LefFile &lef, const std::vector<std::string> &tokens)
{
    if (tokens.size() < 2)
    {
        std::cerr << "Error: Invalid MACRO statement - missing name" << std::endl;
        return;
    }

    LefMacro macro;
    macro.name = tokens[1];

    // 从tokens的第二个元素开始遍历（跳过MACRO命令和macro name）
    for (size_t i = 2; i < tokens.size(); ++i)
    {
        std::string token = tokens[i];
        if (token == "CLASS" && i + 1 < tokens.size())
        {
            macro.classType = tokens[i + 1];
            i += 1; // 跳过TYPE参数
        }
        else if (token == "FOREIGN" && i + 3 < tokens.size())
        {
            macro.foreign.push_back(tokens[i + 1]);
            macro.foreign.push_back(tokens[i + 2]);
            macro.foreign.push_back(tokens[i + 3]);
            i += 3; // 跳过DIRECTION参数
        }
        else if (token == "ORIGIN" /*&& i + 2 < tokens.size()*/)
        {
            int j = i + 1;
            while (j < tokens.size() && isNumber(tokens[j]))
            {
                macro.origin.push_back(tokens[j]);
                j += 1;
            }
            i += macro.origin.size(); // 跳过ORIGIN参数
        }
        else if (token == "SIZE" && i + 3 < tokens.size())
        {
            macro.size = {tokens[i + 1], tokens[i + 3]};
            i += 3; // 跳过已处理的SIZE参数
        }
        else if (token == "SYMMETRY" && i + 1 < tokens.size())
        {
            // 收集所有SYMMETRY参数直到下一个关键字或结束
            for (size_t j = i + 1; j < tokens.size(); ++j)
            {
                if (tokens[j] == "CLASS" || tokens[j] == "FOREIGN" || tokens[j] == "ORIGIN" || tokens[j] == "SIZE" || tokens[j] == "SYMMETRY" || tokens[j] == "SITE" || tokens[j] == "END" || tokens[j] == "PIN" || tokens[j] == "OBS")
                {
                    break;
                }
                macro.symmetry.push_back(tokens[j]);
            }
            i += macro.symmetry.size(); // 跳过所有symmetry参数
        }
        else if (token == "SITE" && i + 1 < tokens.size())
        {
            macro.site.push_back(tokens[i + 1]);
            i += 1;
        }
        else if (token == "PIN" && i + 1 < tokens.size())
        {
            PIN pin;
            pin.name = tokens[i + 1];
            int j = i + 2;
            for (size_t j = i + 2; j < tokens.size(); ++j)
            {
                const std::string &token = tokens[j];

                if (token == "DIRECTION" && j + 1 < tokens.size())
                {
                    pin.direction = tokens[j + 1];
                    j += 1; // 跳过已处理的DIRECTION参数
                }
                else if (token == "PORT" && j + 1 < tokens.size())
                {
                    for (size_t k = j + 1; k < tokens.size(); ++k)
                    {
                        if (tokens[j] == "CLASS" || tokens[j] == "FOREIGN" || tokens[j] == "ORIGIN" || tokens[j] == "SIZE" || tokens[j] == "SYMMETRY" || tokens[j] == "SITE" || tokens[j] == "END" || tokens[j] == "PIN" || tokens[j] == "OBS" || tokens[j] == "LAYER" || tokens[j] == "RECT")
                        {
                            break;
                        }
                        pin.port.push_back(tokens[k]);
                    }
                    j += pin.port.size(); // 跳过所有port参数
                }
                else if(token == "RECT" && j + 4 < tokens.size())
                {
                    for (size_t k = 1; k <= 4; ++k)
                    {
                        if (isNumber(tokens[j+k]))
                        {
                            pin.layer.push_back(tokens[j+k]);
                        }
                        else{
                            break;
                        }
                    }
                    j += 4; // 跳过所有layer参数
                }
                else if (token == "END" && tokens[j + 1] == pin.name)
                {
                    break;
                }
                // 其他未识别的token会被忽略
            }
            i += j;
            macro.pins.push_back(pin);
        }
        else if (token == "OBS" && i + 1 < tokens.size())
        {
            OBS obs;
            int j = i + 1;
            for (size_t j = i + 1; j < tokens.size(); ++j)
            {
                const std::string &token = tokens[j];

                if (token == "LAYER" && j + 1 < tokens.size())
                {
                    LAYER layer;
                    layer.name = tokens[j + 1];
                    int k = j + 2;
                    for (size_t k = j + 2; k < tokens.size(); ++k)
                    {
                        if (tokens[k] == "RECT" && k + 4 < tokens.size())
                        {
                            for (size_t m = 1; m <= 4; ++m)
                            {
                                if (isNumber(tokens[k + m]))
                                {
                                    layer.rect.push_back(tokens[k + m]);
                                }
                                else
                                {
                                    break;
                                }
                            }
                            k += 4; // 跳过所有rect参数
                        }
                        else if (tokens[k] == "END" && tokens[k + 1] == layer.name)
                        {
                            break;
                        }
                        // 其他未识别的token会被忽略
                    }
                    j += k;
                    obs.layers.push_back(layer);
                }
                else if (token == "END" && tokens[j + 1] == "OBS")
                {
                    break;
                }
                // 其他未识别的token会被忽略
            }
            i += j;
            macro.OBSlayers.push_back(obs);
        }
        else if (token == "END" && tokens[i + 1] == macro.name)
        {
            break;
        }
        // 其他未识别的token会被忽略
    }
    lef.addMacro(macro);
}