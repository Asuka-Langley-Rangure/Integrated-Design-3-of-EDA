#include "defParser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <regex>
#include <cctype>


DefParser::DefParser(const std::string& fname) : filename(fname) {}

// 静态成员函数实现
// defParser.cpp 中
bool DefParser::getStatement(std::istream& in, std::string& statement) {
    static std::string carry;  // 缓存上次剩下的内容
    statement.clear();
    bool inQuote = false;

    auto trim_left = [](std::string& s){
        size_t p = s.find_first_not_of(" \t\r\n");
        if (p != std::string::npos) s.erase(0, p);
        else s.clear();
    };

    // 辅助函数：在字符串中提取第一条语句
    auto extract_one = [&](std::string& src, std::string& out)->bool {
        bool inQ = false;
        for (size_t i = 0; i < src.size(); ++i) {
            char c = src[i];
            if (c == '"') inQ = !inQ;
            if (c == ';' && !inQ) {
                out = src.substr(0, i);              // 拿到 ';' 前的内容
                src.erase(0, i + 1);                 // 剩下的内容留着
                trim_left(out);
                trim_left(src);
                return !out.empty();
            }
        }
        return false;
    };

    // 1️⃣ 尝试在缓存中提取
    if (!carry.empty()) {
        if (extract_one(carry, statement)) return true;
    }

    // 2️⃣ 持续读取新行并附加到缓存
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        carry += ' ' + line;
        if (extract_one(carry, statement)) {
            return true;
        }
    }

    // 3️⃣ 文件结束但还有未取完的内容
    trim_left(carry);
    if (!carry.empty()) {
        statement = std::move(carry);
        carry.clear();
        return true;
    }

    return false;  // 文件耗尽，无更多语句
}


bool DefParser::parse() {
    std::ifstream fin(filename);
    if (!fin.is_open()) return false;

    std::cout << "Parsing file: " << filename << std::endl;

    std::string line;
    while (getStatement(fin, line)) {
        std::cout << "[DEBUG] Statement: [" << line << "]" << std::endl;
        // 移除首尾空白
        auto start = line.find_first_not_of(" \t");
        auto end = line.find_last_not_of(" \t");
        if (start == std::string::npos) continue;
        line = line.substr(start, end - start + 1);

        if (line.find("VERSION") == 0) parseVersion(line);
        else if (line.find("UNITS") == 0) parseUnits(line);
        else if (line.find("DESIGN") == 0) parseDesign(line);
        else if (line.find("DIEAREA") == 0) parseDieArea(line);
        else if (line.find("DIVIDERCHAR") == 0) parseDividerChar(line);
        else if (line.find("BUSBITCHARS") == 0) parseBusBitChars(line);
        else if (line.find("COMPONENTS") == 0) {
            int count;
            std::istringstream iss(line);
            std::string key;
            iss >> key >> count;
            parseComponents(fin, count);
        }
        else if (line.find("NETS") == 0) {
            int count;
            std::istringstream iss(line);
            std::string key;
            iss >> key >> count;
            parseNets(fin, count);
        }
        else if (line.find("SPECIALNETS") == 0) {
            int count;
            std::istringstream iss(line);
            std::string key;
            iss >> key >> count;
            parseSpecialNets(fin, count);
        }
        else if (line.find("PINS") == 0) {
            int count;
            std::istringstream iss(line);
            std::string key;
            iss >> key >> count;
            parsePins(fin, count);
        }
        else if (line.find("ROW") == 0) parseRow(line);
        else if (line.find("TRACKS") == 0) parseTrack(line);
        else if (line.find("GCELLGRID") == 0) parseGCellGrid(line);
        else parseError(line);
    }

    return true;
}

// ========== 解析函数实现 ==========

void DefParser::parseVersion(const std::string& line) {
    std::istringstream iss(line);
    std::string key, version;
    iss >> key >> version;
    design.version.version = version;
}

void DefParser::parseUnits(const std::string& line) {
    std::istringstream iss(line);
    std::string key, dist, microns;
    int dbu;
    iss >> key >> dist >> microns >> dbu;
    design.units.dbu_per_micron = dbu;
}

void DefParser::parseDesign(const std::string& line) {
    std::istringstream iss(line);
    std::string key, name;
    iss >> key >> name;
    design.name = name;
}

void DefParser::parseDieArea(const std::string& line) {
    int x1, y1, x2, y2;
    char c;
    std::istringstream iss(line);
    std::string key;
    iss >> key >> c >> x1 >> y1 >> c >> c >> x2 >> y2;
    design.die_area = {x1, y1, x2, y2};
}

void DefParser::parseDividerChar(const std::string& line) {
    std::istringstream iss(line);
    std::string key, value;
    iss >> key >> value;
    design.misc.dividerChar = value;
}

void DefParser::parseBusBitChars(const std::string& line) {
    std::istringstream iss(line);
    std::string key, value;
    iss >> key >> value;
    design.misc.busBitChars = value;
}

// ========== COMPONENTS ==========
void DefParser::parseComponents(std::istream& in, int count) {
    std::string stmt;
    int parsed = 0;
    while (parsed < count && getStatement(in, stmt)) {
        if (stmt.find("- ") == 0 || stmt.find("-\t") == 0 || stmt.find("-") == 0) {
            std::istringstream iss(stmt);
            std::string dash, name, type, plus, placed, orient;
            char c;
            int x, y;
            iss >> dash >> name >> type >> plus >> placed;
            if (plus != "+" || placed != "PLACED") continue;
            iss >> c >> x >> y >> c >> orient;
            design.components.push_back({name, type, x, y, orient});
            parsed++;
        }
    }
    if (!getStatement(in, stmt) || stmt.find("END COMPONENTS") != 0) {
        std::cout << "ERROR: Expected 'END COMPONENTS;'" << std::endl;
        exit(1);
    }
}

// void DefParser::parseComponents(std::istream& in, int count) {
//     std::string stmt;
//     int parsed = 0;

//     // - U1 NAND2X1 + PLACED ( 100 200 ) N ;
//     // - U2 INVX1 + FIXED ( 300 400 ) FN ;
//     // - U3 BUF1X + UNPLACED ;
//     static const std::regex head_re(
//         R"(^\s*-\s+(\S+)\s+(\S+))", std::regex::icase);
//     static const std::regex placed_re(
//         R"(\+\s*PLACED\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*([A-Z0-9]+)?)", std::regex::icase);
//     static const std::regex fixed_re(
//         R"(\+\s*FIXED\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*([A-Z0-9]+)?)", std::regex::icase);
//     static const std::regex unplaced_re(
//         R"(\+\s*UNPLACED\b)", std::regex::icase);

//     while (parsed < count && getStatement(in, stmt)) {
//         std::smatch m;
//         if (!std::regex_search(stmt, m, head_re)) {
//             // 不是组件语句，跳过
//             continue;
//         }
//         DefComponent comp;
//         comp.name  = m[1];
//         comp.macro = m[2];

//         std::smatch pm;
//         if (std::regex_search(stmt, pm, placed_re)) {
//             comp.x = std::stoi(pm[1]);
//             comp.y = std::stoi(pm[2]);
//             comp.orient = pm[3].matched ? pm[3].str() : "";
//             comp.placed = true;
//             comp.fixed  = false;
//         } else if (std::regex_search(stmt, pm, fixed_re)) {
//             comp.x = std::stoi(pm[1]);
//             comp.y = std::stoi(pm[2]);
//             comp.orient = pm[3].matched ? pm[3].str() : "";
//             comp.placed = true;
//             comp.fixed  = true;
//         } else if (std::regex_search(stmt, pm, unplaced_re)) {
//             comp.placed = false; // 无坐标
//             comp.fixed  = false;
//             comp.orient.clear();
//         } else {
//             // 没声明放置状态：视作未放置
//             comp.placed = false;
//             comp.fixed  = false;
//             comp.orient.clear();
//         }

//         design.components.push_back(std::move(comp));
//         ++parsed;
//     }

//     // 这里通常由外层消费 "END COMPONENTS ;"
//     // 如果你的原设计让 parseComponents 自己消费 END，可在此处再调用一次 getStatement 验证：
//     // std::string endStmt;
//     // if (!(getStatement(in, endStmt) && endStmt.find("END COMPONENTS") == 0)) { /* 报错或容错 */ }
// }


// ========== NETS ==========
// void DefParser::parseNets(std::istream& in, int count) {
//     std::string line;
//     int parsed = 0;
//     while (parsed < count && std::getline(in, line)) {
//         if (line.find("-") == 0) {
//             // 例子: - net1 ( inst1 A ) ( inst2 B ) ;
//             std::istringstream iss(line);
//             std::string dash, netname;
//             iss >> dash >> netname;
//             DefNet net;
//             net.name = netname;

//             std::string token;
//             while (iss >> token) {
//                 if (token == "(") {
//                     std::string inst, pin, close;
//                     iss >> inst >> pin >> close; // close = ")"
//                     net.connections.push_back({inst, pin});
//                 }
//             }
//             design.nets.push_back(net);
//             parsed++;
//         }
//     }
//     if(std::getline(in, line)){
//         if(line.find("END NETS") == 0 && parsed == count){
//             return;
//         }
//         else
//         {
//             std::cout << "ERROR: NETS section is not closed properly" << std::endl;
//             exit(1);
//         }
//     }
// }

// static inline bool looksCoord(const std::string& s) {
//     if (s.empty()) return false;
//     for (unsigned char c : s) {
//         if (!std::isdigit(c) && c!='-' && c!='+') return false;
//     }
//     return true;
// }
static inline bool looksCoord(const std::string& s) {
    if (s.empty()) return false;
    for (unsigned char c : s) {
        if (!std::isdigit(c) && c!='-' && c!='+') return false;
    }
    return true;
}
void DefParser::parseNets(std::istream& in, int /*count_hint*/) {
    using std::regex; using std::smatch; using std::regex_search; using std::sregex_iterator;
    std::string stmt;

    // 以分号为单位逐条读取；getStatement 需为“带残余缓存”的版本
    // 头部匹配 "- netname"
    static const regex head_re    (R"(^\s*-\s+(\S+))", std::regex::icase);
    // 批量匹配所有 ( inst pin )
    static const regex pin_pair_re(R"(\(\s*([^\s\)]+)\s+([^\s\)]+)\s*\))");

    while (getStatement(in, stmt)) {
        // 1) 结束：END NETS
        if (stmt.rfind("END NETS", 0) == 0) break;

        // 2) 必须从 '-' 开头；否则是空、注释或杂质，跳过
        smatch m;
        if (!regex_search(stmt, m, head_re)) continue;

        DefNet net{};
        net.name = m[1];

        // 3) 把所有 ( inst pin ) 扫出来，并过滤坐标对
        for (sregex_iterator it(stmt.begin(), stmt.end(), pin_pair_re), ed;
             it != ed; ++it) {
            const std::string inst = (*it)[1];
            const std::string pin  = (*it)[2];
            if (looksCoord(inst) || looksCoord(pin)) continue;
            net.connections.push_back({inst, pin});
        }

        design.nets.push_back(std::move(net));
    }
}



// ========== SPECIALNETS ==========
void DefParser::parseSpecialNets(std::istream& in, int count) {
    std::string stmt;
    int parsed = 0;
    DefSpecialNet currentNet;

    while (parsed < count && getStatement(in, stmt)) {
        if (stmt.find("- ") == 0) {
            if (!currentNet.name.empty()) {
                design.specialnets.push_back(currentNet);
                currentNet = DefSpecialNet{};
                parsed++;
            }
            std::istringstream iss(stmt);
            std::string dash, netname;
            iss >> dash >> netname;
            currentNet.name = netname;

            std::string token;
            while (iss >> token) {
                if (token == "(") {
                    std::string inst, pin, close;
                    if (iss >> inst >> pin >> close && close == ")") {
                        currentNet.connections.push_back({inst, pin});
                    }
                }
            }
        }
        else if (stmt.find("+") == 0) {
            std::istringstream iss(stmt);
            std::string plus, type;
            iss >> plus >> type;
            DefSpecialWire wire{type};
            if (type == "ROUTED" || type == "NEW") {
                iss >> wire.layer >> wire.width;
                std::string token;
                while (iss >> token) {
                    if (token == "(") {
                        int x, y;
                        char c;
                        if (iss >> x >> y >> c && c == ')') {
                            wire.points.emplace_back(x, y);
                        }
                    } else if (token == "SHAPE") {
                        iss >> wire.shape;
                    }
                }
                currentNet.wires.push_back(wire);
            }
        }
        else if (stmt.find("END SPECIALNETS") == 0) {
            if (!currentNet.name.empty()) {
                design.specialnets.push_back(currentNet);
                parsed++;
            }
            break;
        }
    }
    if (parsed != count) {
        std::cout << "WARNING: SPECIALNETS count mismatch" << std::endl;
    }
}

// ========== PINS ==========
void DefParser::parsePins(std::istream& in, int count) {
    std::string stmt;
    int parsed = 0;
    while (parsed < count && getStatement(in, stmt)) {
        if (stmt.find("- ") == 0) {
            std::istringstream iss(stmt);
            std::string dash, name, plus, key;
            DefPin pin;
            iss >> dash >> name;
            pin.name = name;

            while (iss >> plus >> key) {
                if (key == "NET") iss >> pin.net;
                else if (key == "DIRECTION") iss >> pin.direction;
                else if (key == "LAYER") {
                    iss >> pin.layer;
                    char c;
                    if (iss >> c && c == '(' && iss >> pin.x >> pin.y >> c && c == ')') {}
                    else parseError("Invalid pin layer: " + stmt);
                }
            }
            design.pins.push_back(pin);
            parsed++;
        }
        else if (stmt.find("END PINS") == 0) {
            break;
        }
    }
}

// void DefParser::parsePins(std::istream& in, int count) {
//     std::string stmt;
//     int parsed = 0;

//     // - PIN_A + NET net1 + DIRECTION INPUT
//     //   + PORT
//     //     + LAYER metal2 ( lx ly ) ( ux uy )
//     //   + PLACED ( 100 200 ) N ;
//     static const std::regex head_re(
//         R"(^\s*-\s+(\S+))", std::regex::icase);
//     static const std::regex net_re(
//         R"(\+\s*NET\s+(\S+))", std::regex::icase);
//     static const std::regex dir_re(
//         R"(\+\s*DIRECTION\s+(\S+))", std::regex::icase);
//     static const std::regex placed_re(
//         R"(\+\s*PLACED\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*([A-Z0-9]+)?)", std::regex::icase);

//     while (parsed < count && getStatement(in, stmt)) {
//         std::smatch m;
//         if (!std::regex_search(stmt, m, head_re)) continue;

//         DefPin pin;
//         pin.name = m[1];

//         if (std::regex_search(stmt, m, net_re)) pin.net = m[1];
//         if (std::regex_search(stmt, m, dir_re)) pin.direction = m[1];
//         if (std::regex_search(stmt, m, placed_re)) {
//             pin.x = std::stoi(m[1]);
//             pin.y = std::stoi(m[2]);
//             pin.orient = m[3].matched ? m[3].str() : "";
//             pin.placed = true;
//         } else {
//             pin.placed = false;
//         }

//         design.pins.push_back(std::move(pin));
//         ++parsed;
//     }

//     // 同样：若由本函数消费 END，则在此处验证 "END PINS ;"
// }

// ========== ROW / TRACKS / GCELLGRID ==========
void DefParser::parseRow(const std::string& line) {
    std::istringstream iss(line);
    std::string key, name, site, orient, do_str, by_str, step_str;
    int x, y, do_x, by_y, step_x, step_y;
    iss >> key >> name >> site >> x >> y >> orient >> do_str >> do_x >> by_str >> by_y >> step_str >> step_x >> step_y;
    design.rows.push_back({name, site, x, y, orient, do_x, by_y, step_x, step_y});
}

void DefParser::parseTrack(const std::string& line) {
    std::istringstream iss(line);
    std::string key, do_str, step_str, layer_str, layer;
    char direction;
    int start, count, step;
    iss >> key >> direction >> start >> do_str >> count >> step_str >> step >> layer_str >> layer;
    design.tracks.push_back({direction, start, count, step, layer});
}

void DefParser::parseGCellGrid(const std::string& line) {
    std::istringstream iss(line);
    std::string key, do_str, step_str;
    char direction;
    int start, count, step;
    iss >> key >> direction >> start >> do_str >> count >> step_str >> step;
    design.gcells.push_back({direction, start, count, step});
}

void DefParser::parseError(const std::string& line) {
    std::cout << "Error: " << line << std::endl;
}