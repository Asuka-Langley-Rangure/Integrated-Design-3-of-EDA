#include "defParser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <regex>
#include <cctype>

DefParser::DefParser(const std::string& fname) : filename(fname) {}


bool DefParser::parse() {
    std::ifstream in(filename);
    if (!in) return false;

    std::string line;
    while (std::getline(in, line)) {
        // 去掉行首空白
        auto pos = line.find_first_not_of(" \t\r");
        if (pos == std::string::npos) continue;
        const std::string head = line.substr(pos);

        if (head.rfind("VERSION", 0) == 0)        { parseVersion(head); continue; }
        if (head.rfind("DIVIDERCHAR", 0) == 0)    { parseDividerChar(head); continue; }
        if (head.rfind("BUSBITCHARS", 0) == 0)    { parseBusBitChars(head); continue; }
        if (head.rfind("DESIGN", 0) == 0)         { parseDesign(head); continue; }
        if (head.rfind("UNITS", 0) == 0)          { parseUnits(head); continue; }
        if (head.rfind("DIEAREA", 0) == 0)        { parseDieArea(head); continue; }
        if (head.rfind("ROW", 0) == 0)            { parseRow(head); continue; }
        if (head.rfind("TRACKS", 0) == 0)         { parseTrack(head); continue; }
        if (head.rfind("GCELLGRID", 0) == 0)      { parseGCellGrid(head); continue; }

        // 块段：COMPONENTS / NETS / PINS / SPECIALNETS
        if (head.rfind("COMPONENTS", 0) == 0) {
            int count = 0; std::sscanf(head.c_str(), "COMPONENTS %d", &count);
            parseComponents(in, count);  // 由它消费直到 "END COMPONENTS ;"
            continue;
        }
        if (head.rfind("PINS", 0) == 0) {
            int count = 0; std::sscanf(head.c_str(), "PINS %d", &count);
            std::cerr << "[NETS] header count=" << count << "\n";
            parsePins(in, count);        // 消费到 "END PINS ;"
            continue;
        }
        if (head.rfind("SPECIALNETS", 0) == 0) {
            std::cerr << "[DEBUG] entering parseSpecialNets()\n";
            int count = 0; std::sscanf(head.c_str(), "SPECIALNETS %d", &count);
            parseSpecialNets(in, count); // 消费到 "END SPECIALNETS ;"
            continue;
        }
        if (head.find("NETS") != std::string::npos && head.find("NETS") < 5) {
            std::cerr << "[DEBUG] entering parseNets()\n";
            int count = 0; std::sscanf(head.c_str(), "NETS %d", &count);
            parseNets(in, count);        // 消费到 "END NETS ;"
            continue;
        }
        

        // 其它不识别的行
        parseError(head);
    }
    return true;
}

bool DefParser::getStatement(std::istream& in, std::string& statement) {
    static std::string carry;  // 残余缓冲
    statement.clear();

    auto trim_left = [](std::string& s) {
        size_t p = s.find_first_not_of(" \t\r\n");
        if (p == std::string::npos) s.clear();
        else if (p > 0) s.erase(0, p);
    };

    auto extract_one = [&](std::string& src, std::string& out)->bool {
        bool inQuote = false;
        for (size_t i=0; i<src.size(); ++i) {
            char c = src[i];
            if (c == '"') inQuote = !inQuote;
            if (c == ';' && !inQuote) {
                out = src.substr(0, i+1);  // 保留分号
                src.erase(0, i+1);
                trim_left(out);
                // 不 trim src 的左侧 '-' 避免吞掉下一句
                return !out.empty();
            }
        }
        return false;
    };

    // 先处理残余
    if (!carry.empty()) {
        if (extract_one(carry, statement))
            return true;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        trim_left(line);
        if (!carry.empty()) carry += ' ';
        carry += line;
        if (extract_one(carry, statement))
            return true;
    }

    trim_left(carry);
    if (!carry.empty()) {
        statement = std::move(carry);
        carry.clear();
        return true;
    }
    return false;
}



void DefParser::parseComponents(std::istream& in, int count) {
    using std::regex; using std::smatch; using std::regex_search; using std::sregex_iterator;
    std::string stmt; int parsed = 0;

    static const regex head_re(R"(^\s*-\s+(\S+)\s+(\S+))", std::regex::icase);
    static const regex placed_re(R"(\+\s*PLACED\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*([A-Z0-9]+)?)", std::regex::icase);
    static const regex fixed_re (R"(\+\s*FIXED\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*([A-Z0-9]+)?)", std::regex::icase);
    static const regex unpl_re  (R"(\+\s*UNPLACED\b)", std::regex::icase);

    while (parsed < count && getStatement(in, stmt)) {
        if (stmt.rfind("END COMPONENTS", 0) == 0) break;

        smatch m;
        if (!regex_search(stmt, m, head_re)) continue;

        DefComponent c{};
        c.name  = m[1];
        c.macro = m[2];

        smatch pm;
        if (regex_search(stmt, pm, placed_re)) {
            c.x = std::stoi(pm[1]); c.y = std::stoi(pm[2]);
            c.orient = pm[3].matched ? pm[3].str() : "";
            c.placed = true; c.fixed = false;
        } else if (regex_search(stmt, pm, fixed_re)) {
            c.x = std::stoi(pm[1]); c.y = std::stoi(pm[2]);
            c.orient = pm[3].matched ? pm[3].str() : "";
            c.placed = true; c.fixed = true;
        } else if (regex_search(stmt, pm, unpl_re)) {
            c.placed = false; c.fixed = false; c.orient.clear();
        } else {
            c.placed = false; c.fixed = false; c.orient.clear();
        }

        design.components.push_back(std::move(c));
        ++parsed;
    }

    // 消费 END COMPONENTS ;
    if (stmt.rfind("END COMPONENTS", 0) != 0) {
        std::string endStmt;
        if (!(getStatement(in, endStmt) && endStmt.find("END COMPONENTS") == 0)) {
            parseError("Missing 'END COMPONENTS;'");
        }
    }
}

void DefParser::parsePins(std::istream& in, int count) {
    using std::regex; using std::smatch; using std::regex_search;
    std::string stmt; int parsed = 0;

    static const regex head_re   (R"(^\s*-\s+(\S+))", std::regex::icase);
    static const regex net_re    (R"(\+\s*NET\s+(\S+))", std::regex::icase);
    static const regex dir_re    (R"(\+\s*DIRECTION\s+(\S+))", std::regex::icase);
    static const regex placed_re (R"(\+\s*PLACED\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*([A-Z0-9]+)?)", std::regex::icase);

    while (parsed < count && getStatement(in, stmt)) {
        if (stmt.rfind("END PINS", 0) == 0) break;

        smatch m;
        if (!regex_search(stmt, m, head_re)) continue;

        DefPin p{};
        p.name = m[1];

        if (regex_search(stmt, m, net_re)) p.net = m[1];
        if (regex_search(stmt, m, dir_re)) p.direction = m[1];
        if (regex_search(stmt, m, placed_re)) {
            p.x = std::stoi(m[1]); p.y = std::stoi(m[2]);
            p.orient = m[3].matched ? m[3].str() : "";
            p.placed = true;
        } else {
            p.placed = false;
        }

        design.pins.push_back(std::move(p));
        ++parsed;
    }

    if (stmt.rfind("END PINS", 0) != 0) {
        std::string endStmt;
        if (!(getStatement(in, endStmt) && endStmt.find("END PINS") == 0)) {
            parseError("Missing 'END PINS;'");
        }
    }
}

static inline bool looksCoord(const std::string& s) {
    if (s.empty()) return false;
    for (unsigned char c : s)
        if (!std::isdigit(c) && c!='-' && c!='+') return false;
    return true;
}

void DefParser::parseNets(std::istream& in, int /*count_hint*/) {
    using std::regex; using std::smatch; using std::sregex_iterator;
    std::string stmt;

    // 头部： - netName
    static const regex head_re(R"(^\s*-\s+(\S+))", std::regex::icase);
    // 端点：( inst pin ) —— inst 允许字母/下划线开头，避免把坐标当成实例
    static const regex pin_pair_re(R"(\(\s*([A-Za-z_][^\s\)]*)\s+([^\s\)]+)\s*\))");

    size_t netCount = 0, pairKept = 0, pairSkipCoord = 0;

    while (getStatement(in, stmt)) {
        if (stmt.rfind("END NETS", 0) == 0) break;
        //std::cerr << "[NETS-STMT] >>> " << stmt.substr(0, 80) << "\n";
        smatch m;
        if (!std::regex_search(stmt, m, head_re)) {
            // 不是一条 net 语句（可能是空/噪声），跳过
            continue;
        }

        DefNet net{};
        net.name = m[1];

        for (sregex_iterator it(stmt.begin(), stmt.end(), pin_pair_re), ed;
             it != ed; ++it) {
            const std::string inst = (*it)[1];
            const std::string pin  = (*it)[2];
            if (looksCoord(inst) || looksCoord(pin)) { ++pairSkipCoord; continue; }
            net.connections.push_back({inst, pin});
            ++pairKept;
        }

        design.nets.push_back(std::move(net));
        ++netCount;
    }

    //你也可以在这里放一点调试日志：
    std::cerr << "[NETS] parsed nets=" << netCount
              << " pairs kept=" << pairKept
              << " coord-like skipped=" << pairSkipCoord << "\n";
}


void DefParser::parseVersion(const std::string& line) {
    std::istringstream iss(line);
    std::string key, version;
    iss >> key >> version;
    design.version.version = version;
}

void DefParser::parseUnits(const std::string& line) {
    // UNITS DISTANCE MICRONS 1000 ;
    int dbu=0;
    if (std::sscanf(line.c_str(), "UNITS DISTANCE MICRONS %d", &dbu) == 1) {
        design.units.dbu_per_micron = dbu; // 若有字段
    }
}

void DefParser::parseDesign(const std::string& line) {
    // DESIGN mytop ;
    char name[1024] = {0};
    if (std::sscanf(line.c_str(), "DESIGN %1023s", name) == 1) {
        // 去掉尾部 ';'
        std::string n(name); if (!n.empty() && n.back()==';') n.pop_back();
        design.name = n;
    }
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

void DefParser::parseRow(const std::string& line) {
    // 示例: ROW CORE_ROW_0 CoreSite 2000 2000 FS DO 1464 BY 1 STEP 200 0 ;
    DefRow row;
    std::istringstream iss(line);
    std::string keyword, dummy;

    iss >> keyword; // ROW
    iss >> row.name >> row.site >> row.x >> row.y >> row.orient;

    // 可选: DO <count_x> BY <count_y> STEP <step_x> <step_y>
    std::string token;
    while (iss >> token) {
        if (token == "DO") iss >> row.do_count_x;
        else if (token == "BY") iss >> row.do_count_y;
        else if (token == "STEP") iss >> row.step_x >> row.step_y;
    }

    design.rows.push_back(std::move(row));
}


void DefParser::parseTrack(const std::string& line) {
    // 示例: TRACKS Y 200 DO 730 STEP 400 LAYER Metal9 ;
    DefTrack track;
    std::istringstream iss(line);
    std::string keyword, dummy;
    iss >> keyword; // TRACKS
    iss >> track.direction >> track.start >> dummy >> track.count >> dummy >> track.step;

    // 可选层信息
    std::string token;
    while (iss >> token) {
        if (token == "LAYER") {
            std::string layer;
            iss >> layer;
            track.layer = layer;
            break;
        }
    }

    design.tracks.push_back(std::move(track));
}


void DefParser::parseGCellGrid(const std::string& line) {
    // 示例: GCELLGRID X 296100 DO 2 STEP 700 ;
    DefGCellGrid gcell;
    std::istringstream iss(line);
    std::string keyword, dummy;
    iss >> keyword; // GCELLGRID
    iss >> gcell.direction >> gcell.start >> dummy >> gcell.count >> dummy >> gcell.step;

    design.gcells.push_back(std::move(gcell));
}


void DefParser::parseSpecialNets(std::istream& in, int /*count_hint*/) {
    using std::regex; using std::smatch; using std::sregex_iterator;
    std::string line, buffer;
    bool inNet = false;
    DefSpecialNet currentNet;

    static const regex head_re(R"(^\s*-\s+(\S+))", std::regex::icase);
    static const regex pair_re(R"(\(\s*([^\s\)]+)\s+([^\s\)]+)\s*\))");

    size_t netCount = 0, pairCount = 0, skipCoord = 0;

    std::cerr << "[DEBUG] entering parseSpecialNets() at file pos=" << in.tellg() << "\n";

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // 忽略空行
        if (line.find_first_not_of(" \t") == std::string::npos)
            continue;

        // ✅ 碰到 END SPECIALNETS 就结束
        if (line.find("END SPECIALNETS") != std::string::npos) {
            if (inNet) {
                design.specialnets.push_back(std::move(currentNet));
                ++netCount;
                inNet = false;
            }
            break;
        }

        // ---------- 判断是否是新的 net ----------
        smatch m;
        if (regex_search(line, m, head_re)) {
            // 保存上一个 net
            if (inNet) {
                design.specialnets.push_back(std::move(currentNet));
                ++netCount;
                inNet = false;
            }

            currentNet = DefSpecialNet{};
            currentNet.name = m[1];
            inNet = true;
            continue;
        }

        // ---------- 若在一个 net 内，提取 (inst pin) ----------
        if (inNet) {
            for (sregex_iterator it(line.begin(), line.end(), pair_re), ed; it != ed; ++it) {
                std::string first = (*it)[1];
                std::string second = (*it)[2];

                // 只有两个字段都是纯数字/符号时才跳过坐标对
                auto isCoordLike = [](const std::string& s){
                    if (s.empty()) return false;
                    return std::all_of(s.begin(), s.end(),
                                    [](char c){return std::isdigit(c)||c=='+'||c=='-';});
                };
                if (isCoordLike(first) && isCoordLike(second)) { ++skipCoord; continue; }


                currentNet.connections.push_back({first, second});
                ++pairCount;
            }
        }
    }

    std::cerr << "[DEBUG] leaving parseSpecialNets() at file pos=" << in.tellg() << "\n";
    std::cerr << "[SPECIALNETS] parsed specialnets=" << netCount
              << " pairs=" << pairCount
              << " skipped=" << skipCoord << "\n";
}



void DefParser::parseError(const std::string& line) {
    // 你可以选择静默、打印 warning，或记录未知行便于调试
    // std::cerr << "[WARN] Unrecognized line: " << line << "\n";
}

