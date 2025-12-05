#pragma once
#include <string>
#include <vector>

// ===== 基本数据结构 =====

// 必须先定义 DefConnection，因为 DefNet 会用到它
struct DefConnection {
    std::string instance;
    std::string pin;
    DefConnection() = default;
    DefConnection(const std::string& inst, const std::string& p) : instance(inst), pin(p) {}
};

struct DefVersion {
    std::string version;
};

struct DefUnits {
    int dbu_per_micron = 0;
};

struct DefDieArea {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
};

struct DefComponent {
    std::string name;      // 实例名
    std::string type;      // 单元类型
    int x = 0, y = 0;      // 坐标
    std::string orient;    // 方向 (N, S, FS...)
    bool placed=false;
    bool fixed = false;
    std::string macro;
};

struct DefNet {
    std::string name;
    std::vector<DefConnection> connections;
    DefNet() = default;
};

struct DefRow {
    std::string name;
    std::string site;
    int x = 0, y = 0;
    std::string orient;
    int do_count_x = 0;
    int do_count_y = 0;
    int step_x = 0;
    int step_y = 0;
};

struct DefTrack {
    char direction;
    int start = 0;
    int count = 0;
    int step = 0;
    std::string layer;
};

struct DefGCellGrid {
    char direction;
    int start = 0;
    int count = 0;
    int step = 0;
};

struct DefSpecialWire {
    std::string type;
    std::string layer;
    int width = 0;
    std::vector<std::pair<int, int>> points;
    std::string shape;
};

struct DefSpecialNet {
    std::string name;
    std::vector<DefConnection> connections;
    std::vector<DefSpecialWire> wires;
};

struct DefMisc {
    std::string dividerChar;
    std::string busBitChars;
};

struct DefPin {
    std::string name;
    std::string net;
    int x = 0, y = 0;
    std::string layer;
    std::string direction, orient;
    bool placed = false;
};

// ===== 整体设计对象 =====
struct DefDesign {
    std::string name;
    DefVersion version;
    DefUnits units;
    DefDieArea die_area;
    DefMisc misc;
    std::vector<DefComponent> components;
    std::vector<DefNet> nets;
    std::vector<DefRow> rows;
    std::vector<DefTrack> tracks;
    std::vector<DefGCellGrid> gcells;
    std::vector<DefSpecialNet> specialnets;
    std::vector<DefPin> pins;
};

// ===== 解析器类 =====
class DefParser {
public:
    DefParser(const std::string& filename);
    bool parse();
    const DefDesign& getDesign() const { return design; }

private:
    std::string filename;
    DefDesign design;

    // 工具函数：按分号读取完整语句
    static bool getStatement(std::istream& in, std::string& statement);

    // 解析函数
    void parseVersion(const std::string& line);
    void parseUnits(const std::string& line);
    void parseDesign(const std::string& line);
    void parseDieArea(const std::string& line);
    void parseComponents(std::istream& in, int count);
    void parseNets(std::istream& in, int count);
    void parseRow(const std::string& line);
    void parseTrack(const std::string& line);
    void parseGCellGrid(const std::string& line);
    void parseSpecialNets(std::istream& in, int count);
    void parseDividerChar(const std::string& line);
    void parseBusBitChars(const std::string& line);
    void parsePins(std::istream& in, int count);
    void parseError(const std::string& line);
};