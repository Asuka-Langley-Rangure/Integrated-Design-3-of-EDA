#pragma once
#include <string>
#include <vector>
#include <utility> // for std::pair

// =================== 基本结构体 ===================

// 单位定义
struct LefUnits {
    int databaseMicrons = 0; // e.g., 2000
};

// 站点定义
struct LefSite {
    std::string name;
    std::pair<double, double> size; // width, height
    std::string siteClass;          // e.g., "CORE"
    std::vector<std::string> symmetry; // e.g., {"Y", "X"}
};

// 层定义
struct LefLayer {
    std::string name;
    std::string type; // e.g., "ROUTING", "CUT"

    std::vector<std::string> routingFactor; // e.g., {"Y", "X"}

    // ROUTING 层特有
    std::string direction;
    std::vector<double> pitch;
    std::vector<double> offset;
    double height = 0.0;
    double thickness = 0.0;
    double fillSpacing = 0.0;
    double width = 0.0;
};

// 宏单元定义（可扩展）
class LAYER {
public:
    std::string name;
    std::vector<std::string> rect;
};
class PIN {
public:
    std::string name;
    std::string direction;
    std::vector<std::string> port;
    std::vector<std::string> layer;
};
class OBS {
public:
    std::vector<LAYER> layers;
};
struct LefMacro {
    std::string name;
    std::string classType;
    std::vector<std::string> foreign; // 外部引用
    std::vector<std::string> origin;
    std::vector<std::string> size;
    std::vector<std::string> symmetry;
    std::vector<std::string> site; // 站点名
    
    std::vector<PIN> pins;
    std::vector<OBS> OBSlayers;
};

// =================== 顶层类 ===================
class LefFile {
public:
    std::string version;
    std::string busBitChars;
    std::string dividerChar;
    LefUnits units;
    double manufacturingGrid = 0.0;

    std::vector<LefSite> sites;
    std::vector<LefLayer> layers;
    std::vector<LefMacro> macros;

public:
    LefFile() = default;

    // 添加元素
    void addSite(const LefSite& site);
    void addLayer(const LefLayer& layer);
    void addMacro(const LefMacro& macro);

    // 查询方法
    LefLayer* findLayer(const std::string& name);
    LefSite* findSite(const std::string& name);

    // 调试输出
    void printSummary() const;
};
