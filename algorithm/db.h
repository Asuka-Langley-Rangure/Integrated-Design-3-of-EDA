// placement/db.h
#pragma once
#include "../eigen3/Eigen/Dense"
#include <string>
#include <vector>
#include <unordered_map>
#include <cassert>

struct Cell {
    std::string name;   // 单元名
    double x = 0.0, y = 0.0;  // 坐标
    double width = 1.0, height = 1.0; // 可选尺寸
    bool fixed = false;  // 是否固定（如宏块或 I/O）

    // 显式构造函数
    Cell(std::string n="", double x_=0, double y_=0,
         double w=1.0, double h=1.0, bool f=false)
        : name(std::move(n)), x(x_), y(y_), width(w), height(h), fixed(f) {}
};

struct Net {
    std::string name;          // 网络名
    std::vector<int> pins;     // Cell 索引
};

struct PlacementDB {
    std::vector<Cell> cells;
    std::vector<Net> nets;
    double die_xl = 0, die_yl = 0, die_xh = 0, die_yh = 0;

    // 名称→索引的快速查表
    std::unordered_map<std::string, int> name_to_idx;

    // -------- 坐标向量接口 --------
    Eigen::VectorXd getX() const {
        Eigen::VectorXd x(cells.size());
        for (size_t i = 0; i < cells.size(); ++i) x(i) = cells[i].x;
        return x;
    }

    Eigen::VectorXd getY() const {
        Eigen::VectorXd y(cells.size());
        for (size_t i = 0; i < cells.size(); ++i) y(i) = cells[i].y;
        return y;
    }

    void setXY(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
        assert(x.size() == (int)cells.size());
        assert(y.size() == (int)cells.size());
        for (size_t i = 0; i < cells.size(); ++i) {
            cells[i].x = x(i);
            cells[i].y = y(i);
        }
    }

    // -------- 数据管理接口 --------
    int addCell(const std::string& name, double x=0, double y=0, bool fixed=false) {
        int id = static_cast<int>(cells.size());
        cells.push_back({name, x, y, 1.0, 1.0, fixed});
        name_to_idx[name] = id;
        return id;
    }

    void addNet(const std::string& name, const std::vector<std::string>& conn_names) {
        Net net;
        net.name = name;
        for (const auto& nm : conn_names) {
            auto it = name_to_idx.find(nm);
            if (it != name_to_idx.end()) net.pins.push_back(it->second);
        }
        if (!net.pins.empty()) nets.push_back(std::move(net));
    }

    void dumpToFile(const std::string& outFile) const;

};
