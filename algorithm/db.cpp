// placement/db_debug.cpp
#include "db.h"
#include <fstream>
#include <iostream>
#include <unordered_set>

// 将 PlacementDB 的内容完整 dump 到文本文件，并做一些基本检查
void dumpPlacementDB(const PlacementDB& db, const std::string& outFile)
{
    std::ofstream ofs(outFile);
    if (!ofs) {
        std::cerr << "[ERR] dumpPlacementDB: cannot open file " << outFile << "\n";
        return;
    }

    ofs << "================== [ PlacementDB Dump ] ==================\n\n";

    // --- 基本信息 ---
    ofs << "DieArea: (" << db.die_xl << ", " << db.die_yl << ") - ("
        << db.die_xh << ", " << db.die_yh << ")\n";
    ofs << "Cells  : " << db.cells.size() << "\n";
    ofs << "Nets   : " << db.nets.size()  << "\n";
    ofs << "Name->Idx map size: " << db.name_to_idx.size() << "\n\n";

    // --- 检查 name_to_idx 是否和 cells 一致 ---
    ofs << "=== Name -> Index Consistency Check ===\n";
    bool nameMapOK = true;
    for (const auto& kv : db.name_to_idx) {
        const std::string& name = kv.first;
        int idx = kv.second;
        if (idx < 0 || idx >= (int)db.cells.size()) {
            ofs << "[WARN] name_to_idx entry out of range: "
                << name << " -> " << idx << " (cells.size=" << db.cells.size() << ")\n";
            nameMapOK = false;
            continue;
        }
        const auto& c = db.cells[idx];
        if (c.name != name) {
            ofs << "[WARN] name_to_idx mismatch: "
                << name << " -> " << idx
                << ", but cells[" << idx << "].name = " << c.name << "\n";
            nameMapOK = false;
        }
    }
    if (nameMapOK) {
        ofs << "[OK] All name_to_idx entries are consistent.\n";
    }
    ofs << "\n";

    // --- 列出所有 Cell ---
    ofs << "=== Cells (" << db.cells.size() << ") ===\n";
    for (size_t i = 0; i < db.cells.size(); ++i) {
        const auto& c = db.cells[i];
        ofs << "  [" << i << "] " << c.name
            << "  pos=(" << c.x << ", " << c.y << ")"
            << "  size=(" << c.width << " x " << c.height << ")"
            << "  fixed=" << (c.fixed ? "true" : "false")
            << "\n";
    }
    ofs << "\n";

    // --- 列出所有 Net，并顺便统计被引用的 cell ---
    ofs << "=== Nets (" << db.nets.size() << ") ===\n";
    std::unordered_set<int> referencedCells;
    for (size_t i = 0; i < db.nets.size(); ++i) {
        const auto& net = db.nets[i];
        ofs << "  [" << i << "] Net " << net.name
            << "  pins=" << net.pins.size() << "\n";

        for (size_t j = 0; j < net.pins.size(); ++j) {
            int cid = net.pins[j];
            if (cid < 0 || cid >= (int)db.cells.size()) {
                ofs << "       [WARN] invalid cell index " << cid
                    << " in net " << net.name << "\n";
                continue;
            }
            const auto& c = db.cells[cid];
            ofs << "       (" << cid << ") " << c.name
                << "  pos=(" << c.x << ", " << c.y << ")"
                << (c.fixed ? " [FIXED]" : "")
                << "\n";
            referencedCells.insert(cid);
        }
    }
    ofs << "\n";

    // --- 统计：哪些 cell 从未出现在任何 net 中 ---
    ofs << "=== Unconnected Cells (not referenced by any net) ===\n";
    bool anyUnconnected = false;
    for (int i = 0; i < (int)db.cells.size(); ++i) {
        if (referencedCells.find(i) == referencedCells.end()) {
            const auto& c = db.cells[i];
            ofs << "  [" << i << "] " << c.name
                << "  pos=(" << c.x << ", " << c.y << ")"
                << (c.fixed ? " [FIXED]" : "")
                << "\n";
            anyUnconnected = true;
        }
    }
    if (!anyUnconnected) {
        ofs << "  [OK] Every cell is referenced by at least one net.\n";
    }
    ofs << "\n";

    // --- 简单总结 ---
    ofs << "================== [ End of PlacementDB Dump ] ==================\n";
    ofs.close();

    std::cerr << "[INFO] PlacementDB dumped to " << outFile << "\n";
}

void PlacementDB::dumpToFile(const std::string& outFile) const {
    dumpPlacementDB(*this, outFile);
}

