#include "DefPlaceAdapter.h"
#include "../Common/objects.h"
#include "../Common/common.h"
#include <limits>

namespace DefPlaceAdapter {

    /// 辅助：清空 PlaceData（不 delete 指针，对象回收可以统一在别处处理）
    static void resetPlaceData(PlaceData& db) {
        db.Nodes.clear();
        db.Terminals.clear();
        db.Pins.clear();
        db.Nets.clear();
        db.SiteRows.clear();
        db.moduleMap.clear();
        db.bins.clear();

        db.moduleCount   = 0;
        db.terminalCount = 0;
        db.macroCount    = 0;
        db.netCount      = 0;
        db.pinCount      = 0;

        db.binRows = 0;
        db.binCols = 0;
        db.siteHeight = 0.0f;
        // chipRegion 在构造函数里已经有默认值了，这里可以不动
    }

    // ============================================================
    //  PlacementDB -> PlaceData
    // ============================================================
    void buildPlaceDataFromPlacementDB(const PlacementDB& src, PlaceData& dst) {
        resetPlaceData(dst);

        // -------- 0) 用 PlacementDB 的 die_* 更新 chipRegion（如果合法）--------
        if (src.die_xl < src.die_xh && src.die_yl < src.die_yh) {
            dst.chipRegion.ll = POS_2D(static_cast<float>(src.die_xl),
                                       static_cast<float>(src.die_yl));
            dst.chipRegion.ur = POS_2D(static_cast<float>(src.die_xh),
                                       static_cast<float>(src.die_yh));
        }

        // -------- 1) cells -> Modules --------
        // 建一个 index -> Module* 的数组，后面建 net/pin 时会用到
        std::vector<Module*> index2Module(src.cells.size(), nullptr);
        dst.Nodes.reserve(src.cells.size());

        for (size_t i = 0; i < src.cells.size(); ++i) {
            const Cell& c = src.cells[i];

            Module* m = new Module();   // Module() 里会调用 Init()

            // 名字
            m->name = c.name;

            // 坐标与尺寸
            m->center.x = static_cast<float>(c.x);
            m->center.y = static_cast<float>(c.y);
            m->width    = static_cast<float>(c.width);
            m->height   = static_cast<float>(c.height);
            m->area     = m->width * m->height;

            // 其他属性
            m->orientation = 'N';
            m->isFixed  = c.fixed;
            m->isMacro  = false;   // 这里暂时不区分宏块，你之后可以根据宽高等再细分
            m->isFiller = false;

            m->idx = static_cast<int>(dst.Nodes.size());

            // 简单起见：所有 cell 先都当成 Nodes（包括 fixed）
            dst.Nodes.push_back(m);
            dst.moduleMap[m->name] = m;

            index2Module[i] = m;
        }

        dst.moduleCount   = static_cast<int>(dst.Nodes.size());
        dst.terminalCount = static_cast<int>(dst.Terminals.size());
        // 统计一下 isMacro
        int macroCnt = 0;
        for (Module* m : dst.Nodes) {
            if (m && m->isMacro) ++macroCnt;
        }
        dst.macroCount = macroCnt;

        // -------- 2) nets + pins --------
        dst.Nets.reserve(src.nets.size());
        for (size_t ni = 0; ni < src.nets.size(); ++ni) {
            const DbNet& dnet = src.nets[ni];

            Net* net = new Net();
            net->idx = static_cast<int>(dst.Nets.size());
            // 如果你的 Net 还有 name 字段，可以在 objects.h 里加，
            // 然后在这里写： net->name = dnet.name;

            dst.Nets.push_back(net);

            for (int cellIdx : dnet.pins) {
                if (cellIdx < 0 || static_cast<size_t>(cellIdx) >= index2Module.size())
                    continue;
                Module* mod = index2Module[cellIdx];
                if (!mod) continue;

                Pin* p = new Pin();
                p->idx    = static_cast<int>(dst.Pins.size());
                p->module = mod;
                p->net    = net;

                // 暂时让 pin 坐标等于模块中心（或者用 offset 全 0）
                p->offset.x = 0.0f;
                p->offset.y = 0.0f;

                mod->modulePins.push_back(p);
                net->netPins.push_back(p);
                dst.Pins.push_back(p);
            }
        }

        dst.netCount = static_cast<int>(dst.Nets.size());
        dst.pinCount = static_cast<int>(dst.Pins.size());

        // -------- 3) SiteRows / bins / siteHeight 这边目前没有信息，就保持默认 --------
    }

    // ============================================================
    //  PlaceData -> PlacementDB
    // ============================================================
    void buildPlacementDBFromPlaceData(const PlaceData& src, PlacementDB& dst) {
        // 清空原有内容
        dst.cells.clear();
        dst.nets.clear();
        dst.name_to_idx.clear();

        // -------- 1) Modules -> cells --------
        // 建一个 Module* -> cell index 的映射，后面建 net 时用
        std::map<const Module*, int> mod2idx;

        auto add_module_as_cell = [&dst, &mod2idx](const Module* m) {
            if (!m) return;

            Cell cell;
            cell.name   = m->name;
            cell.x      = static_cast<double>(m->center.x);
            cell.y      = static_cast<double>(m->center.y);
            cell.width  = static_cast<double>(m->width);
            cell.height = static_cast<double>(m->height);
            cell.fixed  = m->isFixed;

            int id = static_cast<int>(dst.cells.size());
            dst.cells.push_back(cell);
            dst.name_to_idx[cell.name] = id;
            mod2idx[m] = id;
        };

        // 先普通节点
        for (const Module* m : src.Nodes) {
            add_module_as_cell(m);
        }

        // 再 Terminals（一般是 IO/FIXED），为了画图也一起放进 cells
        for (const Module* t : src.Terminals) {
            add_module_as_cell(t);
        }

        // -------- 2) Nets -> DbNet --------
        dst.nets.reserve(src.Nets.size());
        for (const Net* net : src.Nets) {
            if (!net) continue;

            DbNet dnet;
            // 如果 Net 有名字字段，可在这里拷贝；否则简单生成一个
            dnet.name = "net_" + std::to_string(net->idx);

            for (const Pin* p : net->netPins) {
                if (!p || !p->module) continue;
                auto it = mod2idx.find(p->module);
                if (it == mod2idx.end()) continue;
                dnet.pins.push_back(it->second);
            }

            if (!dnet.pins.empty()) {
                dst.nets.push_back(std::move(dnet));
            }
        }

        // -------- 3) 用 chipRegion 或 cells 估算 die_* --------
        if (src.chipRegion.ll.x < src.chipRegion.ur.x &&
            src.chipRegion.ll.y < src.chipRegion.ur.y) {
            // 有合法 chipRegion，就直接用
            dst.die_xl = static_cast<double>(src.chipRegion.ll.x);
            dst.die_yl = static_cast<double>(src.chipRegion.ll.y);
            dst.die_xh = static_cast<double>(src.chipRegion.ur.x);
            dst.die_yh = static_cast<double>(src.chipRegion.ur.y);
        } else if (!dst.cells.empty()) {
            // 否则根据所有 cell 的包围盒来估算
            double minx = std::numeric_limits<double>::infinity();
            double miny = std::numeric_limits<double>::infinity();
            double maxx = -std::numeric_limits<double>::infinity();
            double maxy = -std::numeric_limits<double>::infinity();

            for (const auto& c : dst.cells) {
                minx = std::min(minx, c.x);
                miny = std::min(miny, c.y);
                maxx = std::max(maxx, c.x + c.width);
                maxy = std::max(maxy, c.y + c.height);
            }

            dst.die_xl = minx;
            dst.die_yl = miny;
            dst.die_xh = maxx;
            dst.die_yh = maxy;
        } else {
            // 没有 cells，就保持默认 0
            dst.die_xl = dst.die_yl = dst.die_xh = dst.die_yh = 0.0;
        }
    }

} // namespace DefPlaceAdapter
