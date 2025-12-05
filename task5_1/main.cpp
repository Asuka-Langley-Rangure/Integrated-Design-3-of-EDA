#include "defParser.h"
#include "algorithm/db.h"
#include "algorithm/nesterov.h"

#include "parse_bookshelf.h"

#include "DefPlaceAdapter.h"
#include "plot.h"

#include "eigen3/Eigen/Dense"
#include <bits/stdc++.h>
#include <vector>
using namespace std;

// 工具函数：判断字符串是否为坐标
static inline bool looksCoord(const std::string& s) {
    if (s.empty()) return false;
    for (unsigned char c : s)
        if (!std::isdigit(c) && c!='-' && c!='+' && c!='*')
            return false;
    return true;
}

void analyzeDesign(const DefDesign& dsg, const std::string& path) {
    cout << "==== DEF Summary ====\n";
    cout << "File        : " << path << "\n";
    cout << "Design Name : " << dsg.name << "\n";
    cout << "Version     : " << dsg.version.version << "\n";
    cout << "DBU/um      : " << dsg.units.dbu_per_micron << "\n";
    cout << "DividerChar : " << dsg.misc.dividerChar << "\n";
    cout << "BusBitChars : " << dsg.misc.busBitChars << "\n";
    cout << "DieArea     : (" << dsg.die_area.x1 << "," << dsg.die_area.y1
         << ") - (" << dsg.die_area.x2 << "," << dsg.die_area.y2 << ")\n\n";

    // ===== COMPONENTS =====
    size_t compN = dsg.components.size();
    size_t placedN=0, fixedN=0, unplacedN=0;
    long long minx=LLONG_MAX, miny=LLONG_MAX, maxx=LLONG_MIN, maxy=LLONG_MIN;
    for (const auto& c : dsg.components) {
        if (c.placed) placedN++; else unplacedN++;
        if (c.fixed) fixedN++;
        if (c.placed) {
            minx = min<long long>(minx, c.x);
            miny = min<long long>(miny, c.y);
            maxx = max<long long>(maxx, c.x);
            maxy = max<long long>(maxy, c.y);
        }
    }
    cout << "Components  : " << compN
         << " (placed=" << placedN
         << ", fixed=" << fixedN
         << ", unplaced=" << unplacedN << ")\n";
    if (placedN)
        cout << "  CoordRange X:[" << minx << ", " << maxx
             << "] Y:[" << miny << ", " << maxy << "]\n";
    if (!dsg.components.empty()) {
        cout << "  Sample Components:\n";
        for (size_t i=0;i<min<size_t>(3,compN);++i) {
            const auto& c=dsg.components[i];
            cout << "    - " << c.name << " (" << c.macro << ") ";
            if (c.placed) cout << "+PLACED("<<c.x<<","<<c.y<<") ";
            if (c.fixed) cout << "+FIXED ";
            cout << c.orient << "\n";
        }
    }
    cout << "\n";

    // ===== PINS =====
    cout << "Pins        : " << dsg.pins.size() << "\n";
    if (!dsg.pins.empty()) {
        cout << "  Sample Pins:\n";
        for (size_t i=0;i<min<size_t>(3,dsg.pins.size());++i){
            const auto& p=dsg.pins[i];
            cout << "    - " << p.name
                 << " NET=" << p.net
                 << " DIR=" << p.direction
                 << " @("<<p.x<<","<<p.y<<") "<<p.orient<<"\n";
        }
    }
    cout << "\n";

    // ===== NETS =====
    size_t netN=dsg.nets.size(), connTotal=0, emptyNets=0;
    size_t maxFanout=0; string maxNet;
    for (const auto& net:dsg.nets){
        size_t fanout=0;
        for (const auto& c:net.connections){
            if (looksCoord(c.instance)||looksCoord(c.pin)) continue;
            fanout++;
        }
        connTotal+=fanout;
        if (fanout==0) emptyNets++;
        if (fanout>maxFanout){maxFanout=fanout;maxNet=net.name;}
    }
    double avgFanout=netN?double(connTotal)/netN:0.0;
    cout << "Nets        : " << netN << "\n";
    cout << "  Total (inst-pin) pairs : " << connTotal << "\n";
    cout << "  Avg fanout             : " << fixed << setprecision(3)<<avgFanout<<"\n";
    cout << "  Empty nets             : " << emptyNets << "\n";
    cout << "  Max fanout net         : '" << maxNet << "' = " << maxFanout << "\n";
    if (!dsg.nets.empty()){
        cout << "  Sample Nets:\n";
        for (size_t i=0;i<min<size_t>(3,dsg.nets.size());++i){
            const auto& e=dsg.nets[i];
            cout << "    - " << e.name << " : ";
            size_t shown=0;
            for (const auto& p:e.connections){
                if (looksCoord(p.instance)||looksCoord(p.pin)) continue;
                cout << "("<<p.instance<<" "<<p.pin<<") ";
                if(++shown>=8) break;
            }
            cout << "\n";
        }
    }
    cout << "\n";

    // ===== SPECIALNETS =====
    cout << "SpecialNets : " << dsg.specialnets.size() << "\n";
    size_t spPairs=0;
    for (const auto& sn:dsg.specialnets)
        spPairs += sn.connections.size();
    cout << "  Total connections      : " << spPairs << "\n";
    if (!dsg.specialnets.empty()){
        cout << "  Sample SpecialNets:\n";
        for (size_t i=0;i<min<size_t>(3,dsg.specialnets.size());++i){
            const auto& s=dsg.specialnets[i];
            cout << "    - " << s.name << " : ";
            size_t shown=0;
            for (const auto& p:s.connections){
                cout << "("<<p.instance<<" "<<p.pin<<") ";
                if(++shown>=6) break;
            }
            cout << "\n";
        }
    }
    cout << "\n";

    // ===== ROWS / TRACKS / GCELLGRID =====
    cout << "Rows        : " << dsg.rows.size() << "\n";
    cout << "Tracks      : " << dsg.tracks.size() << "\n";
    cout << "GCellGrids  : " << dsg.gcells.size() << "\n";

    cout << "\n==== Summary Done ====\n";
}

// 从 DefDesign 构建 PlacementDB（示例）
PlacementDB buildDB(const DefDesign& dsg) {
    PlacementDB db;
    db.die_xl = dsg.die_area.x1;
    db.die_yl = dsg.die_area.y1;
    db.die_xh = dsg.die_area.x2;
    db.die_yh = dsg.die_area.y2;

    std::cerr << "[DEBUG] DIEAREA: ("
              << db.die_xl << "," << db.die_yl << ") - ("
              << db.die_xh << "," << db.die_yh << ")"<< std::endl;;

    db.cells.reserve(dsg.components.size());
    std::unordered_map<std::string,int> idx;

    size_t fixed_cnt = 0;
    for (size_t i=0;i<dsg.components.size();++i) {
        const auto& c = dsg.components[i];
        db.cells.emplace_back(c.name, c.x, c.y, 1.0, 1.0, c.fixed);
        idx[c.name] = int(i);
        if (c.fixed) fixed_cnt++;
        if (i < 5) {
            std::cerr << "[DEBUG] Cell " << i << ": "
                      << c.name << " @(" << c.x << "," << c.y << ") "
                      << (c.fixed ? "fixed" : "movable") << std::endl;
        }
    }
    std::cerr << "[DEBUG] Total Cells: " << db.cells.size()
              << " (fixed=" << fixed_cnt << ")"<< std::endl;

    db.nets.reserve(dsg.nets.size());
    size_t total_conn = 0;
    for (const auto& n : dsg.nets) {
        DbNet net; net.name = n.name;
        for (const auto& cp : n.connections) {
            auto it = idx.find(cp.instance);
            if (it != idx.end()){
                net.pins.push_back(it->second);
                total_conn++;
            } else {
                std::cerr << "[WARN] net " << n.name
                          << " has unknown instance " << cp.instance << std::endl;
            }
        }
        if (net.pins.size() > 1) db.nets.push_back(std::move(net));
        //if (!net.pins.empty()) db.nets.push_back(std::move(net));
    }

    std::cerr << "[DEBUG] Nets imported: " << db.nets.size()
              << ", total pins: " << total_conn << std::endl;

    return db;
}

// --- 随机初始化 ---
void randomInitialize(PlacementDB& db) {
    std::mt19937 rng(12345);  // 固定种子，结果可复现
    // std::uniform_real_distribution<double> dx(db.die_xl, db.die_xh);
    // std::uniform_real_distribution<double> dy(db.die_yl, db.die_yh);
    std::uniform_real_distribution<double> dx(db.die_xl, db.die_xh * 0.2);
    std::uniform_real_distribution<double> dy(db.die_yl, db.die_yh * 0.2);


    for (auto& c : db.cells) {
        if (!c.fixed) {
            c.x = dx(rng);
            c.y = dy(rng);
        }
    }
    std::cerr << "[INIT] Random placement initialized.\n";
}

void randomInitialize(PlaceData& db) {
    // 固定随机种子，保证可复现
    std::mt19937 rng(12345);

    // 从芯片区域拿边界
    const double xl = db.chipRegion.ll.x;
    const double yl = db.chipRegion.ll.y;
    const double xh = db.chipRegion.ur.x;
    const double yh = db.chipRegion.ur.y;

    const double w = xh - xl;
    const double h = yh - yl;

    // 和你原来 PlacementDB 版本一样：只在左下角 20% 区域里随机
    std::uniform_real_distribution<double> dx(xl, xl + 0.2 * w);
    std::uniform_real_distribution<double> dy(yl, yl + 0.2 * h);

    // 只动可移动的模块：Nodes 里非 fixed 的 Module
    for (auto* m : db.Nodes) {
        if (!m) continue;
        if (m->isFixed) continue;   // 保留 macro / IO 这些固定单元

        m->center.x = dx(rng);
        m->center.y = dy(rng);
    }

    std::cerr << "[INIT] Random placement initialized on PlaceData.\n";
}

// --- 主求解循环 + 日志 ---
void runPlacement(PlacementDB& db, const std::string& log_csv = "iteration_log.csv") {
    using namespace std;

    WirelengthModel wl(&db, /*gamma=*/200.0);
    DensityModel dens(&db, /*bins_r=*/64, /*bins_c=*/64);
    NesterovSolver solver(&db, wl, dens);

    Eigen::VectorXd x = db.getX();
    Eigen::VectorXd y = db.getY();

    ofstream log(log_csv);
    log << "iter,HPWL,LSE,Overflow,lambda\n";

    cout << "\n==== Global Placement Start ====\n";

    double hpwl_init = wl.computeHPWL(x, y);
    double wlse_init = wl.computeCost(x, y);
    double ovf_init  = dens.computeOverflow(x, y);
    cout << "[Init] HPWL=" << hpwl_init
         << "  W(LSE)=" << wlse_init
         << "  Overflow=" << ovf_init << "\n";

    double last_hpwl = hpwl_init;
    double rel_change = 1.0;
    int max_iters = 300;

    for (int it = 1; it <= max_iters; ++it) {
        solver.step(x, y); // Nesterov单步更新

        double hpwl = wl.computeHPWL(x, y);
        double wlse = wl.computeCost(x, y);
        double ovf  = dens.computeOverflow(x, y);
        double lam  = solver.lambda();

        log << it << "," << hpwl << "," << wlse << "," << ovf << "," << lam << "\n";

        // 仅每 10 轮打印一次，第一轮单独打印
        if (it % 10 == 0 || it == 1) {
            cout << "[Iter " << setw(3) << it << "] "
                 << "HPWL=" << hpwl
                 << "  W(LSE)=" << wlse
                 << "  Overflow=" << ovf
                 << "  lambda=" << lam << endl;
        }

        // === 收敛判断 ===
        rel_change = fabs(hpwl - last_hpwl) / std::max(1.0, last_hpwl);
        last_hpwl = hpwl;

        // 至少迭代 50 轮后才允许收敛
        if (it > 50 && rel_change < 1e-5 && ovf < 1e-3) {
            cout << "Converged at iteration " << it << endl;
            break;
        }

        // 防止过度发散
        if (!std::isfinite(hpwl) || !std::isfinite(wlse)) {
            cerr << "[ERROR] Encountered NaN/Inf during placement! Abort.\n";
            break;
        }
    }

    db.setXY(x, y);
    cout << "Global placement finished.\n";

    log.close();
}

// ====================== 辅助函数：PlaceData -> PlacementDB ======================
static void buildPlacementDBFromPlaceData(const PlaceData& src, PlacementDB& dst)
{
    dst.cells.clear();
    dst.nets.clear();
    dst.name_to_idx.clear();

    // 芯片区域
    dst.die_xl = src.chipRegion.ll.x;
    dst.die_yl = src.chipRegion.ll.y;
    dst.die_xh = src.chipRegion.ur.x;
    dst.die_yh = src.chipRegion.ur.y;

    // 1) 把每个 Module 映射成一个 Cell
    for (auto* m : src.Nodes) {
        if (!m) continue;
        // 这里用 module 的名字做 key，坐标取 center
        dst.addCell(m->name,
                    m->center.x,
                    m->center.y,
                    m->isFixed);
    }

    // 2) 把 Net 映射成 DbNet（只关心模块连线关系）
    for (auto* net : src.Nets) {
        if (!net) continue;

        std::vector<std::string> conn_names;
        conn_names.reserve(net->netPins.size());

        for (auto* p : net->netPins) {
            if (!p || !p->module) continue;
            conn_names.push_back(p->module->name);
        }

        // 利用 PlacementDB 自带的 addDbNet 通过名字查 cell 索引
        if (!conn_names.empty()) {
            dst.addDbNet(net->name, conn_names);
        }
    }
}

// ====================== 辅助函数：PlacementDB -> PlaceData（只同步坐标） ======================
static void syncXYToPlaceData(const PlacementDB& src, PlaceData& dst)
{
    // 利用 PlaceData 自带的 moduleMap 快速按名字找 Module*
    for (const auto& cell : src.cells) {
        auto it = dst.moduleMap.find(cell.name);
        if (it == dst.moduleMap.end()) continue;

        Module* m = it->second;
        if (!m) continue;

        m->center.x = static_cast<float>(cell.x);
        m->center.y = static_cast<float>(cell.y);
    }
}

// ====================== 主求解循环（在 PlaceData 上调用） ======================
void runPlacement(PlaceData& placeDB, const std::string& log_csv = "iteration_log.csv")
{
    using namespace std;

    // 1) 从 PlaceData 构造一个“数值用”的 PlacementDB
    PlacementDB db;
    buildPlacementDBFromPlaceData(placeDB, db);

    // 2) 在这个 db 上构造模型和求解器
    WirelengthModel wl(&db, /*gamma=*/200.0);
    DensityModel    dens(&db, /*bins_r=*/64, /*bins_c=*/64);
    NesterovSolver  solver(&db, wl, dens);

    Eigen::VectorXd x = db.getX();
    Eigen::VectorXd y = db.getY();

    ofstream log(log_csv);
    log << "iter,HPWL,LSE,Overflow,lambda\n";

    cout << "\n==== Global Placement Start ====\n";

    double hpwl_init = wl.computeHPWL(x, y);
    double wlse_init = wl.computeCost(x, y);
    double ovf_init  = dens.computeOverflow(x, y);
    cout << "[Init] HPWL=" << hpwl_init
         << "  W(LSE)=" << wlse_init
         << "  Overflow=" << ovf_init << "\n";

    double last_hpwl   = hpwl_init;
    double rel_change  = 1.0;
    const int max_iters = 300;

    for (int it = 1; it <= max_iters; ++it) {
        // Nesterov 单步更新
        solver.step(x, y);

        double hpwl = wl.computeHPWL(x, y);
        double wlse = wl.computeCost(x, y);
        double ovf  = dens.computeOverflow(x, y);
        double lam  = solver.lambda();

        log << it << "," << hpwl << "," << wlse << "," << ovf << "," << lam << "\n";

        // 每 10 轮打印一次，第一轮单独打印
        if (it % 10 == 0 || it == 1) {
            cout << "[Iter " << setw(3) << it << "] "
                 << "HPWL=" << hpwl
                 << "  W(LSE)=" << wlse
                 << "  Overflow=" << ovf
                 << "  lambda=" << lam << endl;
        }

        // === 收敛判断 ===
        rel_change = fabs(hpwl - last_hpwl) / std::max(1.0, last_hpwl);
        last_hpwl  = hpwl;

        // 至少迭代 50 轮后才允许收敛
        if (it > 50 && rel_change < 1e-5 && ovf < 1e-3) {
            cout << "Converged at iteration " << it << endl;
            break;
        }

        // 防止过度发散
        if (!std::isfinite(hpwl) || !std::isfinite(wlse)) {
            cerr << "[ERROR] Encountered NaN/Inf during placement! Abort.\n";
            break;
        }
    }

    // 3) 把优化后的坐标写回 PlacementDB
    db.setXY(x, y);

    cout << "Global placement finished.\n";
    log.close();

    // 4) 再把坐标同步回 PlaceData（只更新 Module::center）
    syncXYToPlaceData(db, placeDB);
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    string path = "./data/example.def";
    auto t0 = chrono::high_resolution_clock::now();

    DefParser parser(path);
    bool ok = parser.parse();
    auto t1 = chrono::high_resolution_clock::now();

    if (!ok) {
        cerr << "❌ Parse failed for: " << path << "\n";
        return 1;
    }

    auto ms = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    cerr << "[INFO] Parse done in " << ms << " ms\n";

    const auto& design = parser.getDesign();
    //analyzeDesign(design, path);
    PlacementDB db = buildDB(design);

    randomInitialize(db);     // ✅ 随机初始位置
    runPlacement(db);         // ✅ 执行主求解并输出日志

    // 转换
    PlaceData* placeData = new PlaceData();
    DefPlaceAdapter::buildPlaceDataFromPlacementDB(db, *placeData);

    // 调试
    PrintPlaceData(placeData);

    // 可视化
    PLOTTING::plotPlacementSimple("placement_result_3", *placeData);

    delete placeData;

    return 0;
}
