#include "parse_bookshelf.h"
#include <iomanip>
#include <limits>

using std::tuple;
using std::map;
using std::string;
using std::vector;
using std::istringstream;
using std::ifstream;
using std::regex;
using std::smatch;

tuple<map<string,string>, string> parse_top(const string &filename) {
    map<string,string> top_info;
    string folder_path = filesystem::path(filename).parent_path().string();

    ifstream f(filename);
    string line;
    if (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);
        for (size_t i=2; i<parts.size(); i++) {
            smatch m;
            regex re("(.*)\\.(\\w+)$");
            if (regex_match(parts[i], m, re)) {
                string suffix = m[2];
                top_info[suffix] = parts[i];
            }
        }
    }
    return {top_info, folder_path};
}

void parse_nodes(const string &filename, PlaceData* db) {
    ifstream f(filename);
    if (!f) {
        std::cerr << "Cannot open nodes file: " << filename << std::endl;
        return;
    }
    if (!db) return;

    string line;
    while (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;
        if (parts[0].empty()) continue;
        if (parts[0][0]=='#') continue;

        if (parts[0] == "NumNodes") db->moduleCount = stoi(parts[2]);
        else if (parts[0] == "NumTerminals") db->terminalCount = stoi(parts[2]);
        else if (parts[0][0] == 'o') {
            Module* module = new Module();
            module->Init();
            db->Nodes.push_back(module);
            module->idx = stoi(parts[0].substr(1));
            module->name = parts[0];
            db->moduleMap[module->name] = module;
            if (parts.size() == 4 && parts[3] == "terminal"){
                module->isTerminal = true;
            }
            module->width  = stod(parts[1]);
            module->height = stod(parts[2]);
            module->area   = module->width * module->height;
        }
    }
}

void parse_scl(const string &filename, PlaceData* db) {
    ifstream f(filename);
    if (!f) {
        std::cerr << "Cannot open scl file: " << filename << std::endl;
        return;
    }
    if (!db) return;

    SiteRow currentSiteRow;
    string line;
    while (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;
        if (parts[0].empty()) continue;
        if (parts[0][0]=='#') continue;

        if (parts[0] == "Coordinate") {
            currentSiteRow.bottom = stod(parts[2]);
        }
        else if (parts[0] == "Height") {
            currentSiteRow.height = stod(parts[2]);
        }
        else if (parts[0] == "Sitewidth") {
            currentSiteRow.step = stod(parts[2]); // or Sitespacing
        }
        else if (parts[0] == "SubrowOrigin") {
            double origin_x = stod(parts[2]);
            int num_sites = stoi(parts[5]);
            currentSiteRow.start.x = origin_x;
            currentSiteRow.start.y = currentSiteRow.bottom;
            currentSiteRow.end.x   = origin_x + num_sites * currentSiteRow.step;
            currentSiteRow.end.y   = currentSiteRow.bottom;
        }
        else if (parts[0] == "End") {
            db->SiteRows.push_back(currentSiteRow);
        }
    }
}

void parse_nets(const std::string& filename, PlaceData* db)
{
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "Cannot open nets file: " << filename << std::endl;
        return;
    }
    if (!db) return;

    std::string line;
    Net* currentNet = nullptr;
    while (std::getline(fin, line)) {

        if (line.empty()) continue;
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;
        if (parts[0].empty()) continue;
        if (parts[0][0]=='#') continue;

        if (parts[0] == "NumNets"){
            db->netCount = stoi(parts[2]);
            continue;
        }
        if (parts[0] == "NumPins") {
            db->pinCount = stoi(parts[2]);
            continue;
        }
        // New net
        if (parts[0] == "NetDegree") {
            currentNet = new Net();
            currentNet->init();
            currentNet->name = parts[3];
            currentNet->idx = stoi(parts[3].substr(1));
            int deg = stoi(parts[2]);
            for(int i = 0; i < deg; ++i){
                std::getline(fin, line);
                istringstream iss2(line);
                vector<string> pin_parts;
                while (iss2 >> token) pin_parts.push_back(token);
                Pin* pin = new Pin();
                pin->init();
                pin->idx = i;
                auto it = db->moduleMap.find(pin_parts[0]);
                pin->module = (it == db->moduleMap.end() ? nullptr : it->second);
                if (pin->module) {
                    pin->module->modulePins.push_back(pin);
                }
                pin->net = currentNet;
                pin->direction = pin_parts[1];
                pin->offset.x = stod(pin_parts[3]);
                pin->offset.y = stod(pin_parts[4]);
                currentNet->netPins.push_back(pin);
            }
            db->Nets.push_back(currentNet);
            continue;
        }
    }
}

void parse_pl(const string &filename, PlaceData* db) {
    ifstream f(filename);
    if (!f) {
        std::cerr << "Cannot open pl file: " << filename << std::endl;
        return;
    }
    if (!db) return;

    string line;
    while (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;
        if (parts[0].empty()) continue;
        if (parts[0][0]=='#') continue;

        if (parts[0][0]=='o'){
            auto it = db->moduleMap.find(parts[0]);
            if (it == db->moduleMap.end()) continue;
            Module* module = it->second;

            double llx = stod(parts[1]);
            double lly = stod(parts[2]);

            module->coor.x = float(llx);
            module->coor.y = float(lly);

            // 你的 Module 使用 center 存位置信息，这里将 LL 转为 center
            module->center.x = float(llx + module->width  / 2.0);
            module->center.y = float(lly + module->height / 2.0);

            if (parts.size() >= 5) {
                // 仅支持 N/S/E/W
                char o = parts[4][0];
                if (o == 'N') module->orientation = 0;
                else if (o == 'E') module->orientation = 90;
                else if (o == 'S') module->orientation = 180;
                else if (o == 'W') module->orientation = 270;
                else module->orientation = 0;
            }
            if (parts.size() >= 6 && parts[5] == "/FIXED") {
                module->isFixed = true;
                module->isMacro = true; // 这里假设固定模块都是宏单元
            }
        }
    }
}

void PrintDatabaseSummary(const PlaceData* db)
{
    if (!db) {
        std::cerr << "Error: PlaceData pointer is null.\n";
        return;
    }

    long long num_modules   = (long long)db->moduleCount;
    long long num_nodes     = (long long)db->moduleCount - db->terminalCount;
    long long num_terminals = (long long)db->terminalCount;
    long long num_nets      = (long long)db->netCount;
    long long num_pins      = (long long)db->pinCount;

    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = std::numeric_limits<double>::lowest();
    double rowsize = 0;
    int rownum = 0;
    for (auto &r : db->SiteRows) {
        xmin = std::min(xmin, static_cast<double>(r.start.x));
        ymin = std::min(ymin, static_cast<double>(r.bottom));
        xmax = std::max(xmax, static_cast<double>(r.end.x));
        ymax = std::max(ymax, static_cast<double>(r.bottom + r.height));
    }
    if (!db->SiteRows.empty()) {
        rowsize = db->SiteRows.front().height;
        rownum  = (int)db->SiteRows.size();
    }

    long long cellArea = 0, fixedArea = 0, fixedInCore = 0;
    for (auto* m : db->Nodes) {
        if (!m) continue;
        if (!m->isTerminal){
            cellArea += (long long)m->area;
        }
        if (m->isFixed) {
            fixedArea += (long long)m->area;
            if (m->center.x >= xmin && m->center.x <= xmax &&
                m->center.y >= ymin && m->center.y <= ymax) {
                fixedInCore += (long long)m->area;
            }
        }
    }

    long long movableArea = cellArea;
    long long coreArea = (long long)((xmax - xmin) * (ymax - ymin));
    long long freeSitesArea = coreArea - fixedInCore;
    double placementUtil = (freeSitesArea > 0) ? (double)movableArea / freeSitesArea : 0.0;
    long long usedArea = movableArea + fixedInCore;
    double coreDensity = (coreArea > 0) ? (double)usedArea / coreArea : 0.0;

    long long max_degree = 0;
    long long pin_bin_1 = 0, pin_bin_2 = 0, pin_bin_3_10 = 0, pin_bin_11_100 = 0, pin_bin_100 = 0;
    for (auto* net : db->Nets) {
        if (!net) continue;
        int deg = (int)net->netPins.size();
        if (deg > max_degree) max_degree = deg;
        if (deg == 1) pin_bin_1++;
        else if (deg == 2) pin_bin_2++;
        else if (deg <= 10) pin_bin_3_10++;
        else if (deg <= 100) pin_bin_11_100++;
        else pin_bin_100++;
    }

    auto kfmt = [](long long v) {
        return string("(= ") + std::to_string(v / 1000) + "k)";
    };

    using std::cout;
    using std::endl;
    using std::setw;
    using std::setprecision;

    cout << "Use BOOKSHELF placement format\n";
    cout << "Set core region from site info: lower left: (" << (long long)xmin << "," << (long long)ymin
         << ") to upper right: (" << (long long)xmax << "," << (long long)ymax << ")\n";
    cout << "    NumModules:" << num_modules << "\n";
    cout << "    NumNodes: " << num_nodes << " " << kfmt(num_nodes) << "\n";
    cout << "    Terminals = " << num_terminals << "\n";
    cout << "        Nets = " << num_nets << "\n";
    cout << "        Pins = " << num_pins << "\n";
    cout << "Max net degree= " << max_degree << "\n";

    cout << "<<<< DATABASE SUMMARIES >>>>\n";
    cout << "        Core region: lower left: (" << (long long)xmin << "," << (long long)ymin
         << ") to upper right: (" << (long long)xmax << "," << (long long)ymax << ")\n";
    cout << "    Row Height/Number:" << (long long)rowsize << "/" << rownum << " (site step 1)\n";

    cout.setf(std::ios::fixed);
    cout << setprecision(6);
    cout << "        Core Area: " << coreArea << " (" << coreArea << ")\n";
    cout << "        Cell Area: " << cellArea << " (" << (double)cellArea/coreArea*100 << "%)\n";
    cout << "        Movable Area: " << movableArea << " (" << (double)movableArea/coreArea*100 << "%)\n";
    cout << "        Fixed Area:  " << fixedArea << " (" << (double)fixedArea/coreArea*100 << "%)\n";
    cout << "        Fixed Area in Core: " << fixedInCore << " (" << (double)fixedInCore/coreArea*100 << "%)\n";
    cout << "        Placement Util.: " << placementUtil*100 << "% (=move/freeSites)\n";
    cout << "        Core Density: " << coreDensity*100 << "% (=usedArea/core)\n";

    const string s23(23, ' ');
    const string s21(21, ' ');
    const string s24(24, ' ');
    const string s14(14, ' ');

    long long cell_cnt = num_nodes - num_terminals;

    cout << s23 << "Cell #: " << cell_cnt << " " << kfmt(cell_cnt) << "\n";
    cout << s21 << "Object #: " << num_nodes << " " << kfmt(num_nodes)
         << " (fixed: " << num_terminals << ") (macro: 0)\n";
    cout << s24 << "Net #: " << num_nets << " " << kfmt(num_nets) << "\n";
    cout << s14 << "Max net degree=: " << max_degree << "\n";
    cout << s24 << "Pin 1 (" << pin_bin_1 << ") 2 (" << pin_bin_2
         << ") 3-10 (" << pin_bin_3_10 << ") 11-100 (" << pin_bin_11_100
         << ") 100- (" << pin_bin_100 << ")\n";
    cout << s24 << "Pin #: " << num_pins << "\n";
}

void PrintPlaceData(const PlaceData* db)
{
    if (!db) {
        std::cerr << "PlaceData pointer is null." << std::endl;
        return;
    }

    using std::cout;
    using std::endl;
    using std::setw;

    cout << "================== [ PlaceData Summary ] ==================\n";
    cout << "Modules   : " << db->moduleCount   << endl;
    cout << "Terminals : " << db->terminalCount << endl;
    cout << "Macros    : " << db->macroCount    << endl;
    cout << "Nets      : " << db->netCount      << endl;
    cout << "Pins      : " << db->pinCount      << endl;
    cout << "-----------------------------------------------------------\n\n";

    cout << "=== Modules (" << db->Nodes.size() << ") ===\n";
    for (auto* m : db->Nodes) {
        if (!m) continue;
        cout << "  [" << setw(3) << m->idx << "] "
             << std::left << setw(8) << m->name
             << "  W=" << setw(6) << m->width
             << "  H=" << setw(6) << m->height
             << "  Area=" << setw(8) << m->area
             << "  Center=(" << m->center.x << "," << m->center.y << ")"
             << "  Ori=" << setw(2) << m->orientation
             << "  " << (m->isTerminal ? "[T]" : "")
             << "  " << (m->isFixed ? "[Fixed]" : "")
             << endl;
    }
    cout << endl;

    cout << "=== Nets (" << db->Nets.size() << ") ===\n";
    for (auto* n : db->Nets) {
        if (!n) continue;
        cout << "Net " << n->name << " (" << n->netPins.size() << " pins)\n";
        for (auto* p : n->netPins) {
            if (!p || !p->module) continue;
            cout << "    Pin[" << p->idx << "] "
                 << std::left << setw(8) << p->module->name
                 << " Dir=" << setw(2) << p->direction
                 << " Offset=(" << p->offset.x << "," << p->offset.y << ")\n";
        }
    }
    cout << endl;

    cout << "=== SiteRows (" << db->SiteRows.size() << ") ===\n";
    for (size_t i = 0; i < db->SiteRows.size(); ++i) {
        const auto& row = db->SiteRows[i];
        cout << "Row[" << i << "]  Bottom=" << row.bottom
             << "  Height=" << row.height
             << "  Step=" << row.step
             << "  Start=(" << row.start.x << "," << row.start.y << ")"
             << "  End=("   << row.end.x   << "," << row.end.y   << ")\n";
    }

    cout << "===========================================================\n";
}

bool ParseBookshelfDataset(const fs::path tmp_path, PlaceData* db) {
    if (!fs::exists(tmp_path)) {
        std::cerr << "❌ ERROR: .aux file not found at: " << tmp_path << std::endl;
        return false;
    }
    if (!db) return false;

    try{
        auto [top_info, folder_path] = parse_top(tmp_path.string());
        fs::path base_path = fs::path(folder_path);
        parse_scl((base_path / top_info["scl"]).string(),   db);
        parse_nodes((base_path / top_info["nodes"]).string(), db);
        parse_nets((base_path / top_info["nets"]).string(), db);
        parse_pl((base_path / top_info["pl"]).string(),     db);

        double xmin = std::numeric_limits<double>::max();
        double ymin = std::numeric_limits<double>::max();
        double xmax = std::numeric_limits<double>::lowest();
        double ymax = std::numeric_limits<double>::lowest();

        for (auto &r : db->SiteRows) {
            xmin = std::min(xmin, static_cast<double>(r.start.x));
            ymin = std::min(ymin, static_cast<double>(r.bottom));
            xmax = std::max(xmax, static_cast<double>(r.end.x));
            ymax = std::max(ymax, static_cast<double>(r.bottom + r.height));
        }
        
        db->chipRegion.ll = POS_2D(float(xmin), float(ymin));
        db->chipRegion.ur = POS_2D(float(xmax), float(ymax));
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing bookshelf dataset: " << e.what() << std::endl;
        return false;
    }
    return true;
}

static inline double rect_area(double w, double h){ return (w>0 && h>0) ? w*h : 0.0; }

// 从 SiteRow 推导核心区；若没有 SiteRow，则用模块包围盒兜底
static void infer_core_from_rows(const PlaceData* db,
                                 double& llx, double& lly,
                                 double& urx, double& ury,
                                 double& row_h_out, int& row_n_out, double& site_step_out)
{
    llx =  std::numeric_limits<double>::infinity();
    lly =  std::numeric_limits<double>::infinity();
    urx = -std::numeric_limits<double>::infinity();
    ury = -std::numeric_limits<double>::infinity();
    std::vector<double> row_h;
    std::vector<double> steps;

    for (const auto& r : db->SiteRows) {
        double x0 = r.start.x, y0 = r.start.y;
        double x1 = r.end.x,   y1 = r.end.y + r.height; // end 为行末右下角，+height 得到上边界
        llx = std::min(llx, std::min(x0, x1));
        lly = std::min(lly, std::min(y0, y1));
        urx = std::max(urx, std::max(x0, x1));
        ury = std::max(ury, std::max(y0, y1));
        if (r.height > 0) row_h.push_back(r.height);
        if (r.step   > 0) steps.push_back(r.step);
    }
    if (!std::isfinite(llx)) { // 无 SiteRow，用模块兜底
        llx =  std::numeric_limits<double>::infinity();
        lly =  std::numeric_limits<double>::infinity();
        urx = -std::numeric_limits<double>::infinity();
        ury = -std::numeric_limits<double>::infinity();
        for (const Module* m : db->Nodes) {
            if (!m) continue;
            POS_2D ll = m->getLL_2D();
            POS_2D ur = m->getUR_2D();
            llx = std::min( llx, static_cast<double>(ll.x) );
            lly = std::min( lly, static_cast<double>(ll.y) );
            urx = std::max( urx, static_cast<double>(ur.x) );
            ury = std::max( ury, static_cast<double>(ur.y) );

        }
        if (!std::isfinite(llx)) { llx=lly=0; urx=ury=1; }
    }
    auto median = [](std::vector<double>& v)->double{
        if (v.empty()) return 0.0;
        std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
        return v[v.size()/2];
    };
    row_h_out      = median(row_h);
    row_n_out      = static_cast<int>(db->SiteRows.size());
    site_step_out  = steps.empty() ? 1.0 : median(steps);
}

static void degree_buckets(const PlaceData* db,
                           long long& d2, long long& d3_10,
                           long long& d11_100, long long& d100p, int& dmax)
{
    d2=d3_10=d11_100=d100p=0; dmax=0;
    for (const Net* net : db->Nets) {
        if (!net) continue;
        int d = (int)net->netPins.size();
        dmax = std::max(dmax, d);
        if (d==2) ++d2;
        else if (d>=3 && d<=10) ++d3_10;
        else if (d>=11 && d<=100) ++d11_100;
        else if (d>100) ++d100p;
    }
}

static inline void print_percent(double num, double den){
    double p = (den>0.0 ? 100.0*num/den : 0.0);
    std::cout<<std::fixed<<std::setprecision(2)<<p<<"%";
}

// 主入口：输出与截图一致的概要
// binRows/binCols：你最终建 bin 的尺寸；binAddTimeSec：若没有可传负数
void printDesignSummary(const PlaceData* db, int binRows=512, int binCols=512, double binAddTimeSec=-1.0)
{
    using std::cout; using std::endl;

    if (!db) { std::cerr << "[printDesignSummary] db is null\n"; return; }

    // 1) Core 区域
    double llx,lly,urx,ury,rowH,siteStep; int rowN;
    infer_core_from_rows(db, llx,lly,urx,ury, rowH,rowN,siteStep);
    const double coreW = std::max(0.0, urx-llx);
    const double coreH = std::max(0.0, ury-lly);
    const double coreA = coreW * coreH;

    // 2) 模块统计（按你的 Module 字段）
    long long cellCnt=0, fixedCnt=0, macroCnt=0;
    double movableA=0.0, fixedA_in_core=0.0, totalUsedA=0.0;

    auto overlap_with_core = [&](const Module* m)->bool{
        POS_2D mll = m->getLL_2D();
        POS_2D mur = m->getUR_2D();
        double ix0 = std::max<double>(llx, mll.x);
        double iy0 = std::max<double>(lly, mll.y);
        double ix1 = std::min<double>(urx, mur.x);
        double iy1 = std::min<double>(ury, mur.y);
        return (ix1>ix0 && iy1>iy0);
    };

    for (const Module* m : db->Nodes) {
        if (!m) continue;
        const double a = rect_area(m->width, m->height);
        if (a<=0.0) continue;
        ++cellCnt;
        if (m->isMacro) ++macroCnt;
        if (m->isFixed || m->isTerminal) {
            ++fixedCnt;
            if (overlap_with_core(m)) fixedA_in_core += a;
        } else {
            movableA += a;
        }
        totalUsedA += a;
    }

    // 3) 网络与引脚
    const long long netCnt = (long long)db->Nets.size();
    // pin 总数：按 netPins 累加（PlaceData 里如果也有 Pins，可替换为 db->Pins.size()）
    long long pinCnt = 0;
    for (const Net* net : db->Nets) if (net) pinCnt += (long long)net->netPins.size();

    long long d2,d3_10,d11_100,d100p; int dmax;
    degree_buckets(db, d2,d3_10,d11_100,d100p, dmax);

    // 4) 利用率与密度
    const double freeSites = std::max(0.0, coreA - fixedA_in_core);
    const double placeUtil = (freeSites>0 ? movableA / freeSites : 0.0);
    const double coreDensity = (coreA>0 ? (movableA + fixedA_in_core) / coreA : 0.0);

    // 5) Bin 参数（若尚未建 bin，就按维度估算步长）
    const double stepX = (binCols>0 ? coreW / binCols : 0.0);
    const double stepY = (binRows>0 ? coreH / binRows : 0.0);

    // === 输出 ===
    cout << u8"●  Overview:" << endl;
    cout << "Core region: lower left: (" << (long long)llx << "," << (long long)lly
         << ") to upper right: (" << (long long)urx << "," << (long long)ury << ")" << endl;
    cout << "Row Height/Number: " << (long long)rowH << " / " << rowN
         << " (site step " << std::fixed << std::setprecision(6) << siteStep << ")" << endl;
    cout << "Core Area: " << (long long)coreA << " (" << std::scientific << coreA << std::defaultfloat << ")" << endl;

    cout << "Cell Area: " << (long long)totalUsedA << " (";
    print_percent(totalUsedA, coreA); cout << ")" << endl;

    cout << "Movable Area: " << (long long)movableA << " (";
    print_percent(movableA, coreA); cout << ")" << endl;

    cout << endl;
    cout << "Fixed Area: " << (long long)fixedA_in_core << " (";
    print_percent(fixedA_in_core, coreA); cout << ")" << endl;

    cout << "Fixed Area in Core: " << (long long)fixedA_in_core << " (";
    print_percent(fixedA_in_core, coreA); cout << ")" << endl;

    cout << "Placement Util.: " << std::fixed << std::setprecision(2)
         << (100.0*placeUtil) << "% (=move/freeSites)" << endl;

    cout << "Core Density: " << std::fixed << std::setprecision(2)
         << (100.0*coreDensity) << "% (=usedArea/core)" << endl;

    cout << "Cell #: " << cellCnt << " (=~cells)" << endl;

    cout << "Object #: " << cellCnt
         << " (=~cells) (fixed: " << fixedCnt << ") (macro: " << macroCnt << ")" << endl;

    cout << "Net #: " << netCnt << endl;
    cout << "Max net degree==: " << dmax << endl;

    cout << "Pin 2 (" << d2 << ") 3-10 (" << d3_10 << ") 11-100 (" << d11_100
         << ") 100- (" << d100p << ")" << endl;

    cout << "Pin #: " << pinCnt << endl;

    cout << endl;
    cout << u8"●  Bin Setting:" << endl;
    cout << "Bin dimension: [" << binRows << "," << binCols << "]" << endl;
    cout << "coreRegion width: "  << (long long)coreW << endl;
    cout << "coreRegion height: " << (long long)coreH << endl;
    cout << "Bin step: [" << std::fixed << std::setprecision(4) << stepX << "," << stepY << "]" << endl;
    if (binAddTimeSec >= 0.0)
        cout << "Bin add time: " << std::fixed << std::setprecision(6) << binAddTimeSec << endl;
}