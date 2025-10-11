#include "parse_bookshelf.h"

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

void parse_nodes(const string &filename, PlaceData& db) {
    ifstream f(filename);
    if (!f) {
        std::cerr << "Cannot open nodes file: " << filename << std::endl;
        return;
    }
    string line;
    while (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;

        if (parts[0][0]=='#') continue;

        if (parts[0] == "NumNodes") db.moduleCount = stoi(parts[2]);
        else if (parts[0] == "NumTerminals") db.terminalCount = stoi(parts[2]);
        else if (parts[0][0] == 'o') {
            Module* module = new Module();
            module->Init();
            db.Nodes.push_back(module);
            module->idx = stoi(parts[0].substr(1));
            module->name = parts[0];
            db.moduleMap[module->name] = module;
            if (parts.size() == 4){
                if (parts[3] == "terminal"){
                    module->isTerminal = true;
                }
            }
            module->width = stod(parts[1]);
            module->height = stod(parts[2]);
            module->area = module->width * module->height;
        }
    }
}

void parse_scl(const string &filename, PlaceData& db) {
    ifstream f(filename);
    if (!f) {
        std::cerr << "Cannot open scl file: " << filename << std::endl;
        return;
    }
    SiteRow currentSiteRow;
    string line;
    while (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;

        if (parts[0][0]=='#') continue;

        if (parts[0] == "Coordinate") {
            currentSiteRow.bottom = stod(parts[2]);
        }
        else if (parts[0] == "Height") {
            currentSiteRow.height = stod(parts[2]);
        }
        else if (parts[0] == "Sitewidth") {
            currentSiteRow.step = stod(parts[2]); // 或者取 Sitespacing
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
            db.SiteRows.push_back(currentSiteRow);
        }
    }
}

void parse_nets(const std::string& filename, PlaceData& db)
{
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "Cannot open nets file: " << filename << std::endl;
        return;
    }
    std::string line;
    Net* currentNet = nullptr;
    while (std::getline(fin, line)) {

        if (line.empty()) continue;
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;

        if (parts[0][0]=='#') continue;

        if (parts[0] == "NumNets"){
            db.netCount = stoi(parts[2]);
            continue;
        }
        if (parts[0] == "NumPins") {
            db.pinCount = stoi(parts[2]);
            continue;
        }
        // 新网络
        if (parts[0] == "NetDegree") {
            currentNet = new Net();
            currentNet->init();
            currentNet->name = parts[3];
            currentNet->idx = stoi(parts[3].substr(1));
            for(int i = 0; i < stoi(parts[2]); ++i){
                std::getline(fin, line);
                istringstream iss(line);
                vector<string> pin_parts;
                while (iss >> token) pin_parts.push_back(token);
                Pin* pin = new Pin();
                pin->init();
                pin->idx = i;
                pin->module = db.moduleMap[pin_parts[0]];
                pin->net = currentNet;
                pin->direction = pin_parts[1];
                pin->offset.x = stod(pin_parts[3]);
                pin->offset.y = stod(pin_parts[4]);
                currentNet->netPins.push_back(pin);
            }
            db.Nets.push_back(currentNet);
            continue;
        }
    }
}

void parse_pl(const string &filename, PlaceData& db) {
    ifstream f(filename);
    if (!f) {
        std::cerr << "Cannot open pl file: " << filename << std::endl;
        return;
    }
    string line;
    Module* module;
    while (getline(f, line)) {
        istringstream iss(line);
        vector<string> parts;
        string token;
        while (iss >> token) parts.push_back(token);

        if (parts.empty()) continue;

        if (parts[0][0]=='#') continue;

        if (parts[0][0]=='o'){
            module = db.moduleMap[parts[0]];
            module->center.x = float(stod(parts[1])+module->width/2);
            module->center.y = float(stod(parts[2])+module->height/2);
            module->orientation = parts[4];
            if (parts.size() == 6){
                if (parts[5] == "/FIXED"){
                    module->isFixed = true;
                }
            }
        }
    }
}

void PrintDatabaseSummary(const PlaceData* db)
{
    if (!db) {
        cerr << "Error: PlaceData pointer is null.\n";
        return;
    }

    // -------------------- 统计核心信息 --------------------
    long long num_modules   = (long long)db->moduleCount;
    long long num_nodes     = (long long)db->moduleCount-db->terminalCount;
    long long num_terminals = (long long)db->terminalCount;
    long long num_nets      = (long long)db->netCount;
    long long num_pins      = (long long)db->pinCount;

    // ----------- core region from site info -------------
    double xmin = numeric_limits<double>::max();
    double ymin = numeric_limits<double>::max();
    double xmax = numeric_limits<double>::lowest();
    double ymax = numeric_limits<double>::lowest();
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

    // ----------- 统计面积信息 -------------
    long long cellArea = 0, fixedArea = 0, fixedInCore = 0;
    for (auto* m : db->Nodes) {
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

    long long movableArea = cellArea; // 按样例设为相同
    long long coreArea = (long long)((xmax - xmin) * (ymax - ymin));
    long long freeSitesArea = coreArea - fixedInCore;
    double placementUtil = (freeSitesArea > 0) ? (double)movableArea / freeSitesArea : 0.0;
    long long usedArea = movableArea + fixedInCore;
    double coreDensity = (coreArea > 0) ? (double)usedArea / coreArea : 0.0;

    // ----------- 统计 net degree 分布 -------------
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
        return string("(= ") + to_string(v / 1000) + "k)";
    };

    // -------------------- 开始打印 --------------------
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

    cout.setf(ios::fixed);
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
        cerr << "PlaceData pointer is null." << endl;
        return;
    }

    cout << "================== [ PlaceData Summary ] ==================\n";
    cout << "Modules   : " << db->moduleCount   << endl;
    cout << "Terminals : " << db->terminalCount << endl;
    cout << "Macros    : " << db->macroCount    << endl;
    cout << "Nets      : " << db->netCount      << endl;
    cout << "Pins      : " << db->pinCount      << endl;
    cout << "-----------------------------------------------------------\n\n";

    // 打印模块信息
    cout << "=== Modules (" << db->Nodes.size() << ") ===\n";
    for (auto* m : db->Nodes) {
        if (!m) continue;
        cout << "  [" << setw(3) << m->idx << "] "
             << left << setw(8) << m->name
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

    // 打印网络信息
    cout << "=== Nets (" << db->Nets.size() << ") ===\n";
    for (auto* n : db->Nets) {
        if (!n) continue;
        cout << "Net " << n->name << " (" << n->netPins.size() << " pins)\n";
        for (auto* p : n->netPins) {
            if (!p || !p->module) continue;
            cout << "    Pin[" << p->idx << "] "
                 << left << setw(8) << p->module->name
                 << " Dir=" << setw(2) << p->direction
                 << " Offset=(" << p->offset.x << "," << p->offset.y << ")\n";
        }
    }
    cout << endl;

    // 打印 SiteRow 信息
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

bool ParseBookshelfDataset(const fs::path tmp_path, PlaceData& db) {
    if (!fs::exists(tmp_path)) {
        cerr << "❌ ERROR: .aux file not found at: " << tmp_path << endl;
        return false;
    }
    try{
        auto [top_info, folder_path] = parse_top(tmp_path.string());
        fs::path base_path = fs::path(folder_path);
        parse_scl((base_path / top_info["scl"]).string(), db);
        parse_nets((base_path / top_info["nets"]).string(), db);
        parse_nodes((base_path / top_info["nodes"]).string(), db);
        parse_pl((base_path / top_info["pl"]).string(), db);
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing bookshelf dataset: " << e.what() << std::endl;
        return false;
    }
    return true;
}