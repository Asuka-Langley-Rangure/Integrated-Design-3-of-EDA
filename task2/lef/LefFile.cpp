#include "LefFile.h"
#include <iostream>

void LefFile::addSite(const LefSite& site) { sites.push_back(site); }
void LefFile::addLayer(const LefLayer& layer) { layers.push_back(layer); }
void LefFile::addMacro(const LefMacro& macro) { macros.push_back(macro); }

LefLayer* LefFile::findLayer(const std::string& name) {
    for (auto& l : layers) if (l.name == name) return &l;
    return nullptr;
}

LefSite* LefFile::findSite(const std::string& name) {
    for (auto& s : sites) if (s.name == name) return &s;
    return nullptr;
}

void LefFile::printSummary() const {
    std::cout << "LEF Version: " << version << "\n";
    std::cout << "Bus Bit Chars: " << busBitChars << "\n";
    std::cout << "Divider Char: " << dividerChar << "\n";
    std::cout << "Units: " << units.databaseMicrons << " database microns\n";
    std::cout << "Manufacturing Grid: " << manufacturingGrid << "\n";

    std::cout << "\nSites (" << sites.size() << "):\n";
    for (const auto& s : sites) {
        std::cout << "  " << s.name 
                  << " Size=(" << s.size.first << "," << s.size.second << ")"
                  << " Class=" << s.siteClass << "\n";
        std::cout << "    Symmetry: ";
        for (size_t i = 0; i < s.symmetry.size(); ++i) {
            std::cout << s.symmetry[i];
            if (i + 1 < s.symmetry.size()) std::cout << ", ";
        }
        std::cout << "\n";
    }

    std::cout << "\nLayers (" << layers.size() << "):\n";
    for (const auto& l : layers) {
        std::cout << "  " << l.name << " [" << l.type << "]\n";

        if (!l.routingFactor.empty()) {
            std::cout << "    RoutingFactor: ";
            for (size_t i = 0; i < l.routingFactor.size(); ++i) {
                std::cout << l.routingFactor[i];
                if (i + 1 < l.routingFactor.size()) std::cout << ", ";
            }
            std::cout << "\n";
        }

        if (l.type == "ROUTING") {
            std::cout << "    Direction: " << l.direction << "\n";
            if (!l.pitch.empty()) {
                std::cout << "    Pitch: ";
                for (size_t i = 0; i < l.pitch.size(); ++i) {
                    std::cout << l.pitch[i];
                    if (i + 1 < l.pitch.size()) std::cout << ", ";
                }
                std::cout << "\n";
            }
            if (!l.offset.empty()) {
                std::cout << "    Offset: ";
                for (size_t i = 0; i < l.offset.size(); ++i) {
                    std::cout << l.offset[i];
                    if (i + 1 < l.offset.size()) std::cout << ", ";
                }
                std::cout << "\n";
            }
            std::cout << "    Height: " << l.height << "\n";
            std::cout << "    Thickness: " << l.thickness << "\n";
            std::cout << "    FillSpacing: " << l.fillSpacing << "\n";
            std::cout << "    Width: " << l.width << "\n";
        }
    }
}

