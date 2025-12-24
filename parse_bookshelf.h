#ifndef PARSE_BOOKSHELF_H
#define PARSE_BOOKSHELF_H

#include "../MyPlace/myplace.h"
#include "../PlaceData/placedata.h"

#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <algorithm>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

// 解析 TOP (aux 文件)
std::tuple<std::map<std::string, std::string>, std::string>
parse_top(const std::string &filename);

// 解析 NODES 文件
void parse_nodes(const std::string &filename, PlaceData* db);

// 解析 SCL 文件
void parse_scl(const std::string &filename, PlaceData* db);

// 解析 NETS 文件
void parse_nets(const std::string &filename, PlaceData* db);

// 解析 PL 文件
void parse_pl(const std::string &filename, PlaceData* db);

// 打印数据库信息
void PrintDatabaseSummary(const PlaceData* db);

// 输出 PlaceData 信息
void PrintPlaceData(const PlaceData* db);

// 解析 BOOKSHELF 数据集
bool ParseBookshelfDataset(const fs::path tmp_path, PlaceData* db);

// 任务三的输出设计总结函数
void printDesignSummary(const PlaceData* db, int binRows, int binCols, double binAddTimeSec);

#endif
