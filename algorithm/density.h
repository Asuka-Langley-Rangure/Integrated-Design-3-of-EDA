// placement/density.h
#pragma once
#include "db.h"
#include "../eigen3/Eigen/Dense"
#include <vector>

class DensityModel {
public:
    explicit DensityModel(const PlacementDB* db, int bin_rows=64, int bin_cols=64)
        : db(db), nRows(bin_rows), nCols(bin_cols) {}

    double computeCost(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const;

    void computeGradient(const Eigen::VectorXd& x,
                         const Eigen::VectorXd& y,
                         Eigen::VectorXd& grad_x,
                         Eigen::VectorXd& grad_y) const;

    double computeOverflow(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const;
    void saveDensityMap(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& y,
                    const std::string& filename) const;


private:
    const PlacementDB* db;
    int nRows, nCols;
};
