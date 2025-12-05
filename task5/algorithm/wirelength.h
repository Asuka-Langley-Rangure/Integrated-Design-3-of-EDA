// placement/wirelength.h
#pragma once
#include "db.h"
#include "../eigen3/Eigen/Dense"
#include <cmath>

class WirelengthModel {
public:
    explicit WirelengthModel(const PlacementDB* db, double gamma = 100.0)
        : db(db), gamma(gamma) {}

    // 计算平滑线长近似
    double computeCost(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const;

    // 计算梯度
    void computeGradient(const Eigen::VectorXd& x,
                         const Eigen::VectorXd& y,
                         Eigen::VectorXd& grad_x,
                         Eigen::VectorXd& grad_y) const;
    double computeHPWL(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const; // ← 新增
private:
    const PlacementDB* db;
    double gamma;
};
