// placement/wirelength.cpp
#include "wirelength.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>

inline double log_sum_exp_scaled(const std::vector<double>& vals) {
    double m = -1e100;
    for (double v : vals) m = std::max(m, v);
    double sum = 0.0;
    for (double v : vals) sum += std::exp(v - m);
    return m + std::log(std::max(sum, 1e-300));
}

double WirelengthModel::computeCost(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& y) const {
    double total = 0.0;

    for (const auto& net : db->nets) {
        if (net.pins.size() <= 1) continue;

        std::vector<double> pos_x, neg_x, pos_y, neg_y;
        pos_x.reserve(net.pins.size());
        neg_x.reserve(net.pins.size());
        pos_y.reserve(net.pins.size());
        neg_y.reserve(net.pins.size());

        for (int idx : net.pins) {
            double xg = x(idx) / gamma;
            double yg = y(idx) / gamma;

            pos_x.push_back(xg);
            neg_x.push_back(-xg);
            pos_y.push_back(yg);
            neg_y.push_back(-yg);
        }

        double lse_x = gamma * (log_sum_exp_scaled(pos_x) + log_sum_exp_scaled(neg_x));
        double lse_y = gamma * (log_sum_exp_scaled(pos_y) + log_sum_exp_scaled(neg_y));

        total += lse_x + lse_y;
    }

    if (!std::isfinite(total)) {
        std::cerr << "[WARN] LSE computeCost() overflow!\n";
    }

    return total;
}

void WirelengthModel::computeGradient(const Eigen::VectorXd& x,
                                      const Eigen::VectorXd& y,
                                      Eigen::VectorXd& grad_x,
                                      Eigen::VectorXd& grad_y) const {
    grad_x.setZero();
    grad_y.setZero();

    for (const auto& net : db->nets) {
        size_t n = net.pins.size();
        if (n <= 1) continue;

        std::vector<double> pos_x(n), neg_x(n), pos_y(n), neg_y(n);

        // collect and scale
        for (size_t k = 0; k < n; ++k) {
            double xg = x(net.pins[k]) / gamma;
            double yg = y(net.pins[k]) / gamma;

            pos_x[k] = xg;
            neg_x[k] = -xg;
            pos_y[k] = yg;
            neg_y[k] = -yg;
        }

        // Max trick for numerical stability
        double max_pos_x = *std::max_element(pos_x.begin(), pos_x.end());
        double max_neg_x = *std::max_element(neg_x.begin(), neg_x.end());
        double max_pos_y = *std::max_element(pos_y.begin(), pos_y.end());
        double max_neg_y = *std::max_element(neg_y.begin(), neg_y.end());

        // Compute exp terms
        std::vector<double> exp_pos_x(n), exp_neg_x(n), exp_pos_y(n), exp_neg_y(n);
        for (size_t k = 0; k < n; ++k) {
            exp_pos_x[k] = std::exp(pos_x[k] - max_pos_x);
            exp_neg_x[k] = std::exp(neg_x[k] - max_neg_x);

            exp_pos_y[k] = std::exp(pos_y[k] - max_pos_y);
            exp_neg_y[k] = std::exp(neg_y[k] - max_neg_y);
        }

        double sum_pos_x = std::accumulate(exp_pos_x.begin(), exp_pos_x.end(), 0.0);
        double sum_neg_x = std::accumulate(exp_neg_x.begin(), exp_neg_x.end(), 0.0);
        double sum_pos_y = std::accumulate(exp_pos_y.begin(), exp_pos_y.end(), 0.0);
        double sum_neg_y = std::accumulate(exp_neg_y.begin(), exp_neg_y.end(), 0.0);

        // ∂L/∂x_i = exp_pos/sum_pos − exp_neg/sum_neg
        for (size_t k = 0; k < n; ++k) {
            int id = net.pins[k];
            grad_x(id) += (exp_pos_x[k] / sum_pos_x - exp_neg_x[k] / sum_neg_x);
            grad_y(id) += (exp_pos_y[k] / sum_pos_y - exp_neg_y[k] / sum_neg_y);
        }
    }
}

double WirelengthModel::computeHPWL(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& y) const {
    double total = 0.0;

    for (const auto& net : db->nets) {
        if (net.pins.size() <= 1) continue;

        double minx = 1e100, maxx = -1e100;
        double miny = 1e100, maxy = -1e100;

        for (int idx : net.pins) {
            double xx = x(idx), yy = y(idx);
            minx = std::min(minx, xx); maxx = std::max(maxx, xx);
            miny = std::min(miny, yy); maxy = std::max(maxy, yy);
        }

        total += (maxx - minx) + (maxy - miny);
    }

    return total;
}
