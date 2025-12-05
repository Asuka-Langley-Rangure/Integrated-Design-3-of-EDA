// placement/wirelength.cpp
#include "wirelength.h"
#include <cmath>
#include <numeric>
#include <bits/stdc++.h>

double WirelengthModel::computeCost(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& y) const {
    double total = 0.0;

    for (const auto& net : db->nets) {
        if (net.pins.empty()) continue;

        // collect coordinates
        std::vector<double> xs, ys;
        xs.reserve(net.pins.size());
        ys.reserve(net.pins.size());
        for (int idx : net.pins) {
            xs.push_back(x(idx));
            ys.push_back(y(idx));
        }

        auto log_sum_exp = [&](const std::vector<double>& vals, int sign) {
            // 数值稳定版 log-sum-exp
            double maxv = -1e100;
            for (double v : vals) maxv = std::max(maxv, sign * v / gamma);
            double sum = 0.0;
            for (double v : vals) sum += std::exp(sign * v / gamma - maxv);
            if (sum < 1e-300) sum = 1e-300; // 避免 log(0)
            return gamma * (maxv + std::log(sum));
        };


        total += log_sum_exp(xs, +1) + log_sum_exp(xs, -1)
               + log_sum_exp(ys, +1) + log_sum_exp(ys, -1);
    }
    if (!std::isfinite(total)) {
        std::cerr << "[WARN] computeCost() returned NaN or INF!\n";
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
        if (net.pins.size() <= 1) continue;

        // Collect coordinates
        std::vector<int> idxs = net.pins;
        std::vector<double> xs(idxs.size()), ys(idxs.size());
        for (size_t k = 0; k < idxs.size(); ++k) {
            xs[k] = x(idxs[k]);
            ys[k] = y(idxs[k]);
        }

        // --- Compute exp terms for x ---
        std::vector<double> exp_pos_x, exp_neg_x, exp_pos_y, exp_neg_y;
        exp_pos_x.reserve(idxs.size());
        exp_neg_x.reserve(idxs.size());
        exp_pos_y.reserve(idxs.size());
        exp_neg_y.reserve(idxs.size());

        for (size_t k = 0; k < idxs.size(); ++k) {
            double px = xs[k] / gamma;
            double py = ys[k] / gamma;
            // 限制 exp 输入范围 [-50, +50] 防止溢出
            px = std::max(-50.0, std::min(50.0, px));
            py = std::max(-50.0, std::min(50.0, py));
            exp_pos_x.push_back(std::exp(px));
            exp_neg_x.push_back(std::exp(-px));
            exp_pos_y.push_back(std::exp(py));
            exp_neg_y.push_back(std::exp(-py));
        }


        double sum_pos_x = std::accumulate(exp_pos_x.begin(), exp_pos_x.end(), 0.0);
        double sum_neg_x = std::accumulate(exp_neg_x.begin(), exp_neg_x.end(), 0.0);
        double sum_pos_y = std::accumulate(exp_pos_y.begin(), exp_pos_y.end(), 0.0);
        double sum_neg_y = std::accumulate(exp_neg_y.begin(), exp_neg_y.end(), 0.0);

        for (size_t k = 0; k < idxs.size(); ++k) {
            int i = idxs[k];
            grad_x(i) += (exp_pos_x[k] / sum_pos_x - exp_neg_x[k] / sum_neg_x);
            grad_y(i) += (exp_pos_y[k] / sum_pos_y - exp_neg_y[k] / sum_neg_y);
        }
    }
}

double WirelengthModel::computeHPWL(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& y) const {
    double total = 0.0;
    for (const auto& net : db->nets) {
        if (net.pins.empty()) continue;
        double minx=1e100, miny=1e100, maxx=-1e100, maxy=-1e100;
        for (int idx : net.pins) {
            minx = std::min(minx, x(idx)); maxx = std::max(maxx, x(idx));
            miny = std::min(miny, y(idx)); maxy = std::max(maxy, y(idx));
        }
        total += (maxx - minx) + (maxy - miny);
    }
    return total;
}

