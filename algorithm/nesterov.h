// placement/nesterov.h
#pragma once
#include "db.h"
#include "wirelength.h"
#include "density.h"
#include "../eigen3/Eigen/Dense"
#include <iostream>
#include <vector>

class NesterovSolver {
public:
    struct Config {
        double lr = 1e-4;         // 学习率
        double momentum = 0.5;    // 动量
        double lambda = 1.0;      // 密度惩罚系数
        int    max_iters = 3000;  // 最大迭代
        double clip_grad = 1e3;   // 梯度裁剪（数值稳定）
        double stop_rel = 1e-4;   // 相对收敛阈值（位置变化）
        double target_overflow = 0.20; // 目标溢出
        int    log_every = 100;   // 打印间隔
    };

    NesterovSolver(const PlacementDB* db, const Config& cfg,
                   int bins_r=64, int bins_c=64,
                   double lse_gamma=100.0)
        : db(db), cfg(cfg),
          wl(db, lse_gamma),
          den(db, bins_r, bins_c),
          N(db->cells.size()),
          v_x(Eigen::VectorXd::Zero(N)),
          v_y(Eigen::VectorXd::Zero(N)) {
        // 记录哪些 cell 可移动
        movable.assign(N, true);
        for (size_t i=0;i<N;++i) movable[i] = !db->cells[i].fixed;
    }
    NesterovSolver(const PlacementDB* db,
               const WirelengthModel& wl_model,
               const DensityModel& den_model)
    : db(db), wl(wl_model), den(den_model),
      N(db->cells.size()),
      v_x(Eigen::VectorXd::Zero(N)),
      v_y(Eigen::VectorXd::Zero(N)) {
    movable.assign(N, true);
    for (size_t i=0; i<N; ++i) movable[i] = !db->cells[i].fixed;
    }

    double lambda() const { return cfg.lambda; }

    void step(Eigen::VectorXd& x, Eigen::VectorXd& y) {
        Eigen::VectorXd x_look = x + cfg.momentum * v_x;
        Eigen::VectorXd y_look = y + cfg.momentum * v_y;

        Eigen::VectorXd gWx(N), gWy(N), gDx(N), gDy(N);
        wl.computeGradient(x_look, y_look, gWx, gWy);
        den.computeGradient(x_look, y_look, gDx, gDy);

        Eigen::VectorXd gx = gWx + cfg.lambda * gDx;
        Eigen::VectorXd gy = gWy + cfg.lambda * gDy;

        for (size_t i=0; i<N; ++i)
            if (!movable[i]) { gx(i)=0; gy(i)=0; }

        clip(gx, cfg.clip_grad);
        clip(gy, cfg.clip_grad);

        v_x = cfg.momentum * v_x - cfg.lr * gx;
        v_y = cfg.momentum * v_y - cfg.lr * gy;

        x += v_x;
        y += v_y;

        projectToDie(x, y);
    }


    // 入口：给定初始 x,y，原地更新
    void solve(Eigen::VectorXd& x, Eigen::VectorXd& y) {
        using std::cout; using std::endl;

        // 初始 overflow 用来设置 lambda 的量级
        double overflow = den.computeOverflow(x,y);
        if (overflow > 1e-6) cfg.lambda = std::max(cfg.lambda, 0.5/overflow);

        double last_hpwl = wl.computeHPWL(x,y);

        for (int it=1; it<=cfg.max_iters; ++it) {
            // --- 1) Nesterov 预判 ---
            Eigen::VectorXd x_look = x + cfg.momentum * v_x;
            Eigen::VectorXd y_look = y + cfg.momentum * v_y;

            // --- 2) 计算梯度 ---
            Eigen::VectorXd gWx(N), gWy(N), gDx(N), gDy(N);
            wl .computeGradient (x_look, y_look, gWx, gWy);
            den.computeGradient (x_look, y_look, gDx, gDy);

            Eigen::VectorXd gx = gWx + cfg.lambda * gDx;
            Eigen::VectorXd gy = gWy + cfg.lambda * gDy;

            // 固定单元：梯度=0，速度=0
            for (size_t i=0;i<N;++i) if (!movable[i]) {
                gx(i)=0; gy(i)=0; v_x(i)=0; v_y(i)=0;
            }

            // 梯度裁剪（防爆）
            clip(gx, cfg.clip_grad); clip(gy, cfg.clip_grad);

            // --- 3) 更新速度与位置 ---
            v_x = cfg.momentum * v_x - cfg.lr * gx;
            v_y = cfg.momentum * v_y - cfg.lr * gy;
            x += v_x; y += v_y;

            // --- 4) 边界投影（保持在 DIEAREA 内）---
            projectToDie(x, y);

            // --- 5) 自适应 lambda（基于 overflow）---
            overflow = den.computeOverflow(x, y);
            if (overflow < cfg.target_overflow) cfg.lambda *= 0.95;
            else                               cfg.lambda *= 1.05;

            // --- 6) 日志 & 早停 ---
            if (it % cfg.log_every == 0 || it==1) {
                double hpwl   = wl.computeHPWL(x,y);
                double lseW   = wl.computeCost(x,y);
                cout << "[Iter " << it << "] "
                     << "HPWL=" << hpwl
                     << "  W(LSE)=" << lseW
                     << "  Overflow=" << overflow
                     << "  lambda=" << cfg.lambda
                     << "  |v|=" << 0.5*(v_x.norm()+v_y.norm())
                     << endl;

                // 相对改善很小则停
                double rel = std::abs(hpwl - last_hpwl) / std::max(1.0, last_hpwl);
                if (rel < cfg.stop_rel && overflow < cfg.target_overflow) {
                    cout << "Converged: rel=" << rel << ", overflow=" << overflow << endl;
                    break;
                }
                last_hpwl = hpwl;
            }
        }
    }

private:
    const PlacementDB* db;
    Config cfg;
    WirelengthModel wl;
    DensityModel    den;

    size_t N;
    Eigen::VectorXd v_x, v_y;
    std::vector<char> movable;

    static void clip(Eigen::VectorXd& g, double thr) {
        for (int i=0;i<g.size();++i) {
            if (g(i) >  thr) g(i) =  thr;
            if (g(i) < -thr) g(i) = -thr;
        }
    }

    void projectToDie(Eigen::VectorXd& x, Eigen::VectorXd& y) const {
        const double xl=db->die_xl, yl=db->die_yl, xh=db->die_xh, yh=db->die_yh;
        for (int i=0;i<x.size();++i) {
            if (!movable[i]) { x(i)=db->cells[i].x; y(i)=db->cells[i].y; continue; }
            if (x(i) < xl) x(i)=xl;
            if (x(i) > xh) x(i)=xh;
            if (y(i) < yl) y(i)=yl;
            if (y(i) > yh) y(i)=yh;
        }
    }
};
