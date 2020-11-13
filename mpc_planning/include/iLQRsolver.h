#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/speed/det_by_minor.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <chrono>

using namespace std;

using CppAD::AD;
using CppAD::NearEqual;
using Eigen::Matrix;
using Eigen::Dynamic;

typedef Matrix< double    , Dynamic, Dynamic > MatrixXd;
typedef Matrix< AD<double>, Dynamic, Dynamic > MatrixAD;

typedef Matrix< double    , Dynamic, 1 >       VectorXd;
typedef Matrix< AD<double>, Dynamic, 1 >       VectorAD;

#define __DYN_EULER__
// #define __DYN_RK4__

// #define __VERBOSE__

#define __DIFF_CPPAD__
// #define __DIFF_FINITE__

class iLQRsolver
{
public:
    // solver settings
    size_t state_dim = 4;
    size_t input_dim = 2;
    double dt = 0.05;
    size_t N = 20;
    size_t n_itrs = 1;
    size_t n_line = 10;
    double J_reltol = 1e-6;
    double diff_eps = 1e-5;
    bool converged = false;

    // model variables
    double lf = 1.36, lr = 1.36;
    double L = lf + lr;

    // cost related variables
    double Q_v = 5;
    double Q_y = 10;
    double R_delta = 500;
    double R_a = 40;
    double v_target = 10. * 1000./3600.;
    double y_target = 4.;

    template <typename var>
    Matrix< var, Dynamic, 1 > dynamics_continuous(
        const var &t,
        const Matrix< var, Dynamic, 1 > &x,
        const Matrix< var, Dynamic, 1 > &u)
    {
        // state, input
        var X = x[0], Y = x[1], psi = x[2], v = x[3];
        var delta = u[0], a = u[1];

        var beta = atan(lr/L*tan(delta));
        var Xdot = v * cos(psi+beta);
        var Ydot = v * sin(psi+beta);
        var psidot = v/lr * sin(beta);
        var vdot = a;

        Matrix< var, Dynamic, 1 > dx(state_dim);
        dx << Xdot, Ydot, psidot, vdot;
        return dx;
    }

    template <typename var>
    Matrix< var, 1, 1 > cost_instantaneous(
        const size_t &i,
        const Matrix< var, Dynamic, 1 > &x,
        const Matrix< var, Dynamic, 1 > &u)
    {
        // state, input
        var X = x[0], Y = x[1], psi = x[2], v = x[3];
        var delta = u[0], a = u[1];

        Matrix< var, 1, 1 > cost;
        cost[0] += Q_v * pow(v-v_target, 2);
        cost[0] += Q_y * pow(Y-y_target, 2);
        cost[0] += R_delta * pow(delta, 2);
        cost[0] += R_a * pow(a, 2);

        return cost;
    }
    
    template <typename var>
    Matrix< var, Dynamic, 1 > dynamics_discrete(
        const size_t &i,
        const Matrix< var, Dynamic, 1 > &x,
        const Matrix< var, Dynamic, 1 > &u)
    {
#ifdef __DYN_EULER__
        Matrix< var, Dynamic, 1 > dx(state_dim);
        Matrix< var, Dynamic, 1 > x_new(state_dim);
        var t = i * dt;

        dx = dynamics_continuous(t, x, u);
        x_new = x + dx * dt;
        return x_new;
#endif

#ifdef __DYN_RK4__
        Matrix< var, Dynamic, 1 > dx(state_dim);
        Matrix< var, Dynamic, 1 > x_new(state_dim);
        Matrix< var, Dynamic, 1 > k1(state_dim);
        Matrix< var, Dynamic, 1 > k2(state_dim);
        Matrix< var, Dynamic, 1 > k3(state_dim);
        Matrix< var, Dynamic, 1 > k4(state_dim);
        var t = i * dt;
        var t_;
        Matrix< var, Dynamic, 1 > x_(state_dim);

        k1 = dynamics_continuous(t, x, u);

        t_ = t + 0.5*dt;
        x_ = x + 0.5*k1*dt;
        k2 = dynamics_continuous(t_, x_, u);

        x_ = x + 0.5*k2*dt;
        k3 = dynamics_continuous(t_, x_, u);

        t_ = t + dt;
        x_ = x + k3*dt;
        k4 = dynamics_continuous(t_, x_, u);
        x_new = x + (k1 + 2*k2 + 2*k3 + k4) * (dt/6.0);
        return x_new;
#endif
    }

    void forward_propagation(
        const VectorXd &x0,
        const vector<VectorXd> &u_array,
        vector<VectorXd> &x_array)
    {
        x_array[0] = x0;
        for (size_t i = 0; i < N; i++)
            x_array[i+1] = dynamics_discrete(i, x_array[i], u_array[i]);
    }

    VectorXd trajectory_cost(
        const vector<VectorXd> &x_array,
        const vector<VectorXd> &u_array)
    {
        VectorXd J(1);
        for (size_t i = 0; i <= N; i++)
            J += cost_instantaneous(i, x_array[i], u_array[i]);
        return J;
    }

    void apply_control(
        const vector<VectorXd> &x_array,
        const vector<VectorXd> &u_array,
        const vector<VectorXd> &k_array,
        const vector<MatrixXd> &K_array,
        const double alpha,
        vector<VectorXd> &x_array_new,
        vector<VectorXd> &u_array_new)
    {
        x_array_new[0] = x_array[0];
        for (size_t i = 0; i < N; i++){
            u_array_new[i] = u_array[i] + alpha*k_array[i] + K_array[i]*(x_array_new[i]-x_array[i]);
            x_array_new[i+1] = dynamics_discrete(i, x_array_new[i], u_array_new[i]);
        }
    }

    void build_ADFun(
        const size_t &i,
        CppAD::ADFun<double> &f,
        CppAD::ADFun<double> &l)
    {
        VectorAD ad_xu(state_dim+input_dim);
        VectorAD ad_x(state_dim);
        VectorAD ad_u(input_dim);
        VectorAD ad_dx(state_dim);
        VectorAD ad_cost(1);
        ad_xu.setRandom(state_dim+input_dim, 1);

        CppAD::Independent(ad_xu);
        ad_x = ad_xu.head(state_dim);
        ad_u = ad_xu.tail(input_dim);
        ad_dx = dynamics_discrete(i, ad_x, ad_u);
        CppAD::ADFun<double> f_(ad_xu, ad_dx);
        f = f_;

        CppAD::Independent(ad_xu);
        ad_x = ad_xu.head(state_dim);
        ad_u = ad_xu.tail(input_dim);
        ad_cost = cost_instantaneous(i, ad_x, ad_u);
        CppAD::ADFun<double> l_(ad_xu, ad_cost);
        l = l_;
    }

    void ilqr_iterate(
        const VectorXd &x0,
        const vector<VectorXd> &u_init,
        vector<VectorXd> &x_array_opt,
        vector<VectorXd> &u_array_opt,
        vector<VectorXd> &k_array_opt,
        vector<MatrixXd> &K_array_opt)
    {
        vector<VectorXd> x_array(N+1, VectorXd(state_dim));
        vector<VectorXd> u_array(N+1, VectorXd(input_dim)); // u_array[N] should be always 0
        vector<VectorXd> x_array_new(N+1, VectorXd(state_dim));
        vector<VectorXd> u_array_new(N+1, VectorXd(input_dim));
        vector<VectorXd> k_array(N, VectorXd(input_dim));
        vector<MatrixXd> K_array(N, MatrixXd(input_dim, state_dim));
        VectorXd J_opt(1);
        VectorXd J_new(1);
        CppAD::ADFun<double> f;
        CppAD::ADFun<double> l;
        VectorXd xu(state_dim+input_dim);
        MatrixXd f_xu_t, f_xu, fx, fu;
        MatrixXd l_xu, lx, lu;
        MatrixXd l_xuxu, lxx, lux, luu;
        MatrixXd Vx, Vxx;
        MatrixXd Qx, Qu, Qxx, Quu, Qux, invQuu;

        double alpha = 1.0;
        converged = false;

        u_array = u_init;
        forward_propagation(x0, u_array, x_array);
        J_opt = trajectory_cost(x_array, u_array);

        for (int itr = 0; itr < n_itrs; itr++){
            // Initialization of Vx, Vxx
            build_ADFun(N, f, l);
            xu << x_array[N], u_array[N];

            l_xu = l.Jacobian(xu);
            lx = l_xu.topRows(state_dim);

            l_xuxu = l.Hessian(xu, 0);
            l_xuxu.resize(state_dim+input_dim, state_dim+input_dim);
            lxx = l_xuxu.topLeftCorner(state_dim, state_dim);

            Vx = lx;
            Vxx = lxx;

            // Back propagation
            for (int i = N-1; i >= 0; i--){
                build_ADFun(i, f, l);
                xu << x_array[i], u_array[i];

                f_xu_t = f.Jacobian(xu);
                f_xu_t.resize(state_dim+input_dim, state_dim);
                f_xu = f_xu_t.transpose();
                fx = f_xu.leftCols(state_dim);
                fu = f_xu.rightCols(input_dim);

                l_xu = l.Jacobian(xu);
                lx = l_xu.topRows(state_dim);
                lu = l_xu.bottomRows(input_dim);

                l_xuxu = l.Hessian(xu, 0);
                l_xuxu.resize(state_dim+input_dim, state_dim+input_dim);
                lxx = l_xuxu.topLeftCorner(state_dim, state_dim);
                lux = l_xuxu.bottomLeftCorner(input_dim, state_dim);
                luu = l_xuxu.bottomRightCorner(input_dim, input_dim);

                Qx = lx + fx.transpose() * Vx;
                Qu = lu + fu.transpose() * Vx;
                Qxx = lxx + fx.transpose() * Vxx * fx;
                Quu = luu + fu.transpose() * Vxx * fu;
                Qux = lux + fu.transpose() * Vxx * fx;

                invQuu = Quu.inverse();
                k_array[i] = -invQuu*Qu;
                K_array[i] = -invQuu*Qux;
                // k_array[i] = -Quu.ldlt().solve(Qu);
                // K_array[i] = -Quu.ldlt().solve(Qux);

                Vx = Qx - K_array[i].transpose() * Quu * k_array[i];
                Vxx = Qxx - K_array[i].transpose() * Quu * K_array[i];
            }

            // Line search
            for (int j = 0; j < n_line; j++) {
                alpha = pow(1.1, -pow(j, 2));
                // alpha = pow(0.8, j);

                apply_control(x_array, u_array, k_array, K_array, alpha, x_array_new, u_array_new);
                J_new = trajectory_cost(x_array_new, u_array_new);

                if (J_new[0] < J_opt[0]) {
                    if (abs((J_opt[0]-J_new[0])/J_opt[0]) < J_reltol) {
                        J_opt = J_new;
                        x_array = x_array_new;
                        u_array = u_array_new;
                        converged = true;
                        break;
                    }
                    else {
                        J_opt = J_new;
                        x_array = x_array_new;
                        u_array = u_array_new;
                        break;
                    }
                }
            }

            if (converged)
                break;
        }

        x_array_opt = x_array;
        u_array_opt = u_array;
        k_array_opt = k_array;
        K_array_opt = K_array;
    }
};