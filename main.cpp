#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <tuple>
#include <cmath>
#include <iomanip>
#include <Eigen/Dense>

// Функция f(x) = x1^2 + 2*x2^2 + e^(x1^2 + x2^2)
class ObjectiveFunction {
public:
    static double evaluate(const Eigen::Vector2d& x) {
        return x(0)*x(0) + 2*x(1)*x(1) + std::exp(x(0)*x(0) + x(1)*x(1));
    }

    static Eigen::Vector2d gradient(const Eigen::Vector2d& x) {
        double exp_term = std::exp(x(0)*x(0) + x(1)*x(1));
        Eigen::Vector2d grad;
        grad(0) = 2*x(0) + 2*x(0)*exp_term;
        grad(1) = 4*x(1) + 2*x(1)*exp_term;
        return grad;
    }

    static Eigen::Matrix2d hessian(const Eigen::Vector2d& x) {
        double exp_term = std::exp(x(0)*x(0) + x(1)*x(1));
        Eigen::Matrix2d H;
        H(0,0) = 2 + 2*exp_term + 4*x(0)*x(0)*exp_term;
        H(0,1) = 4*x(0)*x(1)*exp_term;
        H(1,0) = 4*x(0)*x(1)*exp_term;
        H(1,1) = 4 + 2*exp_term + 4*x(1)*x(1)*exp_term;
        return H;
    }
};

// Одномерная функция для поиска по направлению
class LineSearchFunction {
private:
    const Eigen::Vector2d& x_;
    const Eigen::Vector2d& d_;

public:
    LineSearchFunction(const Eigen::Vector2d& x, const Eigen::Vector2d& d)
        : x_(x), d_(d) {}

    double operator()(double alpha) const {
        return ObjectiveFunction::evaluate(x_ + alpha * d_);
    }
};

// Поиск шага методом золотого сечения
class GoldenSectionSearch {
public:
    static double search(const Eigen::Vector2d& x, const Eigen::Vector2d& d,
                        double tol = 1e-8, int max_iter = 100) {
        double a = 0.0, b = 2.0;
        double phi = (std::sqrt(5.0) - 1.0) / 2.0;

        double c = b - phi * (b - a);
        double d_point = a + phi * (b - a);

        LineSearchFunction phi_func(x, d);
        double fc = phi_func(c);
        double fd = phi_func(d_point);

        int iter = 0;
        while (std::abs(b - a) > tol && iter < max_iter) {
            if (fc < fd) {
                b = d_point;
                d_point = c;
                fd = fc;
                c = b - phi * (b - a);
                fc = phi_func(c);
            } else {
                a = c;
                c = d_point;
                fc = fd;
                d_point = a + phi * (b - a);
                fd = phi_func(d_point);
            }
            iter++;
        }

        return (a + b) / 2.0;
    }
};

// Базовый класс для методов оптимизации
class Optimizer {
protected:
    std::string name_;

public:
    Optimizer(std::string  name) : name_(std::move(name)) {}
    virtual ~Optimizer() = default;

    virtual int optimize(const Eigen::Vector2d& x0, double eps,
                        Eigen::Vector2d& x_opt,
                        std::vector<std::tuple<int, double, double, double>>& trajectory) = 0;

    const std::string& getName() const { return name_; }

    void printResults(int iterations, const Eigen::Vector2d& x_opt,
                     double f_opt, double grad_norm) const {
        std::cout << "\n" << name_ << ":" << std::endl;
        std::cout << "  Iterations: " << iterations << std::endl;
        std::cout << "  Optimal point: (" << x_opt(0) << ", " << x_opt(1) << ")" << std::endl;
        std::cout << "  Optimal value: " << f_opt << std::endl;
        std::cout << "  Gradient norm: " << grad_norm << std::endl;
    }
};

// Градиентный спуск
class GradientDescent : public Optimizer {
public:
    GradientDescent() : Optimizer("Gradient Descent") {}

    int optimize(const Eigen::Vector2d& x0, double eps,
                Eigen::Vector2d& x_opt,
                std::vector<std::tuple<int, double, double, double>>& trajectory) override {
        Eigen::Vector2d x = x0;
        int iter = 0;
        constexpr int max_iter = 1000;

        trajectory.clear();
        trajectory.emplace_back(iter, x(0), x(1), ObjectiveFunction::evaluate(x));

        while (iter < max_iter) {
            Eigen::Vector2d grad = ObjectiveFunction::gradient(x);
            double grad_norm = grad.norm();

            if (grad_norm < eps) {
                break;
            }

            Eigen::Vector2d d = -grad;
            double alpha = GoldenSectionSearch::search(x, d);
            x = x + alpha * d;

            iter++;
            trajectory.emplace_back(iter, x(0), x(1), ObjectiveFunction::evaluate(x));
        }

        x_opt = x;
        return iter;
    }
};

// BFGS метод
class BFGS : public Optimizer {
public:
    BFGS() : Optimizer("BFGS") {}

    int optimize(const Eigen::Vector2d& x0, double eps,
                Eigen::Vector2d& x_opt,
                std::vector<std::tuple<int, double, double, double>>& trajectory) override {
        Eigen::Vector2d x = x0;
        int iter = 0;
        const int max_iter = 1000;

        // Инициализируем обратную матрицу Гессе единичной матрицей
        Eigen::Matrix2d H_inv = Eigen::Matrix2d::Identity();
        Eigen::Vector2d grad = ObjectiveFunction::gradient(x);

        trajectory.clear();
        trajectory.emplace_back(iter, x(0), x(1), ObjectiveFunction::evaluate(x));

        while (iter < max_iter) {
            double grad_norm = grad.norm();

            if (grad_norm < eps) {
                break;
            }

            // Направление спуска
            Eigen::Vector2d d = -H_inv * grad;

            // Поиск шага
            double alpha = GoldenSectionSearch::search(x, d);

            // Новая точка
            Eigen::Vector2d x_new = x + alpha * d;
            Eigen::Vector2d grad_new = ObjectiveFunction::gradient(x_new);

            // Разности
            Eigen::Vector2d s = x_new - x;
            Eigen::Vector2d y = grad_new - grad;

            // Обновление обратной матрицы Гессе (BFGS формула)
            double sy = s.dot(y);
            if (sy > 1e-10) {
                Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
                Eigen::Matrix2d rho_s_y = (s * y.transpose()) / sy;
                H_inv = (I - rho_s_y) * H_inv * (I - rho_s_y.transpose()) + (s * s.transpose()) / sy;
            }

            x = x_new;
            grad = grad_new;
            iter++;

            trajectory.emplace_back(iter, x(0), x(1), ObjectiveFunction::evaluate(x));
        }

        x_opt = x;
        return iter;
    }
};

// Метод Хука-Джикса
class HookeJeeves : public Optimizer {
public:
    HookeJeeves() : Optimizer("Hooke-Jeeves") {}

    int optimize(const Eigen::Vector2d& x0, double eps,
                Eigen::Vector2d& x_opt,
                std::vector<std::tuple<int, double, double, double>>& trajectory) override {
        Eigen::Vector2d x_base = x0;
        Eigen::Vector2d x_new = x0;
        int iter = 0;
        const int max_iter = 1000;

        double delta = 1.0;
        double delta_min = eps;
        double alpha = 0.5;

        trajectory.clear();
        trajectory.emplace_back(iter, x_base(0), x_base(1), ObjectiveFunction::evaluate(x_base));

        while (delta > delta_min && iter < max_iter) {
            Eigen::Vector2d x_temp = x_new;

            // Исследовательский поиск
            for (int i = 0; i < 2; i++) {
                Eigen::Vector2d x_try = x_temp;
                x_try(i) += delta;
                if (ObjectiveFunction::evaluate(x_try) < ObjectiveFunction::evaluate(x_temp)) {
                    x_temp = x_try;
                } else {
                    x_try(i) -= 2 * delta;
                    if (ObjectiveFunction::evaluate(x_try) < ObjectiveFunction::evaluate(x_temp)) {
                        x_temp = x_try;
                    }
                }
            }

            // Ускоряющий шаг
            if (ObjectiveFunction::evaluate(x_temp) < ObjectiveFunction::evaluate(x_new)) {
                Eigen::Vector2d x_pattern = x_temp + (x_temp - x_base);
                if (ObjectiveFunction::evaluate(x_pattern) < ObjectiveFunction::evaluate(x_temp)) {
                    x_base = x_new;
                    x_new = x_pattern;
                } else {
                    x_base = x_new;
                    x_new = x_temp;
                }
            } else {
                delta *= alpha;
                x_base = x_new;
            }

            iter++;
            trajectory.emplace_back(iter, x_new(0), x_new(1), ObjectiveFunction::evaluate(x_new));
        }

        x_opt = x_new;
        return iter;
    }
};

// Менеджер для записи результатов
class ResultsManager {
    std::ofstream csv_file_;

public:
    ResultsManager(const std::string& filename) {
        csv_file_.open(filename);
        csv_file_ << "iteration,x1,x2,f_value,method" << std::endl;
    }

    ~ResultsManager() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

    void writeTrajectory(const std::vector<std::tuple<int, double, double, double>>& trajectory,
                        const std::string& method_name) {
        for (const auto& point : trajectory) {
            csv_file_ << std::get<0>(point) << ","
                     << std::get<1>(point) << ","
                     << std::get<2>(point) << ","
                     << std::get<3>(point) << ","
                     << method_name << std::endl;
        }
    }
};

int main() {
    ResultsManager results("all_trajectories.csv");

    Eigen::Vector2d x0(1.0, 1.1);
    std::vector<double> epsilons = {0.1, 0.01, 0.001};

    std::cout << std::setprecision(8);

    // Создаем оптимизаторы
    GradientDescent gd;
    BFGS bfgs;
    HookeJeeves hj;

    for (double eps : epsilons) {
        std::cout << "\n========== EPS = " << eps << " ==========" << std::endl;

        std::string eps_str = (eps == 0.1) ? "0.1" : (eps == 0.01) ? "0.01" : "0.001";

        Eigen::Vector2d x_opt;
        std::vector<std::tuple<int, double, double, double>> trajectory;
        int iter;

        // Градиентный спуск
        iter = gd.optimize(x0, eps, x_opt, trajectory);
        double f_opt = ObjectiveFunction::evaluate(x_opt);
        double grad_norm = ObjectiveFunction::gradient(x_opt).norm();
        gd.printResults(iter, x_opt, f_opt, grad_norm);
        results.writeTrajectory(trajectory, "GD_tol_" + eps_str);

        // BFGS
        iter = bfgs.optimize(x0, eps, x_opt, trajectory);
        f_opt = ObjectiveFunction::evaluate(x_opt);
        grad_norm = ObjectiveFunction::gradient(x_opt).norm();
        bfgs.printResults(iter, x_opt, f_opt, grad_norm);
        results.writeTrajectory(trajectory, "BFGS_tol_" + eps_str);

        // Хук-Дживс
        iter = hj.optimize(x0, eps, x_opt, trajectory);
        f_opt = ObjectiveFunction::evaluate(x_opt);
        grad_norm = ObjectiveFunction::gradient(x_opt).norm();
        hj.printResults(iter, x_opt, f_opt, grad_norm);
        results.writeTrajectory(trajectory, "HJ_tol_" + eps_str);
    }

    std::cout << "\n========== Results saved to all_trajectories.csv ==========" << std::endl;

    return 0;
}