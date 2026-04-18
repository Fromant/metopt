#include <Eigen/Dense>
#include <chrono>
#include <iostream>
constexpr int N = 512;
constexpr int iterations = 100;

using Matrix = Eigen::Matrix<float, N, N>;

int main() {

    Eigen::MatrixXd A = Eigen::MatrixXd::Random(N, N);
    Eigen::MatrixXd B = Eigen::MatrixXd::Random(N, N);
    Eigen::MatrixXd C(N, N);

    for (int i = 0; i < iterations; ++i) {
        C = A * B;
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        C = A * B;
    }
    auto end = std::chrono::high_resolution_clock::now();

    double elapsed = std::chrono::duration<double>(end - start).count();
    double avg_time = elapsed / iterations;

    long long flops = 2LL * N * N * N;
    double gflops = (flops * iterations) / elapsed / 1e9;

    std::cout << "Matrix size: " << N << "x" << N << "\n";
    std::cout << "Iterations: " << iterations << "\n";
    std::cout << "Total time: " << elapsed << "s\n";
    std::cout << "Avg time per multiply: " << avg_time << "s\n";
    std::cout << "GFlops: " << gflops << "\n";

    return 0;
}
