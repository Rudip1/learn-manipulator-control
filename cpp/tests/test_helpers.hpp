#pragma once
// Shared helpers for the Catch2 tests.

#include <Eigen/Core>
#include <random>

namespace test_helpers {

constexpr double kPi = 3.14159265358979323846;

/// True when two matrices agree entry-wise to within @p tol.
template <typename A, typename B>
bool near(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b, double tol = 1e-9) {
    return a.rows() == b.rows() && a.cols() == b.cols() && (a - b).cwiseAbs().maxCoeff() <= tol;
}

/// Deterministic random numbers so every run checks the same samples.
class Rng {
  public:
    explicit Rng(unsigned seed = 42) : gen_(seed) {}
    double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(gen_); }
    Eigen::VectorXd vector(Eigen::Index n, double lo, double hi) {
        Eigen::VectorXd v(n);
        for (Eigen::Index i = 0; i < n; ++i) v(i) = uniform(lo, hi);
        return v;
    }

  private:
    std::mt19937 gen_;
};

}  // namespace test_helpers
