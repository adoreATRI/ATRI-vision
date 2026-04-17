#ifndef ATRI_TRACKER__CIRCLE_FITTER_HPP_
#define ATRI_TRACKER__CIRCLE_FITTER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <deque>

namespace atri_tracker
{

struct FittedCircle3D
{
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  double radius;
  bool valid = false;
};

class CircleFitter
{
public:
  CircleFitter(int max_history_size = 150) : max_size_(max_history_size) {}

  void addPoint(const Eigen::Vector3d & pt)
  {
    if (history_points_.size() >= max_size_) {
      history_points_.pop_front();
    }
    history_points_.push_back(pt);
  }
  void clear() { history_points_.clear(); }
  size_t getPointSize() const { return history_points_.size(); }

  FittedCircle3D fit()
  {
    FittedCircle3D result;
    if (history_points_.size() < 15) {
      return result;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & p : history_points_) {
      centroid += p;
    }
    centroid /= history_points_.size();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto & p : history_points_) {
      Eigen::Vector3d diff = p - centroid;
      cov += diff * diff.transpose();
    }
    cov /= history_points_.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);

    Eigen::Vector3d normal = solver.eigenvectors().col(0);

    Eigen::Vector3d u = solver.eigenvectors().col(1);
    Eigen::Vector3d v = solver.eigenvectors().col(2);

    result.normal = normal.normalized();

    size_t N = history_points_.size();
    Eigen::MatrixXd A(N, 3);
    Eigen::VectorXd b(N);

    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector3d diff = history_points_[i] - centroid;
      double x_2d = diff.dot(u);
      double y_2d = diff.dot(v);

      A(i, 0) = x_2d;
      A(i, 1) = y_2d;
      A(i, 2) = 1.0;
      b(i) = x_2d * x_2d + y_2d * y_2d;
    }

    Eigen::Vector3d solution = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    double cx_2d = solution(0) / 2.0;
    double cy_2d = solution(1) / 2.0;
    double r_sq = cx_2d * cx_2d + cy_2d * cy_2d + solution(2);

    if (r_sq <= 0) return result;

    result.radius = std::sqrt(r_sq);

    result.center = centroid + cx_2d * u + cy_2d * v;
    result.valid = true;

    return result;
  }

private:
  size_t max_size_;
  std::deque<Eigen::Vector3d> history_points_;
};

}  // namespace atri_tracker

#endif  // ATRI_TRACKER__CIRCLE_FITTER_HPP_