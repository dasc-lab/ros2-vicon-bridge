#include "vicon_bridge/perturb_transform.hpp"
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;

template <typename T> void test_rot_fro_norm(double epsilon_R, size_t N) {

  size_t fails = 0;
  double min_fro = 1e9;
  double max_fro = 0;
  double sum_fro = 0;

  for (size_t i = 0; i < N; i++) {

    Matrix3<T> R = randRotation<T>(2 * M_PI)
                       .toRotationMatrix(); // create the original matrix
    Matrix3<T> hatR = perturb(R, epsilon_R);

    // compute the frobenius norm
    double fro = (R - hatR).norm();
    min_fro = std::min(min_fro, fro);
    max_fro = std::max(max_fro, fro);
    sum_fro += fro;

    if (fro > epsilon_R) {
      fails += 1;
    }
  }

  std::cout << "min fro: " << min_fro << std::endl;
  std::cout << "max fro: " << max_fro << std::endl;
  std::cout << "mean fro: " << sum_fro / N << std::endl;
  std::cout << "fails: " << fails << std::endl;

  return;
}

int main() {

  srand(time(NULL));

  AngleAxisd R = randRotation<double>(0.01);

  Matrix3d m = R.toRotationMatrix();
  std::cout << m << std::endl << "is unitary: " << m.isUnitary() << std::endl;

  AngleAxisd R2(m);

  std::cout << "R2: " << R2.angle() << std::endl;

  // run test
  // std::cout << "eps_R = -1: " << std::endl;
  // test_rot_fro_norm<float>(-1, 1000);
  // std::cout << std::endl;
  std::cout << "eps_R = 0: " << std::endl;
  test_rot_fro_norm<double>(0.0, 1000);
  std::cout << std::endl;
  std::cout << "eps_R = 0.001: " << std::endl;
  test_rot_fro_norm<float>(0.001, 1000);
  std::cout << std::endl;
  std::cout << "eps_R = 0.01: " << std::endl;
  test_rot_fro_norm<double>(0.01, 1000);
  std::cout << std::endl;
  std::cout << "eps_R = 0.1: ";
  std::cout << std::endl;
  test_rot_fro_norm<float>(0.1, 1000);
  std::cout << std::endl;
  std::cout << "eps_R = 1: " << std::endl;
  test_rot_fro_norm<double>(1.0, 1000);
  std::cout << std::endl;
  std::cout << "eps_R = 5: " << std::endl;
  test_rot_fro_norm<float>(5, 1000);
  std::cout << std::endl;

  // now test the transforms
  std::cout << "R2: " << std::endl;
  std::cout << R2.toRotationMatrix() << std::endl;
  std::cout << "T2: " << std::endl;
  Vector3d T2 = randVector<double>(1.0);
  std::cout << T2 << std::endl;

  Transform<double, 3, Eigen::Isometry> H =
      fromRotationTranslation<double>(R2, T2);

  std::cout << "H: " << std::endl;
  std::cout << H.matrix() << std::endl;

  auto Hhat = perturb(H, 0.1, 0.0);
  std::cout << "Hhat" << std::endl;
  std::cout << Hhat.matrix() << std::endl;

  std::cout << "fronorm: " << (H.matrix() - Hhat.matrix()).norm() << std::endl;

  std::cout << "eps_R = 0, eps_t = 0.0" << std::endl;
  test_rot_fro_norm<double>(0.0, 1000);
  std::cout << std::endl;
}
