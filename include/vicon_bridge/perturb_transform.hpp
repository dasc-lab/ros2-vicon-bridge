// Perturb Transforms
// Devansh Agrawal Nov 2023

#ifndef PERTURB_HPP
#define PERTURB_HPP

#include <Eigen/Geometry>
#include <cmath>

namespace perturb {

using namespace Eigen;

inline double unitRand() { return (double)rand() / double(RAND_MAX); }

// returns a unit vector uniformly on a sphere
template <typename T> Vector3<T> randUnitVector() {

  double phi = 2 * M_PI * unitRand();
  double cos_theta = 2 * unitRand() - 1;
  double theta = std::acos(cos_theta);

  return Vector3<T>(std::sin(theta) * std::cos(phi),
                    std::sin(theta) * std::sin(phi), std::cos(theta));
}

// returns a random vector of magnitude less than max_length
template <typename T> Vector3<T> randVector(double max_length) {

  double x = 2 * unitRand() - 1;
  double y = 2 * unitRand() - 1;
  double z = 2 * unitRand() - 1;

  double l = max_length * unitRand();

  Vector3<T> v(x, y, z);

  return (l / v.norm()) * v;
}

// return a random rotation with a maximum angle of max_angle (rad)
template <typename T> AngleAxis<T> randRotation(double max_angle) {

  // get a random unit vector
  Vector3<T> v = randUnitVector<T>();

  // get the angle
  double angle = max_angle * unitRand();

  // create the axis angle
  return AngleAxis<T>(angle, v);
}

// returns a perturbed rotation matrix X satisfying
// || X - R ||_frobenius <= epsilon_R
template <typename T> Matrix3<T> perturb(Matrix3<T> R, double epsilon_R) {
  if (epsilon_R < 0)
    throw std::invalid_argument("epsilon_R must be positive");

  double L = std::max(-1.0, 1 - epsilon_R * epsilon_R / 4);
  double max_angle = std::acos(L);

  return randRotation<T>(max_angle) * R;
}

template <typename T, typename OrientationType, typename TranslationType>
Transform<T, 3, Eigen::Isometry>
fromRotationTranslation(OrientationType rotation, TranslationType translation) {

  Transform<T, 3, Eigen::Isometry> H;
  H.setIdentity();
  H.rotate(rotation);
  H.translate(translation);
  return H;
}

template <typename T>
Transform<T, 3, Eigen::Isometry> perturb(Transform<T, 3, Eigen::Isometry> &H,
                                         double epsilon_R, double epsilon_t) {

  if ((epsilon_R < 0) || (epsilon_t < 0)) {
    throw std::invalid_argument("epsilon_R, epsilon_t must be positive");
  }

  // first get the perturbed rotation matrix
  Matrix3<T> R = H.linear();
  Matrix3<T> Rhat = perturb(R, epsilon_R);

  // construct a random vector of the right length
  Vector3<T> that = H.translation() + randVector<T>(epsilon_t);

  // create the new transform matrix
  Transform<T, 3, Eigen::Isometry> Hhat;
  Hhat.setIdentity();
  Hhat.rotate(Rhat);
  Hhat.translate(that);

  return Hhat;
}

} // namespace perturb

#endif // PERTURB_HPP
