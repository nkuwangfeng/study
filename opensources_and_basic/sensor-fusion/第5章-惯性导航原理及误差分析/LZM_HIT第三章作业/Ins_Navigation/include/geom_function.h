/*
 * @Description: INS 常用函数
 * @Author: Zm Liu
 * @Date: 
 */
#ifndef GEO_FUNCTIONS_H_
#define GEO_FUNCTIONS_H_
#define GM 3.986004418e14                    // m3 / (s2)
#define Re 6378137                           // #m
#define FLATTENING 1 / 298.257223563         // #Earth flattening, f = (a - b) / a
#define ECCENTRICITY 0.0818191908426215      // #Earth eccentricy, e2 = 2 *f - f ^ 2
#define E_SQR (ECCENTRICITY * ECCENTRICITY)  // #squared eccentricity
#define W_IE 7292115e-11                     // Earth's rotation rate
#define DEG2RAD 0.0174533
#define RAD2DEG 57.295779

#include <Eigen/Core>
struct EarthParameters
{
  double rm;    // meridian radius, m
  double rn;    // normal radius, m
  double g;     // gravity, m / s / s
  double sl;    // sin(Lat)
  double cl;    // cos(lat)
  double w_ie;  // Earth's rotation rate w.r.t the inertial frame, rad/s
};

inline EarthParameters getEarthParameter(const double& latitude, const double& longitude, const double& height)
{
  double normal_gravity = 9.7803253359;
  double k = 0.00193185265241;
  double m = 0.00344978650684;
  EarthParameters earth_param;
  earth_param.sl = sin(latitude);
  earth_param.cl = cos(latitude);
  double sl_sqr = earth_param.sl * earth_param.sl;
  earth_param.rm = (Re * (1 - E_SQR)) / (std::sqrt(1.0 - E_SQR * sl_sqr) * (1.0 - E_SQR * sl_sqr));
  earth_param.rn = Re / (std::sqrt(1.0 - E_SQR * sl_sqr));
  double g1 = normal_gravity * (1 + k * sl_sqr) / std::sqrt(1.0 - E_SQR * sl_sqr);
  earth_param.g = g1 * (1.0 - (2.0 / Re) * (1.0 + FLATTENING + m - 2.0 * FLATTENING * sl_sqr) * height +
                        3.0 * height * height / Re / Re);
  earth_param.w_ie = W_IE;
  return earth_param;
}

inline Eigen::Vector3d lla2ecef(const double& latitude, const double& longitude, const double& height)
{
  double sl = sin(latitude);
  double cl = cos(latitude);
  double sl_sqr = sl * sl;
  double r = Re / std::sqrt(1.0 - E_SQR * sl_sqr);
  double rho = (r + height) * cl;
  double x = rho * cos(longitude);
  double y = rho * sin(longitude);
  double z = (r * (1.0 - E_SQR) + height) * sl;
  std::cout << "lla2ecef is: " << std::endl;
  std::cout << std::setprecision(15) << sl << " " << cl << " " << sl_sqr << " " << E_SQR << " " << r << " " << rho
            << " " << x << " " << y << " " << z << std::endl;
  // Eigen::Vector3d(-2707459.27623115, 4688701.05200736, 3360102.29721514);
  return Eigen::Vector3d(x, y, z);
}

#endif
