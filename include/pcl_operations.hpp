#ifndef PCL_OPERATIONS_H_
#define PCL_OPERATIONS_H_

#include <pcl/point_types.h>

namespace PCLOperations
{
pcl::PointXYZ operator-(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);
pcl::PointXYZ operator/(const pcl::PointXYZ &p, float d);

float norm(const pcl::PointXYZ &p);

/**
 * @brief Given 2 vectors of size 3*1 and 3*1
 * returns the matrix multipication
 * @param[in] p1 - is the first point
 * @param[in] p2 - is the second point
 * @returns float - The matrix mul (1*3)*(3*1)
 */
float operator*(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

pcl::PointXYZ operator*(const pcl::PointXYZ &p, float a);
pcl::PointXYZ operator*(float a, const pcl::PointXYZ &p);
pcl::PointXYZ operator+(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

bool operator==(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs);

pcl::PointXYZ cross_product(const pcl::PointXYZ &v_a, const pcl::PointXYZ &v_b);

// pcl::PointXYZ eigen3d_to_pcl_point(const Eigen::Vector3d &vec);
// Eigen::Vector3d pcl_point_to_eigen3d(const pcl::PointXYZ &point);
} // namespace PCLOperations

#endif // PCL_OPERATIONS_H_
