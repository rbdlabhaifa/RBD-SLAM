#ifndef PCL_OPERATIONS_H_
#define PCL_OPERATIONS_H_

#include <pcl/point_types.h>

namespace PCLOperations {
    pcl::PointXYZ operator-(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
    pcl::PointXYZ operator/(const pcl::PointXYZ& p, float d);

    float norm(const pcl::PointXYZ& p);

    /**
     * @brief Given 2 vectors of size 3*1 and 3*1
     * returns the matrix multipication
     * @param pcl::PointXYZ p1[in] -> is the first point
     * @param pcl::PointXYZ p2[in] -> is the second point
     * @return float -> The matrix mul (1*3)*(3*1)
     */
    float operator*(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

    pcl::PointXYZ operator*(const pcl::PointXYZ& p, float a);
    pcl::PointXYZ operator*(float a, const pcl::PointXYZ& p);
    pcl::PointXYZ operator+(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

    bool operator==(const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs);

    pcl::PointXYZ cross_product(const pcl::PointXYZ& v_a,
                                const pcl::PointXYZ& v_b);
}  // namespace PCLOperations

#endif  // PCL_OPERATIONS_H_
