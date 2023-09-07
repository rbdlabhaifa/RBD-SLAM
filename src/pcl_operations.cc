#include "pcl_operations.hpp"

namespace PCLOperations
{
pcl::PointXYZ operator-(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
}

pcl::PointXYZ operator/(const pcl::PointXYZ &p, float d)
{
    if (d == 0)
        return p;

    return {p.x / d, p.y / d, p.z / d};
}

float norm(const pcl::PointXYZ &p)
{
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float operator*(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

pcl::PointXYZ operator*(const pcl::PointXYZ &p, float a)
{
    return {p.x * a, p.y * a, p.z * a};
}
pcl::PointXYZ operator*(float a, const pcl::PointXYZ &p)
{
    return {p.x * a, p.y * a, p.z * a};
}

pcl::PointXYZ operator+(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z};
}

bool operator==(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

pcl::PointXYZ cross_product(const pcl::PointXYZ &v_a, const pcl::PointXYZ &v_b)
{
    return {v_a.y * v_b.z - v_a.z * v_b.y, -(v_a.x * v_b.z - v_a.z * v_b.x),
            v_a.x * v_b.y - v_a.y * v_b.x};
}

// pcl::PointXYZ eigen3d_to_pcl_point(const Eigen::Vector3d &vec)
// {
//     return pcl::PointXYZ{vec.x(), vec.y(), vec.z()};
// }

// Eigen::Vector3d pcl_point_to_eigen3d(const pcl::PointXYZ &point)
// {
//     return Eigen::Vector3d{point.x, point.y, point.z};
// }

} // namespace PCLOperations
