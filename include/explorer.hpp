#ifndef EXPLORER_H_
#define EXPLORER_H_

#include <pcl/common/io.h>

#include <vector>

class Explorer {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

   public:
    Explorer(const std::vector<Eigen::Matrix<double, 3, 1>>& map_points);

    void add_points_to_cloud(
        const std::vector<Eigen::Matrix<double, 3, 1>>& map_points);
};

#endif  // EXPLORER_H_
