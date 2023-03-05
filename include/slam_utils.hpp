#ifndef SLAM_UTILS_H_
#define SLAM_UTILS_H_

#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <vector>

#include "MapPoint.h"

namespace SLAMUtils {
    cv::Point3f calc_aligned_point(const cv::Point3f& point,
                                   const cv::Mat& R_align,
                                   const cv::Mat& mu_align);
    std::vector<Eigen::Matrix<double, 3, 1>> get_aligned_points_from_slam(
        std::vector<ORB_SLAM3::MapPoint*> map_points, const cv::Mat& R_align,
        const cv::Mat& mu_align);
    std::pair<cv::Mat, cv::Mat> get_alignment_matrices(
        const std::vector<ORB_SLAM3::MapPoint*>& map_points);

}  // namespace SLAMUtils

#endif  // SLAM_UTILS_H_
