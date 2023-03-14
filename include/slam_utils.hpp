#ifndef SLAM_UTILS_H_
#define SLAM_UTILS_H_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

#include "MapPoint.h"

namespace SLAMUtils {
    cv::Point3f calc_aligned_point(const cv::Point3f& point,
                                   const cv::Mat& R_align,
                                   const cv::Mat& mu_align);

    cv::Point3f rotation_matrix_to_euler_angles(const cv::Mat& R);

    cv::Mat calc_aligned_pose(const cv::Mat& pose, const cv::Mat& R_align,
                              const cv::Mat& mu_align);

    std::vector<Eigen::Matrix<double, 3, 1>> get_aligned_points_from_slam(
        std::vector<ORB_SLAM3::MapPoint*> map_points, const cv::Mat& R_align,
        const cv::Mat& mu_align);

    std::pair<cv::Mat, cv::Mat> get_alignment_matrices(
        const std::vector<ORB_SLAM3::MapPoint*>& map_points);

    /**
     * @brief Convert a Sophus Pose to an OpenCV matrix
     */
    cv::Mat sophus_to_cv(const Sophus::SE3f& pose);

}  // namespace SLAMUtils

#endif  // SLAM_UTILS_H_
