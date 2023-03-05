#include "slam_utils.hpp"

#include <opencv2/core/core_c.h>

namespace SLAMUtils {
    cv::Point3f calc_aligned_point(const cv::Point3f& point,
                                   const cv::Mat& R_align,
                                   const cv::Mat& mu_align) {
        return {cv::Mat(R_align * (cv::Mat(point) - mu_align))};
    }

    std::pair<cv::Mat, cv::Mat> get_alignment_matrices(
        const std::vector<ORB_SLAM3::MapPoint*>& map_points) {
        cv::Mat mu_align1;
        cv::Mat R_align;

        std::vector<cv::Point3f> points;
        points.reserve(map_points.size());

        for (const auto& mp : map_points) {
            auto pos = mp->GetWorldPos();
            points.emplace_back(pos.x(), pos.y(), pos.z());
        }

        cv::reduce(points, mu_align1, 01, CV_REDUCE_AVG);

        cv::Point3f mu_align_pnt(mu_align1.at<float>(0), mu_align1.at<float>(1),
                                 mu_align1.at<float>(2));
        cv::Mat mu_align(mu_align_pnt);

        for (auto& p : points) {
            p = p - mu_align_pnt;
        }

        std::size_t nPoints = points.size();
        cv::Mat A(static_cast<int>(nPoints), 3, CV_32F);
        for (std::size_t i = 0; i < nPoints; i++) {
            A.at<float>(static_cast<int>(i), 0) = points[i].x;
            A.at<float>(static_cast<int>(i), 1) = points[i].y;
            A.at<float>(static_cast<int>(i), 2) = points[i].z;
        }

        cv::Mat w;
        cv::Mat u;
        cv::Mat vt;
        cv::SVDecomp(A.t(), w, u, vt);
        R_align = u.t();

        return {R_align, mu_align};
    }

    std::vector<Eigen::Matrix<double, 3, 1>> get_aligned_points_from_slam(
        std::vector<ORB_SLAM3::MapPoint*> map_points, const cv::Mat& R_align,
        const cv::Mat& mu_align) {
        std::vector<Eigen::Matrix<double, 3, 1>> points;

        map_points.erase(
            std::remove(map_points.begin(), map_points.end(), nullptr),
            map_points.end());

        std::transform(
            map_points.begin(), map_points.end(), std::back_inserter(points),
            [&](const auto& p) {
                const Eigen::Vector3f& v = p->GetWorldPos();

                return ORB_SLAM3::Converter::toVector3d(calc_aligned_point(
                    cv::Point3f(v.x(), v.y(), v.z()), R_align, mu_align));
            });

        return points;
    }

}  // namespace SLAMUtils
