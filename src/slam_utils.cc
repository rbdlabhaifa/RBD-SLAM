#include "slam_utils.hpp"

#include <opencv2/core/core_c.h>

#include <opencv2/core/eigen.hpp>

namespace SLAMUtils {
    cv::Point3f calc_aligned_point(const cv::Point3f& point,
                                   const cv::Mat& R_align,
                                   const cv::Mat& mu_align) {
        return {cv::Mat(R_align * (cv::Mat(point) - mu_align))};
    }

    cv::Mat calc_aligned_pose(const cv::Mat& pose, const cv::Mat& R_align,
                              const cv::Mat& mu_align) {
        cv::Mat Rcw = pose.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = pose.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();

        cv::Mat twc = -Rwc * tcw;

        cv::Mat aligned_Rwc = R_align * Rwc;
        cv::Mat aligned_twc = R_align * (twc - mu_align);
        cv::Mat align_pose = cv::Mat::eye(4, 4, CV_32F);

        cv::Mat aligned_Rcw = aligned_Rwc.t();
        cv::Mat aligned_tcw = -aligned_Rcw * aligned_twc;
        aligned_Rcw.copyTo(align_pose.rowRange(0, 3).colRange(0, 3));
        aligned_tcw.copyTo(align_pose.rowRange(0, 3).col(3));

        return align_pose;
    }

    cv::Mat sophus_to_cv(const Sophus::SE3f& pose) {
        cv::Mat pose_cv = cv::Mat::eye(4, 4, CV_32F);
        cv::Mat rotation_cv;
        cv::Mat translation_cv;

        cv::eigen2cv(pose.rotationMatrix(), rotation_cv);
        cv::eigen2cv(pose.translation(), translation_cv);

        rotation_cv.copyTo(pose_cv.rowRange(0, 3).colRange(0, 3));
        translation_cv.copyTo(pose_cv.rowRange(0, 3).col(3));

        return pose_cv;
    }

    cv::Point3f rotation_matrix_to_euler_angles(const cv::Mat& R) {
        float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) +
                        R.at<float>(1, 0) * R.at<float>(1, 0));

        bool singular = sy < 1e-6;  // If

        float x, y, z;
        if (!singular) {
            x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
            y = atan2(-R.at<float>(2, 0), sy);
            z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
        } else {
            x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
            y = atan2(-R.at<float>(2, 0), sy);
            z = 0;
        }
        return {static_cast<float>(x * 180 / CV_PI),
                static_cast<float>(y * 180 / CV_PI),
                static_cast<float>(z * 180 / CV_PI)};
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
