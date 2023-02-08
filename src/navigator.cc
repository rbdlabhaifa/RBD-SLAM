#include "navigator.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/core_c.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>
#include <thread>
#include <vector>

#include "Converter.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include "auxilary.hpp"
#include "drone.hpp"
#include "explorer.hpp"

Navigator::Navigator(
    std::shared_ptr<SomeDrone> drone,
    const std::vector<cv::Point3f>& destinations,
    const std::string& vocabulary_file_path,
    const std::string& calibration_file_path, const std::string& map_file_path,
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>& frame_queue,
    const std::filesystem::path& data_dir)
    : drone(drone),
      destinations(destinations),
      vocabulary_file_path(vocabulary_file_path),
      calibration_file_path(calibration_file_path),
      map_file_path(map_file_path),
      frame_queue(frame_queue),
      pose_updated(false),
      started_navigation(false),
      data_dir(data_dir),
      close_navigation(false) {
    SLAM = std::make_unique<ORB_SLAM3::System>(vocabulary_file_path,
                                               calibration_file_path,
                                               ORB_SLAM3::System::MONOCULAR);
}

Navigator::~Navigator() {
    close_navigation = true;
    update_pose_thread.join();
    SLAM->Shutdown();
}

std::vector<Eigen::Matrix<double, 3, 1>> Navigator::get_points_from_slam() {
    std::vector<Eigen::Matrix<double, 3, 1>> points;
    std::vector<ORB_SLAM3::MapPoint*> map_points =
        SLAM->GetAtlas()
            ->GetCurrentMap()
            ->GetAllMapPoints();  // TODO: should we take all the maps?

    map_points.erase(std::remove(map_points.begin(), map_points.end(), nullptr),
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

std::pair<cv::Mat, cv::Mat> Navigator::get_alignment_matrices(
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
    cv::Mat A((int)nPoints, 3, CV_32F);
    for (std::size_t i = 0; i < nPoints; i++) {
        A.at<float>(i, 0) = points[i].x;
        A.at<float>(i, 1) = points[i].y;
        A.at<float>(i, 2) = points[i].z;
    }

    cv::Mat w, u, vt;
    cv::SVDecomp(A.t(), w, u, vt);
    R_align = u.t();

    return std::make_pair(R_align, mu_align);
}

cv::Point3f Navigator::calc_aligned_point(const cv::Point3f& point,
                                          const cv::Mat& R_align,
                                          const cv::Mat& mu_align) {
    return cv::Point3f(cv::Mat(R_align * (cv::Mat(point) - mu_align)));
}

cv::Mat Navigator::calc_aligned_pose(const cv::Mat& pose,
                                     const cv::Mat& R_align,
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

cv::Point3f Navigator::rotation_matrix_to_euler_angles(const cv::Mat& R) {
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
    return cv::Point3f(x * 180 / CV_PI, y * 180 / CV_PI, z * 180 / CV_PI);
}

cv::Mat Navigator::sophus_to_cv(const Sophus::SE3f& pose) {
    cv::Mat pose_cv = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat rotation_cv, translation_cv;

    cv::eigen2cv(pose.rotationMatrix(), rotation_cv);
    cv::eigen2cv(pose.translation(), translation_cv);

    rotation_cv.copyTo(pose_cv.rowRange(0, 3).colRange(0, 3));
    translation_cv.copyTo(pose_cv.rowRange(0, 3).col(3));

    return pose_cv;
}

void Navigator::update_pose() {
    bool localization_activated = false;
    double frame_cnt = 0;
    std::array<uchar, 640 * 480 * 3> current_frame;

    auto start_time = std::chrono::steady_clock::now();
    while (!close_navigation) {
        auto timestamp = std::chrono::steady_clock::now();
        while (!frame_queue.pop(current_frame)) {
            std::this_thread::sleep_for(20ms);
        }

        const auto sophus_pose = SLAM->TrackMonocular(
            cv::Mat(480, 640, CV_8UC3, current_frame.data()),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                timestamp - start_time)
                .count());
        const cv::Mat current_pose = sophus_to_cv(sophus_pose);

        // if (!localization_activated) {
        //     SLAM.ActivateLocalizationMode();
        //     localization_activated = true;
        // }

        if (!pose_updated &&
            SLAM->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            // if (!started_navigation &&
            //     SLAM->GetAtlas()->GetAllMaps().size() > 1)
            //     continue;

            SLAM->ActivateLocalizationMode();
            if (!started_navigation)
                SLAM->GetAtlas()->ChangeMap(SLAM->GetAtlas()->GetAllMaps()[0]);

            drone->update_pose(current_pose);
            if (!started_navigation) {
                std::lock_guard<std::mutex> lock(cv_m);
                started_navigation = true;
                pose_updated = true;

                cv.notify_one();
            } else {
                pose_updated = true;
            }
        }
    }
}

float Navigator::get_distance_to_destination(const cv::Point3f& p1,
                                             const cv::Point3f& p2) {
    float x_diff = p1.x - p2.x;
    float y_diff = p1.y - p2.y;
    return x_diff * x_diff + y_diff * y_diff;
}

void Navigator::rotate_to_relocalize() {
    drone->send_command("rc 0 0 0 0", false);
    std::this_thread::sleep_for(1s);
    while (!pose_updated) {
        drone->send_command("cw 30");

        std::this_thread::sleep_for(3s);
    }
}

void Navigator::rotate_to_destination_angle(const cv::Point3f& location,
                                            const cv::Point3f& destination) {
    // Get angle from Rwc
    cv::Point3f angles = rotation_matrix_to_euler_angles(Rwc);
    float angle1 = angles.z + 90;

    cv::Point2f vec1(destination.x - location.x, destination.y - location.y);
    // Get angle from vec1
    float angle2 = std::atan2(vec1.y, vec1.x) * 180 / M_PI;

    float ang_diff = angle1 - angle2;
    while (ang_diff > 180) ang_diff -= 360;
    while (ang_diff <= -180) ang_diff += 360;

    if (abs(ang_diff) > 8) {
        drone->send_command("rc 0 0 0 0", false);
        std::this_thread::sleep_for(1s);
        if (ang_diff > 0)
            drone->send_command("cw " + std::to_string((int)ang_diff + 2));
        else
            drone->send_command("ccw " + std::to_string((int)-ang_diff + 2));
        std::this_thread::sleep_for(2s);
        if (abs(ang_diff) > 100) std::this_thread::sleep_for(2s);
        pose_updated = false;  // TODO: maybe remove?
    }
}

cv::Point3f Navigator::get_last_location() {
    if (!pose_updated) rotate_to_relocalize();

    cv::Mat aligned_pose =
        calc_aligned_pose(drone->get_pose(), R_align, mu_align);

    pose_updated = false;

    Rwc = aligned_pose.rowRange(0, 3).colRange(0, 3).t();
    const cv::Mat& tcw = aligned_pose.rowRange(0, 3).col(3);
    if (!save_pose) {
        Auxilary::save_points_to_file(
            std::vector<cv::Point3f>{cv::Point3f(cv::Mat(-Rwc * tcw))},
            data_dir / "pose.xyz");

        save_pose = true;
    }
    return cv::Point3f(cv::Mat(-Rwc * tcw));
}

bool Navigator::goto_next_destination() {
    if (current_destination >= destinations.size()) return false;
    const cv::Point3f& current_dest = destinations[current_destination];
    cv::Point3f last_location;

    while (get_distance_to_destination(last_location = get_last_location(),
                                       current_dest) > std::pow(0.15, 2)) {
        rotate_to_destination_angle(last_location, current_dest);
        std::this_thread::sleep_for(2s);
        if (pose_updated) {
            drone->send_command("rc 0 30 0 0", false);
            pose_updated = false;
            std::this_thread::sleep_for(1s);
        }
    }

    drone->send_command("rc 0 0 0 0", false);
    ++current_destination;
    return true;
}

void Navigator::update_plane_of_flight() {
    // TODO: some dance to dynamically update the plane of flight

    pcl::PointXYZ p1(destinations[0].x, destinations[0].y, destinations[0].z);
    pcl::PointXYZ p2(destinations[1].x, destinations[1].y, destinations[1].z);
    pcl::PointXYZ p3(destinations[2].x, destinations[2].y, destinations[2].z);
    explorer->set_plane_of_flight(p1, p2, p3);
}

bool Navigator::goto_the_unknown() {
    if (!explorer->is_set_plane_of_flight()) update_plane_of_flight();

    cv::Point3f last_location = get_last_location();
    const std::vector<pcl::PointXYZ> path_to_the_unknown =
        explorer->get_points_to_unknown(
            pcl::PointXYZ(last_location.x, last_location.y, last_location.z));

    if (path_to_the_unknown.size() < 10) return false;

    Auxilary::save_path_to_file(
        path_to_the_unknown,
        data_dir / ("path" + std::to_string(++paths_created) + ".xyz"));

    std::for_each(
        path_to_the_unknown.begin(), path_to_the_unknown.end(),
        [&](const auto& p) { destinations.emplace_back(p.x, p.y, p.z); });

    while (goto_next_destination())
        std::cout << "Reached a point in the path to the unknown!" << std::endl;

    return true;
}

void Navigator::start_navigation(bool use_explorer) {
    update_pose_thread = std::thread(&Navigator::update_pose, this);

    std::this_thread::sleep_for(10s);
    drone->send_command("takeoff");
    drone->send_command("up 55");
    drone->send_command("rc -10 0 0 -15", false);
    std::this_thread::sleep_for(3s);
    drone->send_command("rc 0 0 0 0", false);

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk, [&] { return started_navigation == true; });
    }

    auto [R_align, mu_align] = get_alignment_matrices(
        SLAM->GetAtlas()
            ->GetCurrentMap()  // TODO: should we take all the maps?
            ->GetAllMapPoints());
    this->R_align = R_align;
    this->mu_align = mu_align;

    const auto slam_points = get_points_from_slam();
    if (use_explorer) explorer = std::make_shared<Explorer>(slam_points);

    Auxilary::save_points_to_file(slam_points, data_dir / "aligned_points.xyz");

    std::transform(destinations.begin(), destinations.end(),
                   destinations.begin(), [&](const auto& p) {
                       return calc_aligned_point(p, this->R_align,
                                                 this->mu_align);
                   });

    Auxilary::save_points_to_file(destinations,
                                  data_dir / "aligned_destinations.xyz");

    update_plane_of_flight();
}
