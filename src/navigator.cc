#include "navigator.hpp"

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
#include <vector>

#include "drone.hpp"
#include "explorer.hpp"

Navigator::Navigator(
    std::shared_ptr<Drone> drone, const std::vector<cv::Point3f>& destinations,
    ORB_SLAM2::System& SLAM,
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>& frame_queue,
    bool use_explorer)
    : drone(drone),
      destinations(destinations),
      SLAM(SLAM),
      frame_queue(frame_queue),
      pose_updated(false),
      close_navigation(false) {
    if (use_explorer)
        explorer = std::make_shared<Explorer>(get_points_from_slam());
}

Navigator::~Navigator() {
    close_navigation = true;
    update_pose_thread.join();
}

std::vector<Eigen::Matrix<double, 3, 1>> Navigator::get_points_from_slam() {
    std::vector<Eigen::Matrix<double, 3, 1>> points;
    std::vector<ORB_SLAM2::MapPoint*> map_points =
        SLAM.GetMap()->GetAllMapPoints();

    map_points.erase(std::remove(map_points.begin(), map_points.end(), nullptr),
                     map_points.end());

    std::transform(
        map_points.begin(), map_points.end(), std::back_inserter(points),
        [](auto p) {
            return ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
        });

    return points;
}

void Navigator::align_destinations() {
    for (cv::Point3f& d : destinations) {
        d = cv::Point3f(cv::Mat(R_align * (cv::Mat(d) - mu_align)));
    }
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

void Navigator::update_pose() {
    double frame_cnt = 0;
    std::array<uchar, 640 * 480 * 3> current_frame;
    while (!close_navigation) {
        while (!frame_queue.pop(current_frame)) {
            std::this_thread::sleep_for(10ms);
            continue;
        }

        ++frame_cnt;
        if (!pose_updated) {
            drone->update_pose(SLAM.TrackMonocular(
                cv::Mat(480, 640, CV_8UC3, current_frame.data()), frame_cnt));
            pose_updated = !drone->get_pose().empty();
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
    float angle1 = angles.z;

    cv::Point2f vec1(destination.x - location.x, destination.y - location.y);
    // Get angle from vec1
    float angle2 = (std::atan2(vec1.y, vec1.x) * 180 / M_PI) + 90;

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
    return cv::Point3f(cv::Mat(-Rwc * tcw));
}

bool Navigator::goto_next_destination() {
    if (current_destination >= destinations.size()) return false;
    const cv::Point3f& current_dest = destinations[current_destination];
    cv::Point3f last_location;

    while (get_distance_to_destination(last_location = get_last_location(),
                                       current_dest) > std::pow(0.15, 2)) {
        rotate_to_destination_angle(last_location, current_dest);
        drone->send_command("rc 0 30 0 0");
        pose_updated = false;
    }

    drone->send_command("rc 0 0 0 0", false);
    return true;
}

void Navigator::start_navigation() {
    // drone->send_command("takeoff");
    // drone->send_command("up 55");

    auto [R_align, mu_align] = SLAM.GetMap()->align_map();
    this->R_align = R_align;
    this->mu_align = mu_align;

    align_destinations();

    update_pose_thread = std::thread(&Navigator::update_pose, this);
}
