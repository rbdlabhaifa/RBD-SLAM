#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <boost/lockfree/spsc_queue.hpp>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <thread>
#include <vector>

#include "Drone.h"
#include "System.h"

class Navigator {
    Drone& drone;
    std::vector<cv::Point3f> destinations;
    boost::lockfree::spsc_queue<std::vector<uchar>>& frame_queue;
    ORB_SLAM2::System& SLAM;
    std::size_t current_destination = 0;
    cv::Mat aligned_pose;
    cv::Mat Rwc;
    bool pose_updated = false;
    bool close_navigation = false;

    std::thread update_pose_thread;
    std::thread align_destinations_and_pose_thread;

    void start_slam();
    void align_destinations_and_pose();
    float get_distance_to_destination(const cv::Point3f& p1,
                                      const cv::Point3f& p2);
    cv::Point3f get_last_location(bool check_update = true);
    cv::Mat calc_aligned_pose(const cv::Mat& pose, const cv::Mat& R_align,
                              const cv::Mat& mu_align);
    cv::Point3f rotation_matrix_to_euler_angles(const cv::Mat& R);
    void update_pose();

    void rotate_to_relocalize();
    void rotate_to_destination_angle(const cv::Point3f& destination);

   public:
    Navigator(Drone& drone, const std::vector<cv::Point3f>& destinations,
              ORB_SLAM2::System& SLAM,
              boost::lockfree::spsc_queue<std::vector<uchar>>& frame_queue);
    ~Navigator();
    void start_navigation();
    bool goto_next_destination();
};

#endif  // NAVIGATOR_H_
