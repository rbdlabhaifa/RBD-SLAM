#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
#include <vector>

#include "System.h"
#include "drone.hpp"
#include "explorer.hpp"

class Navigator {
    std::shared_ptr<Drone> drone;
    std::vector<cv::Point3f> destinations;
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>& frame_queue;
    ORB_SLAM2::System& SLAM;
    std::filesystem::path data_dir;

    std::size_t current_destination = 0;

    std::size_t paths_created = 0;

    cv::Mat R_align, mu_align;
    cv::Mat Rwc;

    std::atomic_bool pose_updated;
    std::atomic_bool close_navigation;

    std::thread update_pose_thread;

    void align_pose();

    /**
     * @brief Attempt to get the last location relative to the
     * SLAM map, rotating the drone if not localized
     * @returns drone's last location relative to the SLAM map
     */
    cv::Point3f get_last_location();

    std::vector<Eigen::Matrix<double, 3, 1>> get_points_from_slam();

    static float get_distance_to_destination(const cv::Point3f& p1,
                                             const cv::Point3f& p2);
    static cv::Mat calc_aligned_pose(const cv::Mat& pose,
                                     const cv::Mat& R_align,
                                     const cv::Mat& mu_align);
    static cv::Point3f calc_aligned_point(const cv::Point3f& point,
                                          const cv::Mat& R_align,
                                          const cv::Mat& mu_align);
    static cv::Point3f rotation_matrix_to_euler_angles(const cv::Mat& R);

    void update_pose();

    /**
     * @brief Rotate the drone until it's caught features
     */
    void rotate_to_relocalize();
    void rotate_to_destination_angle(const cv::Point3f& location,
                                     const cv::Point3f& destination);

   public:
    std::shared_ptr<Explorer> explorer;

    Navigator(std::shared_ptr<Drone>,
              const std::vector<cv::Point3f>& destinations,
              ORB_SLAM2::System& SLAM,
              boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>&
                  frame_queue,
              const std::filesystem::path& data_dir = ".",
              bool use_explorer = true);
    ~Navigator();

    void start_navigation();
    bool goto_next_destination();
    bool goto_the_unknown();
    void update_plane_of_flight();
};

#endif  // NAVIGATOR_H_
