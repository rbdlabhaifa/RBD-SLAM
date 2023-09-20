#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <condition_variable>
#include <cstddef>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "MapPoint.h"
#include "System.h"
#include "Viewer.h"
#include "drone.hpp"
#include "eigen_operations.hpp"
#include "explorer.hpp"
#include "goal_finder.hpp"
#include "sophus/se3.hpp"

#define DEFAULT_MAX_PATH_SIZE 20

class Navigator
{
    std::vector<cv::Point3f> destinations;
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>> &frame_queue;

    std::unique_ptr<ORB_SLAM3::System> SLAM;
    std::string vocabulary_file_path;
    std::string calibration_file_path;
    std::string map_file_path;

    /// Path to save the entire navigator's data
    std::filesystem::path data_dir;

    std::size_t current_destination = 0;
    std::size_t current_dest_point = 0;

    std::size_t paths_created = 0;

    /// Matrices of ORB_SLAM3 alignments
    cv::Mat R_align, mu_align;

    /// Drone's rotation matrix
    cv::Mat Rwc;

    /// reader for xyz point reading in "start_navigation"
    goal_finder::DataReader _map_reader;

    std::atomic_bool started_navigation = {false};

    /**
     * An atomic boolean assuring we don't read and update the pose at the same
     * time. Pose is updated when it's true
     */
    std::atomic_bool pose_updated = {false};

    std::atomic_bool close_navigation = {false};

    /**
     * An atomic boolean assuring we end the loop to get features before
     * exploring
     */
    std::atomic_bool end_loop;

    bool existing_map;

    std::thread update_pose_thread;

    /// These are used for waiting on end_loop
    std::condition_variable cv;
    std::mutex cv_m;

    /**
     * @brief Attempt to get the last location relative to the
     * SLAM map, rotating the drone if not localized
     * @returns drone's last location relative to the SLAM map
     */
    cv::Point3f get_last_location();

    static float get_distance_to_destination(const cv::Point3f &p1,
                                             const cv::Point3f &p2);

    /**
     * @brief Runs on a thread constantly to update the drone's pose
     */
    void update_pose();

    /**
     * @brief Rotate the drone until it's relocalized
     */
    void rotate_to_relocalize();
    bool rotate_to_destination_angle(const cv::Point3f &location,
                                     const cv::Point3f &destination);

    int scan_num = 0;
    const float dist_scalar_0 = 1.58;
    const float dist_scalar_1 = 1.25;

  public:
    std::shared_ptr<Explorer> explorer;
    std::shared_ptr<SomeDrone> drone;

    Navigator(const Navigator &) = delete;
    Navigator(Navigator &&) = delete;
    Navigator &operator=(const Navigator &) = delete;
    Navigator &operator=(Navigator &&) = delete;
    Navigator(std::shared_ptr<SomeDrone>,
              const std::string &vocabulary_file_path,
              const std::string &calibration_file_path,
              std::string map_file_path,
              boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>
                  &frame_queue,
              bool existing_map, std::filesystem::path data_dir = ".",
              std::vector<cv::Point3f> destinations = {});
    ~Navigator();

    void reset_map_w_context();

    void start_navigation(bool use_explorer = true);

    /**
     * @brief Rotate the drone with some translation in the z axis to obtain
     * features when reaching an unknown area
     */
    void get_features_by_rotating();

    // bool goto_the_unknown(); /* UNUSED */
    bool goto_next_destination();
    std::vector<pcl::PointXYZ>
    get_path_to_the_unknown(std::size_t path_size = DEFAULT_MAX_PATH_SIZE);
    void goto_point(const cv::Point3f &p);

    /**
     * @brief Update the plane of flight by moving the drone in 2 straight lines
     */
    void update_plane_of_flight();
};

#endif // NAVIGATOR_H_
