#include "navigator.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/core_c.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <future>
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
#include <utility>
#include <vector>

#include "Converter.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include "auxilary.hpp"
#include "drone.hpp"
#include "explorer.hpp"
#include "slam_utils.hpp"

using namespace SLAMUtils;

Navigator::Navigator(
    std::shared_ptr<SomeDrone> drone, const std::string &vocabulary_file_path,
    const std::string &calibration_file_path, std::string map_file_path,
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>> &frame_queue,
    bool existing_map, std::filesystem::path data_dir,
    std::vector<cv::Point3f> destinations)
    : drone(std::move(drone)), destinations(std::move(destinations)),
      vocabulary_file_path(vocabulary_file_path),
      calibration_file_path(calibration_file_path),
      map_file_path(std::move(map_file_path)), frame_queue(frame_queue),
      existing_map(existing_map), data_dir(std::move(data_dir)),
      end_loop(existing_map)
{
    SLAM = std::make_unique<ORB_SLAM3::System>(vocabulary_file_path,
                                               calibration_file_path,
                                               ORB_SLAM3::System::MONOCULAR);
}

Navigator::~Navigator()
{
    close_navigation = true;
    update_pose_thread.join();
    SLAM->Shutdown();
}

void Navigator::update_pose()
{
    std::array<uchar, 640 * 480 * 3> current_frame{};

    auto start_time = std::chrono::steady_clock::now();
    while (!close_navigation)
    {
        auto timestamp = std::chrono::steady_clock::now();
        while (!frame_queue.pop(current_frame))
        {
            std::this_thread::sleep_for(20ms);
        }

        const auto sophus_pose = SLAM->TrackMonocular(
            cv::Mat(480, 640, CV_8UC3, current_frame.data()),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                timestamp - start_time)
                .count());

        const auto current_state = SLAM->GetTrackingState();

        if (!pose_updated && current_state == ORB_SLAM3::Tracking::OK)
        {
            if (!started_navigation)
            {
                if (end_loop)
                {
                    SLAM->ActivateLocalizationMode();
                }
                if (existing_map)
                {
                    SLAM->GetAtlas()->ChangeMap(
                        SLAM->GetAtlas()->GetAllMaps()[0]);
                }
            }

            drone->update_pose(sophus_to_cv(sophus_pose));
            if (!started_navigation)
            {
                std::lock_guard<std::mutex> lock(cv_m);
                started_navigation = true;
                pose_updated = true;

                cv.notify_one();
            }
            else
            {
                pose_updated = true;
            }
        }
    }
}

float Navigator::get_distance_to_destination(const cv::Point3f &p1,
                                             const cv::Point3f &p2)
{
    float x_diff = p1.x - p2.x;
    float y_diff = p1.y - p2.y;
    return x_diff * x_diff + y_diff * y_diff;
}

void Navigator::rotate_to_relocalize()
{
    bool need_to_stop = false;

    while (!pose_updated)
    {
        need_to_stop = true;

        drone->send_command("rc 0 0 0 0", false);
        std::this_thread::sleep_for(2s);

        drone->send_command("rc 0 0 0 15", false);
        std::this_thread::sleep_for(2s);
    }

    if (need_to_stop)
    {
        drone->send_command("rc 0 0 0 0", false);
        std::this_thread::sleep_for(2s);
    }
}

void Navigator::rotate_to_destination_angle(const cv::Point3f &location,
                                            const cv::Point3f &destination)
{
    // Get angle from Rwc
    cv::Point3f angles = rotation_matrix_to_euler_angles(Rwc);
    float angle1 = angles.z + 90;

    cv::Point2f vec1(destination.x - location.x, destination.y - location.y);
    // Get angle from vec1
    float angle2 = static_cast<float>(atan2(vec1.y, vec1.x) * 180 / M_PI);

    float ang_diff = angle1 - angle2;
    while (ang_diff > 180)
        ang_diff -= 360;
    while (ang_diff <= -180)
        ang_diff += 360;

    if (abs(ang_diff) > 8)
    {
        drone->send_command("rc 0 0 0 0", false);
        std::this_thread::sleep_for(1s);

        if (ang_diff > 0)
        {
            drone->send_command("cw " +
                                std::to_string(static_cast<int>(ang_diff) + 2));
        }
        else
        {
            drone->send_command(
                "ccw " + std::to_string(static_cast<int>(-ang_diff) + 2));
        }
        std::this_thread::sleep_for(2s);
        if (abs(ang_diff) > 100)
            std::this_thread::sleep_for(2s);

        pose_updated = false;
    }
}

cv::Point3f Navigator::get_last_location()
{
    if (!pose_updated)
    {
        std::this_thread::sleep_for(
            2s); // Sleep first if the pose isn't updated
    }

    if (!pose_updated)
    {
        std::cout << "Pose is not updated" << std::endl;

        rotate_to_relocalize();
    }

    cv::Mat aligned_pose =
        calc_aligned_pose(drone->get_pose(), R_align, mu_align);

    pose_updated = false;

    Rwc = aligned_pose.rowRange(0, 3).colRange(0, 3).t();
    const cv::Mat &tcw = aligned_pose.rowRange(0, 3).col(3);
    return {cv::Mat(-Rwc * tcw)};
}

bool Navigator::goto_next_destination()
{
    if (current_destination >= destinations.size())
        return false;
    goto_point(destinations[current_destination]);

    ++current_destination;
    return true;
}

void Navigator::goto_point(const cv::Point3f &p)
{
    cv::Point3f last_location;

    while (get_distance_to_destination(last_location = get_last_location(), p) >
           std::pow(0.15, 2))
    {
        rotate_to_destination_angle(last_location, p);

        std::this_thread::sleep_for(2s);
        if (pose_updated)
        {
            drone->send_command("rc 0 20 0 0", false);
            std::this_thread::sleep_for(2s);
            pose_updated = false;
        }
    }
    std::cout << "REACHED POINT!" << std::endl;

    drone->send_command("rc 0 0 0 0", false);
}

void Navigator::update_plane_of_flight()
{
    pose_updated = false;
    drone->send_command("left 50");
    const auto p1 = get_last_location();
    drone->send_command("right 100");
    std::this_thread::sleep_for(5s);
    pose_updated = false;
    const auto p2 = get_last_location();
    drone->send_command("forward 100");
    std::this_thread::sleep_for(5s);
    pose_updated = false;
    const auto p3 = get_last_location();
    pose_updated = false;

    Auxilary::save_points_to_file(std::vector<cv::Point3f>{p1, p2, p3},
                                  data_dir / "plane_points.xyz");

    explorer->set_plane_of_flight(pcl::PointXYZ(p1.x, p1.y, p1.z),
                                  pcl::PointXYZ(p2.x, p2.y, p2.z),
                                  pcl::PointXYZ(p3.x, p3.y, p3.z));
}

/* -------------------------------------------------------------------------- */
/*                                  UNUSED!!!                                 */
/* -------------------------------------------------------------------------- */
/* bool Navigator::goto_the_unknown()
{
    if (!explorer->is_set_plane_of_flight())
        update_plane_of_flight();

    cv::Point3f last_location = get_last_location();
    const std::vector<pcl::PointXYZ> path_to_the_unknown =
        explorer->get_points_to_unknown(
            pcl::PointXYZ(last_location.x, last_location.y,
            last_location.z));

    if (path_to_the_unknown.size() < 10)
        return false;

    Auxilary::save_points_to_file(
        path_to_the_unknown,
        data_dir / ("path" + std::to_string(++paths_created) + ".xyz"));

    std::for_each(path_to_the_unknown.begin(), path_to_the_unknown.end(),
                  [&](const auto &p)
                  { destinations.emplace_back(p.x, p.y, p.z); });

    while (goto_next_destination())
    {
        std::cout << "Reached a point in the path to the unknown!" <<
        std::endl;
    }

    return true;
} */

void Navigator::get_point_of_interest(
    const std::vector<Eigen::Matrix<double, 3, 1>> &points,
    std::promise<pcl::PointXYZ> pof_promise, std::size_t last_point,
    const cv::Point3f &last_location)
{
    auto map_filename = "current_map" + std::to_string(last_point) + ".xyz";
    auto start_filename = "start" + std::to_string(last_point) + ".xyz";

    Auxilary::save_points_to_file(points, map_filename);
    Auxilary::save_points_to_file({last_location}, start_filename);

    auto out_filename = "out" + std::to_string(last_point) + ".xyz";
    auto run_cmd = "python3 DelaunayGreedySearch.py " + map_filename + " " +
                   start_filename + " > " + out_filename;

    std::system(run_cmd.c_str());

    std::ifstream out_file(out_filename);
    std::string res_line;
    std::getline(out_file, res_line);

    size_t pos = 0;
    std::string token;
    pcl::PointXYZ point_of_interest;
    int index = 0;
    while ((pos = res_line.find(" ")) != std::string::npos)
    {
        token = res_line.substr(0, pos);
        float token_f = std::stof(token);
        if (index == 0)
            point_of_interest.x = token_f;
        else if (index == 1)
            point_of_interest.y = token_f;
        else if (index == 2)
            point_of_interest.z = token_f;
        ++index;
        res_line.erase(0, pos + 1);
    }

    pof_promise.set_value(point_of_interest);
}

void deleteFile(const std::string &filePath)
{
    std::string deleteCommand = "rm \"" + filePath + "\"";
    int deleteResult = system(deleteCommand.c_str());
    if (deleteResult != 0)
    {
        std::cerr << "Error deleting file: " << filePath << std::endl;
    }
}

void Navigator::reset_map_w_context()
{
    std::vector<ORB_SLAM3::MapPoint *> tracked_points =
        SLAM->GetTrackedMapPoints();
    SLAM->GetAtlas()->GetCurrentMap()->clear();

    ORB_SLAM3::Map *current_slam_map = SLAM->GetAtlas()->GetCurrentMap();
    for (ORB_SLAM3::MapPoint *map_point : tracked_points)
    {
        current_slam_map->AddMapPoint(map_point);
    }
}

std::vector<pcl::PointXYZ>
Navigator::get_path_to_the_unknown(std::size_t path_size)
{
    if (!explorer->is_set_plane_of_flight())
        return {};

    std::vector<pcl::PointXYZ> path_to_the_unknown;
    Eigen::Vector3d goal_exit_point;

    while (path_to_the_unknown.empty())
    {
        const auto aligned_points = get_aligned_points_from_slam(
            SLAM->GetAtlas()->GetCurrentMap()->GetAllMapPoints(), R_align,
            mu_align);
        explorer->set_cloud_points(aligned_points);

        pose_updated = false;
        const cv::Point3f last_location = get_last_location();
        std::cout << "last_location: " << last_location.x << " "
                  << last_location.y << " " << last_location.z << std::endl;

        // get exit point
        std::vector<std::vector<double>> transformed_vec =
            EigenOperations::vec_eigen_mat2vec_vec(aligned_points);

        Eigen::Vector3d vec_last_loc{last_location.x, last_location.y,
                                     last_location.z};

        // set known points
        auto [k_p1, k_p2, k_p3] = explorer->get_plane_of_flight();

        goal_exit_point = goal_finder::Find_Goal(transformed_vec, vec_last_loc,
                                                 k_p1, k_p2, k_p3);
        explorer->exit_point = pcl::PointXYZ(
            goal_exit_point[0], goal_exit_point[1], goal_exit_point[2]);
        // draw the exit point in pangolin

        std::promise<std::vector<pcl::PointXYZ>> path_promise;
        auto path_future = path_promise.get_future();

        std::thread get_path_to_unknown(
            [&](std::promise<std::vector<pcl::PointXYZ>> path_promise)
            {
                path_promise.set_value(
                    explorer->get_points_to_unknown(pcl::PointXYZ(
                        last_location.x, last_location.y, last_location.z)));
            },
            std::move(path_promise));

        do
        {
            drone->send_command("rc 0 0 0 0", false);
        } while (path_future.wait_for(2s) != std::future_status::ready);

        path_to_the_unknown = path_future.get();
        get_path_to_unknown.join();
    }

    Auxilary::save_points_to_file(
        path_to_the_unknown,
        data_dir / ("path" + std::to_string(++paths_created) + ".xyz"));

    Auxilary::save_points_to_file(
        get_aligned_points_from_slam(
            SLAM->GetAtlas()->GetCurrentMap()->GetAllMapPoints(), R_align,
            mu_align),
        data_dir / ("map_for_path" + std::to_string(paths_created) + ".xyz"));

    Auxilary::save_points_to_file(
        path_to_the_unknown,
        data_dir / ("path" + std::to_string(paths_created) + ".xyz"));

    std::vector<pcl::PointXYZ> save_exit = {{
        static_cast<float>(goal_exit_point(0)),
        static_cast<float>(goal_exit_point(1)),
        static_cast<float>(goal_exit_point(2)),
    }};

    Auxilary::save_points_to_file(
        save_exit,
        data_dir / ("exit" + std::to_string(paths_created) + ".xyz"));

    const std::string treeFileName = "tree.xyz";
    const std::string startFileName = "start.xyz";
    const std::string endFileName = "end.xyz";
    const std::string pathsFileName = "paths.csv";
    const std::string initial_path_NAME = "initial_path.xyz";

    // Get the current directory
    char currentDir[FILENAME_MAX];
    if (!getcwd(currentDir, sizeof(currentDir)))
    {
        std::cerr << "Error getting current directory!" << std::endl;
    }

    // Get the parent directory
    std::string parentDir = currentDir;
    size_t pos = parentDir.find_last_of("/\\");
    if (pos != std::string::npos)
    {
        parentDir = parentDir.substr(0, pos);
    }

    std::cout << "data_dir is:" << data_dir.string() << "------------"
              << std::endl;

    // Copy the files to the current directory
    std::string mvTreeCommand =
        "mv \"" + treeFileName + "\" \"" + std::string(data_dir) + "/" +
        std::to_string(paths_created) + "_" + treeFileName + "\"";
    std::string mvStartCommand =
        "mv \"" + startFileName + "\" \"" + std::string(data_dir) + "/" +
        std::to_string(paths_created) + "_" + startFileName + "\"";
    std::string mvEndCommand =
        "mv \"" + endFileName + "\" \"" + std::string(data_dir) + "/" +
        std::to_string(paths_created) + "_" + endFileName + "\"";
    std::string mvPathsCommand =
        "mv \"" + pathsFileName + "\" \"" + std::string(data_dir) + "/" +
        std::to_string(paths_created) + "_" + pathsFileName + "\"";
    std::string mvInitialPathsCommand =
        "mv \"" + initial_path_NAME + "\" \"" + std::string(data_dir) + "/" +
        std::to_string(paths_created) + "_" + initial_path_NAME + "\"";

    int treemvResult = system(mvTreeCommand.c_str());
    int startmvResult = system(mvStartCommand.c_str());
    int endmvResult = system(mvEndCommand.c_str());
    int pathsmvResult = system(mvPathsCommand.c_str());
    int initialPathsmvResult = system(mvInitialPathsCommand.c_str());

    if (treemvResult || startmvResult || endmvResult || pathsmvResult ||
        initialPathsmvResult)
    {
        std::cout << "Files copied successfully!" << std::endl;
    }
    else
    {
        std::cerr << "Error copying files!" << std::endl;
    }

    std::cout << "should copied the files" << std::endl;
    // ---------------------------------------------------------------------------

    std::cout << "GOT PATH" << std::endl;

    if (path_to_the_unknown.size() > path_size)
    {
        path_to_the_unknown.resize(path_size);
    }

    return path_to_the_unknown;
}

void Navigator::get_features_by_rotating()
{
    const int degrees_to_rotate = 30;
    const int times_rotate = 360 / degrees_to_rotate;
    int current_rotate = 0;
    int multiplier = 1;

    while (current_rotate < (times_rotate))
    {
        ++current_rotate;

        if ((times_rotate - current_rotate) % 5 == 0)
            std::cout << "scan movments left: "
                      << (times_rotate - current_rotate) << std::endl;

        drone->send_command("rc 0 0 " + std::to_string(multiplier * 18) + " 15",
                            false);

        multiplier *= -1;
        std::this_thread::sleep_for(5s);
        drone->send_command("rc 0 0 0 0", false);
        std::this_thread::sleep_for(1s);
        pose_updated = false;
        std::this_thread::sleep_for(1s);

        if (!pose_updated)
        {
            int back_times = 0;

            while (!pose_updated && back_times < current_rotate)
            {
                ++back_times;
                drone->send_command(
                    "rc 0 0 " + std::to_string(multiplier * 18) + " -15",
                    false);
                multiplier *= -1;
                std::this_thread::sleep_for(5s);
                drone->send_command("rc 0 0 0 0", false);
                std::this_thread::sleep_for(2s);
                pose_updated = false;
                std::this_thread::sleep_for(1s);
            }

            current_rotate -= back_times;
        }
    }

    pose_updated = false;
}

/**
 * @brief start navigation, initiate
 *
 * @param use_explorer set to true
 * @note !! only runs once !!
 */
void Navigator::start_navigation(bool use_explorer)
{
    update_pose_thread = std::thread(&Navigator::update_pose, this);

    std::this_thread::sleep_for(10s);
    drone->send_command("takeoff");
    std::this_thread::sleep_for(2s);
    drone->send_command("speed 40");
    std::this_thread::sleep_for(2s);
    drone->send_command("up 60");
    std::this_thread::sleep_for(2s);
    if (existing_map)
    {
        drone->send_command("rc -10 0 0 -15", false);
        std::this_thread::sleep_for(3s);
        drone->send_command("rc 0 0 0 0", false);
    }
    else
    {
        get_features_by_rotating();
        end_loop = true;
    }

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk, [&] { return started_navigation.load(); });
    }

    const auto map_points =
        SLAM->GetAtlas()->GetCurrentMap()->GetAllMapPoints();
    const auto [R_align, mu_align] = get_alignment_matrices(map_points);
    this->R_align = R_align;
    this->mu_align = mu_align;

    const auto slam_points =
        get_aligned_points_from_slam(map_points, R_align, mu_align);

    // save to file and read from it for exit algo
    Auxilary::save_points_to_file(slam_points, data_dir / "aligned_points.xyz");

    if (use_explorer)
        explorer = std::make_shared<Explorer>(slam_points);

    std::transform(
        destinations.begin(), destinations.end(), destinations.begin(),
        [&](const auto &p)
        { return calc_aligned_point(p, this->R_align, this->mu_align); });

    Auxilary::save_points_to_file(destinations,
                                  data_dir / "aligned_destinations.xyz");

    if (use_explorer)
        update_plane_of_flight();
}
