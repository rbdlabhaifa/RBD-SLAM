#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "System.h"
#include "Viewer.h"
#include "drone.hpp"
#include "navigator.hpp"
#include "streamer.hpp"

using namespace std::chrono_literals;

std::filesystem::path
create_new_directory_named_current_time(std::string from_dir = "")
{
    if (from_dir != "" && from_dir.back() != '/')
        from_dir = from_dir + "/";

    std::time_t now = std::time(nullptr);
    std::tm time_info = *std::localtime(&now);

    char buffer[20]; // Buffer to store the formatted time

    // Format the time as "DD/MM/YY hh:mm:ss"
    std::strftime(buffer, sizeof(buffer), "%d.%m.%y %H:%M:%S", &time_info);

    std::string current_time(buffer);
    std::filesystem::path directory_named_time = from_dir + current_time;

    std::filesystem::create_directories(directory_named_time);
    return directory_named_time;
}

std::vector<cv::Point3f>
read_drone_destinations(const std::filesystem::path &destinations_file_path)
{
    std::vector<cv::Point3f> destinations;
    std::ifstream fin(destinations_file_path);

    std::cout << "Drone destinations: " << std::endl;

    std::vector<float> values;
    if (fin.good())
    {
        float value = 0;
        while (fin >> value)
        {
            values.push_back(value);
            if (values.size() == 3)
            {
                destinations.emplace_back(values[0], values[1], values[2]);
                std::cout << "dest: [" << values[0] << "," << values[1] << ","
                          << values[2] << "]" << std::endl;
                values.clear();
            }
        }
    }

    return destinations;
}

int main(int argc, char *argv[])
{

    std::ifstream programData("/home/ido/rbd/rbd-slam/RBD-SLAM/config.json");
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Main config:
    std::string vocabulary_path = data["Vocabulary_Path"];
    std::string calibration_path = data["calibration_path"];
    std::string map_path = data["map_path"];
    std::string data_save_dir = data["data_save_dir"];

    bool fake_drone = data["fake_drone"];
    bool use_webcam = data["use_webcam"];
    bool offline_mode = data["offline_mode"];

    // smi-magic numbers
    // goal_finder
    float find_exit_dist_scalar = data["find_exit_dist_scalar"];
    // rrt - path builder
    int rrt_path_size = data["rrt_path_size"];
    int rrt_ring_point_amount = data["rrt_ring_point_amount"];
    float rrt_jump_size = data["rrt_jump_size"];
    float rrt_ring_size_scalar = data["rrt_ring_size_scalar"];
    float rrt_goal_threshold = data["rrt_goal_threshold"];

    std::shared_ptr<SomeDrone> drone = std::make_shared<Drone>(!fake_drone);

    drone->activate_drone();

    Streamer streamer(use_webcam
                          ? std::nullopt
                          : std::optional<std::shared_ptr<SomeDrone>>{drone});

    const std::filesystem::path data_dir =
        create_new_directory_named_current_time(data_save_dir);

    Navigator navigator(drone, vocabulary_path, calibration_path, map_path,
                        streamer.get_frame_queue(), false, data_dir);
    std::cout << "Saving data to " << data_dir << std::endl;

    streamer.start_stream();
    navigator.start_navigation();

    while (true)
    {
        const auto path = navigator.get_path_to_the_unknown(
            find_exit_dist_scalar, rrt_goal_threshold, rrt_jump_size,
            rrt_ring_point_amount, rrt_ring_size_scalar, rrt_path_size);

        std::for_each(path.begin(), path.end(),
                      [&](const auto &p)
                      { navigator.goto_point(cv::Point3f(p.x, p.y, p.z)); });

        drone->send_command("down 20");
        std::this_thread::sleep_for(2s);
        navigator.reset_map_w_context();
        navigator.get_features_by_rotating();
    }

    std::cout << "Reached all destinations" << std::endl;

    return 0;
}
