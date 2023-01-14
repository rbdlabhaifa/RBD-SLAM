#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/core/types.hpp>
#include <thread>
#include <vector>

#include "System.h"
#include "drone.hpp"
#include "navigator.hpp"
#include "streamer.hpp"

using namespace std::chrono_literals;

std::vector<cv::Point3f> read_drone_destinations(
    const std::string &destinations_file_path) {
    std::vector<cv::Point3f> destinations;
    std::ifstream fin(destinations_file_path);

    std::cout << "Drone destinations: " << std::endl;

    std::vector<float> values;
    if (fin.good()) {
        std::string param_name;
        float value = 0;
        while (fin >> value) {
            values.push_back(value);
            if (values.size() == 3) {
                destinations.emplace_back(values[0], values[1], values[2]);
                std::cout << "dest: [" << values[0] << "," << values[1] << ","
                          << values[2] << "]" << std::endl;
                values.clear();
            }
        }
    }

    return destinations;
}

int main(int argc, char *argv[]) {
    std::vector<cv::Point3f> destinations =
        read_drone_destinations("drone_destinations.txt");

    std::shared_ptr<Drone> drone = std::make_shared<Drone>(argv[2], "");
    drone->activate_drone();

    Streamer streamer(drone);

    std::string vocabulary_path = "Configuration/Vocabulary/ORBvoc.txt";
    ORB_SLAM2::System SLAM(vocabulary_path, argv[1],
                           ORB_SLAM2::System::MONOCULAR, true, true,
                           "Slam_latest_Map.bin");

    Navigator navigator(drone, destinations, SLAM, streamer.get_frame_queue());
    streamer.start_stream();
    navigator.start_navigation();

    // while (navigator.goto_next_destination())
    //     std::cout << "Reached destination!" << std::endl;
    std::this_thread::sleep_for(3min);

    std::cout << "Reached all destinations" << std::endl;

    return 0;
}
