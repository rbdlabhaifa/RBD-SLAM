#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
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
    const std::filesystem::path& destinations_file_path) {
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

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "USAGE: " << argv[0]
                  << " VOCABULARY_FILE_PATH CALIBRATION_FILE_PATH "
                     "MAP_FILE_PATH DRONE_DESTINATIONS_FILE_PATH"
                  << std::endl;
        return 1;
    }

    const std::vector<cv::Point3f> destinations =
        read_drone_destinations(std::filesystem::path(argv[4]));

    std::shared_ptr<Drone> drone = std::make_shared<Drone>();
    drone->activate_drone();

    Streamer streamer(drone);

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true,
                           true, argv[3]);

    Navigator navigator(drone, destinations, SLAM, streamer.get_frame_queue());
    streamer.start_stream();
    navigator.start_navigation();

    navigator.update_plane_of_flight();

    while (navigator.goto_next_destination())
        std::cout << "Reached destination!" << std::endl;

    if (!navigator.goto_the_unknown())
        std::cout << "Couldn't find a path to the unknown";

    std::this_thread::sleep_for(3min);

    std::cout << "Reached all destinations" << std::endl;

    return 0;
}
