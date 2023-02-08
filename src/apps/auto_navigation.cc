#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/core/types.hpp>
#include <thread>
#include <vector>

#include "System.h"
#include "Viewer.h"
#include "drone.hpp"
#include "navigator.hpp"
#include "streamer.hpp"

using namespace std::chrono_literals;

std::filesystem::path create_new_directory_named_current_time() {
    // TODO: Maybe change the time format
    time_t now = time(0);
    std::string current_time = std::string(ctime(&now));
    current_time.pop_back();
    const std::filesystem::path directory_named_time = current_time;

    std::filesystem::create_directories(directory_named_time);
    return directory_named_time;
}

std::vector<cv::Point3f> read_drone_destinations(
    const std::filesystem::path& destinations_file_path) {
    std::vector<cv::Point3f> destinations;
    std::ifstream fin(destinations_file_path);

    std::cout << "Drone destinations: " << std::endl;

    std::vector<float> values;
    if (fin.good()) {
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

bool find_arg(int argc, char* const argv[], const std::string& arg) {
    for (int i = 0; i < argc; ++i) {
        if (std::string(argv[i]) == arg) return true;
    }

    return false;
}

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "USAGE: " << argv[0]
                  << " VOCABULARY_FILE_PATH CALIBRATION_FILE_PATH "
                     "MAP_FILE_PATH DRONE_DESTINATIONS_FILE_PATH "
                     "[--use-webcam] [--fake-drone]"
                  << std::endl;
        return 1;
    }

    const bool use_webcam = find_arg(argc, argv, "--use-webcam");
    const bool fake_drone = find_arg(argc, argv, "--fake-drone");

    const std::vector<cv::Point3f> destinations =
        read_drone_destinations(std::filesystem::path(argv[4]));

    std::shared_ptr<SomeDrone> drone =
        use_webcam ? std::make_shared<SomeDrone>()
                   : std::make_shared<Drone>(!fake_drone);
    drone->activate_drone();

    Streamer streamer(use_webcam ? nullptr : drone);

    const std::filesystem::path data_dir =
        create_new_directory_named_current_time();
    Navigator navigator(drone, destinations, argv[1], argv[2], argv[3],
                        streamer.get_frame_queue(), data_dir);
    std::cout << "Saving data to " << data_dir << std::endl;

    streamer.start_stream();
    navigator.start_navigation();

    // navigator.update_plane_of_flight();

    while (navigator.goto_next_destination())
        std::cout << "Reached destination!" << std::endl;

    if (!navigator.goto_the_unknown())
        std::cout << "Couldn't find a path to the unknown";

    std::cout << "Reached all destinations" << std::endl;

    return 0;
}
