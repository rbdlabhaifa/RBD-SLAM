#include <Converter.h>
#include <Eigen/src/Core/Matrix.h>
#include <Optimizer.h>
#include <System.h>
#include <Viewer.h>
#include <bits/types/time_t.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui_c.h>
#include <signal.h>

#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <thread>
#include <vector>

#include "MapPoint.h"
#include "drone.hpp"
#include "streamer.hpp"

#define DESTINATIONS_FILE_NAME "drone_destinations.txt"

using namespace std::chrono_literals;

bool continue_session = false;

void exit_session(int s) { continue_session = false; }

std::filesystem::path create_new_directory_named_current_time() {
    // TODO: Maybe change the time format
    time_t now = time(0);
    std::string current_time = std::string(ctime(&now));
    current_time.pop_back();
    const std::filesystem::path directory_named_time = current_time;

    std::filesystem::create_directories(directory_named_time);
    return directory_named_time;
}

void save_point_data(ORB_SLAM3::System& SLAM,
                     const std::filesystem::path& directory) {
    const std::filesystem::path point_data_xyz_path =
        directory / "point_data.xyz";
    const std::filesystem::path point_data_csv_path =
        directory / "point_data.csv";
    const std::vector<ORB_SLAM3::MapPoint*> map_points =
        SLAM.GetAtlas()
            ->GetCurrentMap()  // TODO: should we take all the maps?
            ->GetAllMapPoints();

    std::ofstream point_data(point_data_csv_path);
    std::ofstream point_data_xyz(point_data_xyz_path);

    for (const auto& p : map_points) {
        if (p != nullptr) {
            const Eigen::Vector3f v = p->GetWorldPos();

            point_data << v.x() << "," << v.y() << "," << v.z() << std::endl;
            point_data_xyz << v.x() << " " << v.y() << " " << v.z()
                           << std::endl;
        }
    }

    point_data.close();
    point_data_xyz.close();
}

void register_signal() {
    struct sigaction sigint_handler;
    sigint_handler.sa_handler = exit_session;
    sigemptyset(&sigint_handler.sa_mask);
    sigint_handler.sa_flags = 0;
    sigaction(SIGINT, &sigint_handler, NULL);
    continue_session = true;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "USAGE: " << argv[0]
                  << " VOCABULARY_FILE_PATH CALIBRATION_FILE_PATH "
                     "[--continue MAP_FILE_PATH] [--use-webcam]"
                  << std::endl;
        return 1;
    }

    register_signal();

    const bool reuse_map_file =
        argc >= 5 && std::string(argv[3]) == "--continue";
    const bool use_webcam =
        ((reuse_map_file && argc >= 6) || (!reuse_map_file && argc >= 4)) &&
        std::string(argv[reuse_map_file ? 5 : 3]) == "--use-webcam";

    const std::filesystem::path directory_named_time =
        create_new_directory_named_current_time();

    const std::filesystem::path SLAM_map_location =
        !reuse_map_file ? directory_named_time / "SLAM_map.bin" : argv[4];

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR);

    std::shared_ptr<Drone> drone =
        use_webcam ? nullptr : std::make_shared<Drone>();

    Streamer streamer(drone);
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>& frame_queue =
        streamer.get_frame_queue();

    streamer.start_stream();

    auto start_time = std::chrono::steady_clock::now();

    std::array<uchar, 640 * 480 * 3> current_frame;

    while (continue_session) {
        auto timestamp = std::chrono::steady_clock::now();
        while (!frame_queue.pop(current_frame)) {
            std::this_thread::sleep_for(20ms);
        }

        const auto pose = SLAM.TrackMonocular(
            cv::Mat(480, 640, CV_8UC3, current_frame.data()),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                timestamp - start_time)
                .count());
    }

    std::cout << "Shutting down" << std::endl;
    SLAM.Shutdown();
    save_point_data(SLAM, directory_named_time);
    // std::filesystem::rename(DESTINATIONS_FILE_NAME,
    //                         directory_named_time / DESTINATIONS_FILE_NAME);
    SLAM.SaveAtlasAsOsaWithTimestamp(directory_named_time / "SLAM_map.bin");
    // cvDestroyAllWindows();

    return 0;
}
