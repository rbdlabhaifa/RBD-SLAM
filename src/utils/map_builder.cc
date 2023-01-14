#include <Converter.h>
#include <Eigen/src/Core/Matrix.h>
#include <Optimizer.h>
#include <System.h>
#include <bits/types/time_t.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui_c.h>

#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <thread>
#include <vector>

#include "MapPoint.h"
#include "streamer.hpp"

#define DESTINATIONS_FILE_NAME "drone_destinations.txt"

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

void save_point_data(ORB_SLAM2::System& SLAM,
                     const std::filesystem::path& directory) {
    const std::filesystem::path point_data_xyz_path =
        directory / "point_data.xyz";
    const std::filesystem::path point_data_csv_path =
        directory / "point_data.csv";
    const std::vector<ORB_SLAM2::MapPoint*> map_points =
        SLAM.GetMap()->GetAllMapPoints();

    std::ofstream point_data;
    std::ofstream point_data_xyz(point_data_xyz_path);

    point_data.open(point_data_csv_path);

    for (auto p : map_points) {
        if (p != nullptr) {
            const cv::Mat point = p->GetWorldPos();
            const Eigen::Matrix<double, 3, 1> v =
                ORB_SLAM2::Converter::toVector3d(point);

            point_data << v.x() << "," << v.y() << "," << v.z() << std::endl;
            point_data_xyz << v.x() << " " << v.y() << " " << v.z()
                           << std::endl;
        }
    }

    point_data.close();
    point_data_xyz.close();
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "USGAE: " << argv[0]
                  << " VOCABULARY_FILE_PATH CALIBRATION_FILE_PATH MAP_FILE "
                     "[--continue]"
                  << std::endl
                  << std::endl
                  << "In order to use `--continue`, put the corresponding "
                     "MAP_FILE in the current folder"
                  << std::endl;
        return 1;
    }

    const bool reuse_map_file =
        argc >= 5 && std::string(argv[3]) == "--continue";
    const std::filesystem::path directory_named_time =
        create_new_directory_named_current_time();

    const std::filesystem::path SLAM_map_location =
        !reuse_map_file ? directory_named_time / "SLAM_map.bin"
                        : "SLAM_map.bin";

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true,
                           reuse_map_file, SLAM_map_location, reuse_map_file);
    // Drone drone;
    Streamer streamer;
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>& frame_queue =
        streamer.get_frame_queue();

    streamer.start_stream();

    int frame_count = 1;

    std::array<uchar, 640 * 480 * 3> current_frame;

    while (!SLAM.shutdown_requested) {
        while (!frame_queue.pop(current_frame)) {
            std::this_thread::sleep_for(20ms);
        }

        SLAM.TrackMonocular(cv::Mat(480, 640, CV_8UC3, current_frame.data()),
                            frame_count);
    }

    SLAM.Shutdown();
    save_point_data(SLAM, directory_named_time);
    std::filesystem::rename(DESTINATIONS_FILE_NAME,
                            directory_named_time / DESTINATIONS_FILE_NAME);
    SLAM.SaveMap(SLAM_map_location);
    cvDestroyAllWindows();

    return 0;
}
