#include "drone.hpp"

// void Drone::testOnOff() {
//     while (true) {
//         turn_drone_on_and_connect();
//         charger->turn_drone_off();
//         turn_drone_on_and_connect();
//     }
// }

// void Drone::test_reconnection() { charger->test_reconnection(); }

void Drone::send_command(const std::string &cmd, bool wait_for_response) {
    std::cout << "(Tello" << (send_commands ? "" : " Fake")
              << ") command: " << cmd << std::endl;
    if (!send_commands && cmd != "streamon" && cmd != "streamoff") return;

    if (!wait_for_response) {
        tello_->SendCommand(cmd);
        return;
    }

    while (!tello_->SendCommandWithResponse(cmd)) {
        std::cout << "(Tello) failed, send again command: " << cmd << std::endl;
        usleep(200000);
    }
    if (cmd == "takeoff") {
        sleep(7);
        auto height = tello_->GetHeightStatus();
        std::cout << "Height: " << height << std::endl;
    }
}

void Drone::activate_drone() {
    while (!tello_->SendCommandWithResponse("command")) {
        std::cout << "(Tello) failed, send again command \"command\""
                  << std::endl;
        usleep(200000);
    }
}

void SomeDrone::update_pose(const cv::Mat &drone_pose) {
    this->drone_pose_ = drone_pose;
}

const cv::Mat &SomeDrone::get_pose() { return drone_pose_; }

void Drone::tello_stream_on() { send_command("streamon"); }

int Drone::get_battery() { return tello_->GetBatteryStatus(); }

void Drone::tello_stream_off() { send_command("streamoff"); }
