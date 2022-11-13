#include "Drone.h"

// void Drone::testOnOff() {
//     while (true) {
//         turn_drone_on_and_connect();
//         charger->turn_drone_off();
//         turn_drone_on_and_connect();
//     }
// }

// void Drone::test_reconnection() { charger->test_reconnection(); }

void Drone::send_command(const std::string &cmd) {
    std::cout << "(Tello) command: " << cmd << std::endl;

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

void Drone::update_pose(const cv::Mat &drone_pose) {
    this->drone_pose_ = drone_pose;
}

const cv::Mat &Drone::get_pose() { return drone_pose_; }

void Drone::tello_stream_on() { send_command("streamon"); }

int Drone::get_battery() { return tello_->GetBatteryStatus(); }

void Drone::tello_stream_off() { send_command("streamoff"); }

// void Drone::turn_drone_off() { charger->turn_drone_off(); }

// void Drone::turn_drone_on_and_connect(bool turn_on) {
//     if (turn_on) charger->turn_drone_on();
//     charger->connectToDrone(drone_name_);
//     tello_ = std::make_shared<ctello::Tello>();
//     while (!tello_->Bind())
//         ;
//     std::cout << "CONNECTED" << std::endl;
//     charger->set_tello(tello_);
// }
