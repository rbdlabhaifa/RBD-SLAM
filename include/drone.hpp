#ifndef _H_DRONE
#define _H_DRONE

#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <memory>  // std::shared_ptr
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <utility>

#include "ctello.h"

class Drone {
   public:
    // Need to use this constructor for Charger
    Drone(const std::string &drone_name,
          const std::string &rpi_bluetooth_address)
        : tello_(std::make_shared<ctello::Tello>()), drone_name_(drone_name) {}

    Drone() : tello_(std::make_shared<ctello::Tello>()) {
        while (!tello_->Bind())
            ;
    }

    void update_pose(const cv::Mat &drone_pose);
    const cv::Mat &get_pose();

    int get_battery();

    void send_command(const std::string &cmd, bool wait_for_response = true);

    void tello_stream_on();
    void tello_stream_off();

    void turn_drone_on_and_connect(bool turn_on = true);
    void turn_drone_off();
    void activate_drone();

    void testOnOff();

    void test_reconnection();
    /**
     * @brief Set the end drone process object
     *
     * @param end_drone_process
     */
   private:
    //--- Variables ---

    std::shared_ptr<ctello::Tello> tello_;
    std::string drone_name_;

    cv::Mat drone_pose_;

    int arucoId;
};

#endif
