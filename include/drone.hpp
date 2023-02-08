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

class SomeDrone {
   public:
    void update_pose(const cv::Mat &drone_pose);
    const cv::Mat &get_pose();

    virtual int get_battery() { return 0; };

    virtual void send_command(const std::string &cmd,
                              bool wait_for_response = true){};

    virtual void tello_stream_on(){};
    virtual void tello_stream_off(){};

    virtual void activate_drone(){};

   private:
    cv::Mat drone_pose_;
};

class Drone : public SomeDrone {
   public:
    // Need to use this constructor for Charger
    Drone(const std::string &drone_name,
          const std::string &rpi_bluetooth_address, bool send_commands = true)
        : send_commands(send_commands),
          tello_(std::make_shared<ctello::Tello>()),
          drone_name_(drone_name) {}

    Drone(bool send_commands = true)
        : send_commands(send_commands),
          tello_(std::make_shared<ctello::Tello>()) {}

    void send_command(const std::string &cmd, bool wait_for_response = true);
    void tello_stream_on();
    void tello_stream_off();

    void activate_drone();
    int get_battery();

    void testOnOff();

    void test_reconnection();

   private:
    //--- Variables ---

    std::shared_ptr<ctello::Tello> tello_;
    std::string drone_name_;
    bool send_commands;

    int arucoId;
};

#endif
