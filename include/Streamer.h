#ifndef STREAMER_H_
#define STREAMER_H_
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <thread>

#include "Drone.h"

#define MAX_FRAMES_IN_QUEUE 20

class Streamer {
    Drone& drone;
    std::thread image_thread;
    cv::VideoCapture capture;
    boost::lockfree::spsc_queue<std::vector<uchar>> frame_queue;

    bool close_stream = false;

    void grab_image();
    void end_drone_stream();

   public:
    Streamer(Drone& drone, const int max_frames_in_queue = MAX_FRAMES_IN_QUEUE);
    ~Streamer();
    void start_drone_stream();
    boost::lockfree::spsc_queue<std::vector<uchar>>& get_frame_queue();
};

#endif  // STREAMER_H_
