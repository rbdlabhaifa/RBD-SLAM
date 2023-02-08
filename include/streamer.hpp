#ifndef STREAMER_H_
#define STREAMER_H_
#include <opencv2/core/hal/interface.h>

#include <array>
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <thread>

#include "drone.hpp"

#define MAX_FRAMES_IN_QUEUE 3

class Streamer {
    std::shared_ptr<SomeDrone> drone;
    std::thread image_thread;
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>> frame_queue;

    std::atomic_bool close_stream;
    void grab_image();

   public:
    Streamer(std::shared_ptr<SomeDrone> drone = nullptr,
             int max_frames_in_queue = MAX_FRAMES_IN_QUEUE);
    ~Streamer();
    void start_stream();
    void end_stream();
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>&
    get_frame_queue();
};

#endif  // STREAMER_H_
