#ifndef STREAMER_H_
#define STREAMER_H_
#include <opencv2/core/hal/interface.h>

#include <array>
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <optional>
#include <thread>

#include "drone.hpp"

#define MAX_FRAMES_IN_QUEUE 3

/// Streaming from a drone or from a camera
class Streamer {
    std::optional<std::shared_ptr<SomeDrone>> drone;
    std::thread image_thread;

    /**
     * This queue is a single-producer-single-consumer lock-free queue, making
     * it thread-safe to read and write new frames
     */
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>> frame_queue;

    std::atomic_bool close_stream;
    void grab_image();

   public:
    explicit Streamer(
        std::optional<std::shared_ptr<SomeDrone>> drone = std::nullopt,
        int max_frames_in_queue = MAX_FRAMES_IN_QUEUE);
    Streamer(const Streamer &) = delete;
    Streamer(Streamer &&) = delete;
    Streamer &operator=(const Streamer &) = delete;
    Streamer &operator=(Streamer &&) = delete;
    ~Streamer();

    void start_stream();
    void end_stream();
    boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>
        &get_frame_queue();
};

#endif  // STREAMER_H_
