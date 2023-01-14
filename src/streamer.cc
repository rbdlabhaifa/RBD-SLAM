#include "streamer.hpp"

#include <opencv2/core/hal/interface.h>

#include <array>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "drone.hpp"

using namespace std::chrono_literals;

Streamer::Streamer(std::shared_ptr<Drone> drone, int max_frames_in_queue)
    : drone(drone), frame_queue(max_frames_in_queue), close_stream(false) {}

Streamer::~Streamer() { end_stream(); }

boost::lockfree::spsc_queue<std::array<uchar, 640 * 480 * 3>>&
Streamer::get_frame_queue() {
    return frame_queue;
}

void Streamer::grab_image() {
    cv::Mat frame;
    std::array<uchar, 640 * 480 * 3> frame_arr;

    if (drone == nullptr)
        capture.open(0);
    else
        capture.open("udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=5000");

    while (!close_stream) {
        if (!capture.isOpened()) break;  // TODO: add logging here
        capture.read(frame);
        cv::resize(frame, frame, cv::Size(640, 480));
        std::move(frame.data, frame.data + frame.total() * frame.channels(),
                  frame_arr.data());

        frame_queue.push(frame_arr);
    }

    capture.release();
    if (!close_stream && drone != nullptr) drone->tello_stream_off();
}

void Streamer::start_stream() {
    if (drone != nullptr) drone->tello_stream_on();
    image_thread = std::thread(&Streamer::grab_image, this);
}

void Streamer::end_stream() {
    close_stream = true;
    image_thread.join();
    if (drone != nullptr) drone->send_command("streamoff");
}
