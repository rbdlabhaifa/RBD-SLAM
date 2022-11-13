#include "Streamer.h"

#include <opencv2/imgproc.hpp>
#include <thread>

using namespace std::chrono_literals;

Streamer::Streamer(Drone& drone, const int max_frames_in_queue)
    : drone(drone), frame_queue(max_frames_in_queue) {}
Streamer::~Streamer() { end_drone_stream(); }

boost::lockfree::spsc_queue<std::vector<uchar>>& Streamer::get_frame_queue() {
    return frame_queue;
}

void Streamer::grab_image() {
    cv::Mat frame;
    std::vector<uchar> frame_vec;

    capture.open("udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=5000");

    while (!close_stream) {
        if (!capture.isOpened()) break;  // TODO: add logging here
        capture.read(frame);
        cv::resize(frame, frame, cv::Size(640, 480));
        frame_vec.assign(frame.data,
                         frame.data + frame.total() * frame.channels());

        frame_queue.push(frame_vec);
    }

    capture.release();
    drone.tello_stream_off();
}

void Streamer::start_drone_stream() {
    drone.tello_stream_on();
    image_thread = std::thread(&Streamer::grab_image, this);
}

void Streamer::end_drone_stream() {
    close_stream = true;
    image_thread.join();
    drone.send_command("streamoff");
}
