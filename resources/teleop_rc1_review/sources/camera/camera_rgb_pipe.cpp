// Preview-only serial-locked RGB capture. No ROS, CAN or robot interfaces.
#include <librealsense2/rs.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

static volatile std::sig_atomic_t stopping = 0;
static void stop_signal(int) { stopping = 1; }
static void le(std::vector<uint8_t>& b, uint64_t n, int size) {
    for (int i = 0; i < size; ++i) b.push_back((n >> (8*i)) & 255);
}
int main(int argc, char** argv) {
    if (argc != 2 || (std::string(argv[1]) != "401522071685" && std::string(argv[1]) != "409122273067" && std::string(argv[1]) != "409122272257")) {
        std::cerr << "Expected one of the three verified camera serials\n"; return 2;
    }
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    std::signal(SIGPIPE, stop_signal);
    rs2::pipeline pipe;
    bool started = false;
    try {
        rs2::config cfg;
        cfg.enable_device(argv[1]);
        // These three cameras currently share USB2. Keep preview sources at
        // 15 Hz; USB hotplug descriptor timing must not silently select 30 Hz.
        // Robot/control rates are not touched.
        const int fps = 15;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, fps);
        auto profile = pipe.start(cfg);
        started = true;
        auto dev = profile.get_device();
        if (std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) != argv[1] ||
            (std::string(dev.get_info(RS2_CAMERA_INFO_NAME)).find("D435") == std::string::npos && std::string(dev.get_info(RS2_CAMERA_INFO_NAME)).find("D405") == std::string::npos))
            throw std::runtime_error("Device identity mismatch");
        std::cerr << "RealSense RGB preview started serial=" << argv[1] << " 640x480@" << fps << "\n";
        bool first = true;
        while (!stopping) {
            rs2::frameset frames;
            if (!pipe.try_wait_for_frames(&frames, first ? 5000 : 1000)) {
                if (stopping) break;
                throw std::runtime_error("No camera frames within 1000ms");
            }
            auto color = frames.get_color_frame();
            if (!color) continue;
            first = false;
            const int w = color.get_width(), h = color.get_height();
            if (w != 640 || h != 480 || color.get_stride_in_bytes() != w*3)
                throw std::runtime_error("Unexpected RGB frame layout");
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            std::vector<uint8_t> head{'R','S','V','1'};
            le(head,w,4); le(head,h,4); le(head,color.get_frame_number(),8);
            le(head,static_cast<uint64_t>(ns),8); le(head,w*h*3,4);
            if (std::fwrite(head.data(),1,head.size(),stdout)!=head.size() ||
                std::fwrite(color.get_data(),1,w*h*3,stdout)!=static_cast<size_t>(w*h*3) ||
                std::fflush(stdout)!=0) break;
        }
        pipe.stop(); started = false;
        return stopping ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Camera capture unavailable: " << e.what() << "\n";
        if (started) { try { pipe.stop(); } catch (...) {} }
        return 1;
    }
}
