#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
// #include <filesystem>
#include "viridis.h"

using namespace std;
// namespace fs = std::filesystem;

#if defined _MSC_VER
#include <direct.h>
#elif defined __GNUC__
#include <sys/stat.h>
#include <sys/types.h>
#endif

void createDir(string dir) {
#if defined _MSC_VER
    _mkdir(dir.data());
#elif defined __GNUC__
    mkdir(dir.data(), 0777);
#endif
}

int main() {
    try {
        const int width = 640;
        const int height = 480;
        const int fps = 15;
        const int delay = 2000;
        char command;
        bool is_record = true;

        // Pipeline
        rs2::pipeline pipe;
        rs2::config pipe_config;
        pipe_config.enable_stream(RS2_STREAM_DEPTH, width, height,
                                  RS2_FORMAT_Z16, fps);
        pipe_config.enable_stream(RS2_STREAM_COLOR, width, height,
                                  RS2_FORMAT_RGB8, fps);

        rs2::pipeline_profile profile = pipe.start(pipe_config);
        // rs2_intrinsics intrinsics =
        // profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        rs2_intrinsics intrinsics = profile.get_stream(RS2_STREAM_COLOR)
                                        .as<rs2::video_stream_profile>()
                                        .get_intrinsics();
        char root[500];

        rs2::device selected_device = profile.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();
        float depth_scale = depth_sensor.get_depth_scale();
        std::cout << "depth scale: " + std::to_string(depth_scale) << std::endl;

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,
                                    1.f);   // Enable emitter
            // depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); //
            // Disable emitter
        }

        std::ofstream intrinsic;
        intrinsic.open("../data/intrinsics.txt", ios::out);

        std::stringstream intr_ss;
        std::ofstream rgb_list_TUM, depth_list_TUM;

        intr_ss << fixed << "Camera.fx: " << setprecision(1) << intrinsics.fx
                << endl
                << "Camera.fy: " << setprecision(1) << intrinsics.fy << endl
                << "Camera.cx: " << setprecision(1) << intrinsics.ppx << endl
                << "Camera.cy: " << setprecision(1) << intrinsics.ppy << endl
                << endl
                << "Camera.k1: " << setprecision(1) << intrinsics.coeffs[0]
                << endl
                << "Camera.k2: " << setprecision(1) << intrinsics.coeffs[1]
                << endl
                << "Camera.p1: " << setprecision(1) << intrinsics.coeffs[2]
                << endl
                << "Camera.p2: " << setprecision(1) << intrinsics.coeffs[3]
                << endl
                << endl
                << "Camera.width: " << intrinsics.width << endl
                << "Camera.height: " << intrinsics.height << endl;
        intrinsic << intr_ss.str();
        intrinsic.close();

        createDir(std::string("../data/long_record/"));
        snprintf(root, 500, "../data/long_record/%ld/",
                 long(std::time(0)) + int(delay / 1000));   // build root dir
        cout << "Writing to " << root << endl;
        createDir(std::string(root));
        createDir(std::string(root) + "rgb");
        createDir(std::string(root) + "depth");
        rgb_list_TUM.open(std::string(root) + "rgb.txt", ios::out);
        depth_list_TUM.open(std::string(root) + "depth.txt", ios::out);

        rs2::align align_to_color(RS2_STREAM_COLOR);

        while (true) {
            rs2::frameset frameset = pipe.wait_for_frames();
            frameset = align_to_color.process(frameset);

            rs2::video_frame rgb_frame = frameset.get_color_frame();
            rs2::video_frame depth_frame = frameset.get_depth_frame();

            char TUM_timestamp_rgb[200], TUM_timestamp_depth[200];
            snprintf(TUM_timestamp_rgb, 200, "%.9lf",
                     rgb_frame.get_timestamp() / 1e3);
            snprintf(TUM_timestamp_depth, 200, "%.9lf",
                     depth_frame.get_timestamp() / 1e3);

            cv::Mat rgb_data(cv::Size(width, height), CV_8UC3,
                             (void *)rgb_frame.get_data());
            cv::cvtColor(rgb_data, rgb_data, cv::COLOR_RGB2BGR);
            cv::Mat depth_data(cv::Size(width, height), CV_16UC1,
                               (void *)depth_frame.get_data());
            std::ostringstream rgb_filename, depth_filename;

            if (!rgb_data.empty()) {
                if (is_record) {
                    rgb_filename << root << "rgb/"
                                 << std::string(TUM_timestamp_rgb) << ".png";
                    rgb_list_TUM << TUM_timestamp_rgb << " "
                                 << "rgb/" << TUM_timestamp_rgb << ".png"
                                 << endl;
                    cv::imwrite(rgb_filename.str(), rgb_data);
                    cv::circle(rgb_data, cv::Point(50, 50), 5,
                               cv::Scalar(0, 0, 255), -1);
                    cv::putText(rgb_data, "Rec", cv::Point(60, 62),
                                cv::FONT_HERSHEY_TRIPLEX, 1,
                                cv::Scalar(0, 0, 255));
                }
                cv::imshow("rgb", rgb_data);
            }
            if (!depth_data.empty()) {
                float max_depth = 8;
                cv::Mat bgr;
                depth_data.convertTo(bgr, CV_8UC1,
                                     (255 / max_depth) * depth_scale);
                bgr = VirdisColorMap(bgr);
                cv::imshow("depth", bgr);
                if (is_record) {
                    depth_filename << root << "depth/"
                                   << std::string(TUM_timestamp_depth)
                                   << ".png";
                    depth_list_TUM << TUM_timestamp_depth << " "
                                   << "depth/" << TUM_timestamp_depth << ".png"
                                   << endl;
                    cv::imwrite(depth_filename.str(), depth_data);
                }
            }
            cv::waitKey(5);
            rgb_list_TUM.flush();
            depth_list_TUM.flush();
        }
        pipe.stop();
    } catch (const std::exception &e) {
        char msg[500];
        snprintf(msg, 500, "record: %s", e.what());
        printf("%s\n", msg);
    }
    return 0;
}