#include "person_tracker.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
struct Options {
    int camera = 0;
    int width = 640;
    int height = 480;
    int detect_every = 3;
    double hit_threshold = 0.0;
};
void help(const char* name) {
    std::cout << "Usage: " << name
              << " [--camera 0] [--width 640] [--height 480]"
              << " [--detect-every 3] [--hit-threshold 0.0]\n"
              << "q/Esc quit, mouse click person to highlight, c clear selection.\n"
              << "Camera-only application: NO PLC, NO UDP and NO motion commands.\n";
}
bool parseInt(const std::string& s, int& out) {
    try {
        size_t n = 0;
        const int v = std::stoi(s, &n);
        if (n != s.size()) return false;
        out = v; return true;
    } catch (...) { return false; }
}
bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { help(argv[0]); std::exit(0); }
        if (i + 1 >= argc) return false;
        const std::string value = argv[++i];
        int number = 0;
        if (arg == "--camera" && parseInt(value, number) && number >= 0) o.camera = number;
        else if (arg == "--width" && parseInt(value, number) && number >= 320 && number <= 1920) o.width = number;
        else if (arg == "--height" && parseInt(value, number) && number >= 240 && number <= 1080) o.height = number;
        else if (arg == "--detect-every" && parseInt(value, number) && number >= 1 && number <= 15) o.detect_every = number;
        else if (arg == "--hit-threshold") {
            try {
                size_t n = 0;
                o.hit_threshold = std::stod(value, &n);
                if (n != value.size() || !std::isfinite(o.hit_threshold) ||
                    o.hit_threshold < -1.0 || o.hit_threshold > 2.0) return false;
            } catch (...) { return false; }
        } else return false;
    }
    return true;
}
struct Selection {
    const vision::PersonTracker* tracker = nullptr;
    int id = -1;
};
void mouse(int event, int x, int y, int, void* user) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto* sel = static_cast<Selection*>(user);
    if (!sel || !sel->tracker) return;
    sel->id = -1;
    // A click only highlights a detected person; it never sends a motion command.
    for (const auto& track : sel->tracker->tracks()) {
        if (!track.matched) continue;
        const auto& b = track.box;
        if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) {
            sel->id = track.id;
            std::cout << "[TARGET] Selected temporary ID " << sel->id << '\n';
            break;
        }
    }
}
std::vector<vision::Detection> detectPeople(const cv::Mat& image, cv::HOGDescriptor& detector,
                                            double threshold) {
    std::vector<cv::Rect> boxes;
    std::vector<double> weights;
    detector.detectMultiScale(image, boxes, weights, threshold, cv::Size(8, 8),
                              cv::Size(8, 8), 1.05, 2.0, false);
    std::vector<vision::Detection> found;
    for (size_t i = 0; i < boxes.size(); ++i) {
        const cv::Rect& b = boxes[i];
        if (b.width < 25 || b.height < 50) continue;
        vision::Detection d;
        d.box = {float(b.x), float(b.y), float(b.width), float(b.height)};
        d.score = float(i < weights.size() ? weights[i] : 0.0);
        found.push_back(d);
    }
    return found;
}
void overlay(cv::Mat& frame, const vision::PersonTracker& tracker, int chosen,
             double fps, int processed_frames, int detect_every) {
    const cv::Point center(frame.cols / 2, frame.rows / 2);
    cv::line(frame, cv::Point(center.x, 0), cv::Point(center.x, frame.rows),
             cv::Scalar(120, 120, 120), 1);
    cv::line(frame, cv::Point(0, center.y), cv::Point(frame.cols, center.y),
             cv::Scalar(120, 120, 120), 1);
    for (const auto& track : tracker.tracks()) {
        if (!track.matched) continue; // do not present stale boxes as current detections
        const cv::Rect box(cvRound(track.box.x), cvRound(track.box.y),
                           cvRound(track.box.w), cvRound(track.box.h));
        const bool selected = track.id == chosen;
        const cv::Scalar color = selected ? cv::Scalar(0, 200, 255) : cv::Scalar(60, 220, 70);
        cv::rectangle(frame, box, color, selected ? 3 : 2);
        const int cx = box.x + box.width / 2;
        const int cy = box.y + box.height / 2;
        cv::circle(frame, cv::Point(cx, cy), 4, color, -1);
        const double error_x = (double(cx) - center.x) / std::max(1, center.x);
        std::ostringstream label;
        label << "ID " << track.id << "  center_err_x=" << std::fixed
              << std::setprecision(2) << error_x;
        cv::putText(frame, label.str(), cv::Point(std::max(0, box.x), std::max(20, box.y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.46, color, 2, cv::LINE_AA);
    }
    std::ostringstream status;
    status << "CAMERA-ONLY / NO UDP  FPS:" << std::fixed << std::setprecision(1) << fps
           << "  detections every " << detect_every << " frames";
    cv::rectangle(frame, cv::Rect(0, 0, frame.cols, 32), cv::Scalar(24, 24, 24), -1);
    cv::putText(frame, status.str(), cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX,
                0.42, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(frame, "Click: highlight person | C: clear | Q/Esc: quit",
                cv::Point(10, frame.rows - 12), cv::FONT_HERSHEY_SIMPLEX,
                0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    (void)processed_frames;
}
} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) { help(argv[0]); return 2; }

    cv::VideoCapture camera(options.camera, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::cerr << "Cannot open /dev/video" << options.camera
                  << ". Check camera permissions, index and other apps.\n";
        return 1;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    vision::PersonTracker tracker(8);
    Selection selection;
    selection.tracker = &tracker;
    const std::string window = "USB Camera - Person Detection & Tracking (No PLC)";
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window, mouse, &selection);
    std::cout << "[VISION] Camera opened. PERSON detector: OpenCV HOG/SVM.\n"
              << "[VISION] No UDP sockets are created in this program.\n"
              << "[VISION] Stand fully visible in camera view for detection.\n";
    int frame_count = 0;
    auto last = std::chrono::steady_clock::now();
    double fps = 0.0;
    for (;;) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) {
            std::cerr << "[VISION] Camera stream lost; exiting.\n";
            break;
        }
        ++frame_count;
        const bool detect_now = (frame_count % options.detect_every) == 1 ||
                                options.detect_every == 1;
        if (detect_now) {
            const std::vector<vision::Detection> detections =
                detectPeople(frame, hog, options.hit_threshold);
            tracker.update(detections, frame.cols, frame.rows);
        }
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        if (dt > 0) fps = fps == 0.0 ? 1.0 / dt : fps * 0.85 + (1.0 / dt) * 0.15;
        last = now;
        overlay(frame, tracker, selection.id, fps, frame_count, options.detect_every);
        cv::imshow(window, frame);
        const int key = cv::waitKey(1) & 0xff;
        if (key == 'q' || key == 27) break;
        if (key == 'c') selection.id = -1;
    }
    camera.release();
    cv::destroyAllWindows();
    return 0;
}
