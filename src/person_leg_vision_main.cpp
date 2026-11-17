// Standalone USB camera demo: YOLO11n pose ONNX -> leg joints -> track IDs.
// No dependency on PlcUdp, libusb or robot motor control.
#include "person_tracker.hpp"
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int kInput = 640;
constexpr int kValues = 56; // xywh (4) + confidence (1) + 17 * xyz (51)
constexpr int kJoints = 17;
constexpr float kJointThreshold = 0.40f;

struct Pose {
    vision::Rect box;
    float confidence = 0;
    std::array<cv::Point2f, kJoints> points;
    std::array<float, kJoints> scores;
};
struct Options {
    std::string model = "models/yolo11n-pose.onnx";
    int camera = 0;
    int width = 640;
    int height = 480;
    float score = 0.38f;
    int every = 2;
};
struct Selected {
    const vision::PersonTracker* tracker = nullptr;
    int target_id = -1;
};
bool number(const std::string& s, int& out) {
    try {
        size_t p = 0;
        out = std::stoi(s, &p);
        return p == s.size();
    } catch (...) { return false; }
}
void help() {
    std::cout << "Usage: person_leg_vision [--model models/yolo11n-pose.onnx]"
                 " [--camera 0] [--width 640] [--height 480] [--every 2]\n"
                 "Mouse click: lock target | c: clear | q/Esc: quit.\n"
                 "Only camera and local inference. No UDP/PLC commands.\n";
}
bool parse(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "-h" || key == "--help") { help(); std::exit(0); }
        if (i + 1 >= argc) return false;
        const std::string value = argv[++i];
        int v = 0;
        if (key == "--model") opt.model = value;
        else if (key == "--camera" && number(value,v) && v >= 0) opt.camera = v;
        else if (key == "--width" && number(value,v) && v >= 320 && v <= 1920) opt.width = v;
        else if (key == "--height" && number(value,v) && v >= 240 && v <= 1080) opt.height = v;
        else if (key == "--every" && number(value,v) && v >= 1 && v <= 15) opt.every = v;
        else return false;
    }
    return true;
}
void onMouse(int event, int x, int y, int, void* data) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    Selected* selected = static_cast<Selected*>(data);
    selected->target_id = -1;
    for (const auto& track : selected->tracker->tracks()) {
        if (!track.matched) continue;
        const auto& b = track.box;
        if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) {
            selected->target_id = track.id;
            std::cout << "[TARGET] ID " << track.id << '\n';
            return;
        }
    }
}
struct Letterbox {
    cv::Mat image;
    float scale = 1;
    int left = 0, top = 0;
};
Letterbox prepare(const cv::Mat& input) {
    Letterbox l;
    l.scale = std::min(float(kInput) / input.cols, float(kInput) / input.rows);
    const int w = std::max(1, cvRound(input.cols * l.scale));
    const int h = std::max(1, cvRound(input.rows * l.scale));
    cv::Mat resized;
    cv::resize(input, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
    l.left = (kInput - w) / 2;
    l.top = (kInput - h) / 2;
    cv::copyMakeBorder(resized, l.image, l.top, kInput - h - l.top,
                       l.left, kInput - w - l.left,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return l;
}
float coord(float v, int pad, float scale, int maximum) {
    return std::max(0.0f, std::min(float(maximum - 1), (v - pad) / scale));
}
std::vector<Pose> detect(cv::dnn::Net& net, const cv::Mat& frame, float threshold) {
    const Letterbox lb = prepare(frame);
    const cv::Mat blob = cv::dnn::blobFromImage(lb.image, 1.0/255.0,
                          cv::Size(kInput, kInput), cv::Scalar(), true, false, CV_32F);
    net.setInput(blob);
    const cv::Mat output = net.forward();
    if (output.dims != 3 || output.size[0] != 1) {
        throw std::runtime_error("Unexpected ONNX output: expected 3-dimensional [1,56,N]");
    }
    const bool channels_first = output.size[1] == kValues;
    const bool channels_last = output.size[2] == kValues;
    if (!channels_first && !channels_last)
        throw std::runtime_error("Unsupported pose output. Export YOLO11n-pose with nms=False, imgsz=640");
    const int proposals = channels_first ? output.size[2] : output.size[1];
    const float* tensor = reinterpret_cast<const float*>(output.ptr<float>());
    auto value = [&](int n, int channel) -> float {
        return channels_first ? tensor[channel * proposals + n]
                              : tensor[n * kValues + channel];
    };

    std::vector<Pose> poses;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for (int i = 0; i < proposals; ++i) {
        const float score = value(i, 4);
        if (!std::isfinite(score) || score < threshold) continue;
        const float xc = value(i, 0), yc = value(i, 1);
        const float w = value(i, 2), h = value(i, 3);
        if (!std::isfinite(xc) || !std::isfinite(yc) ||
            !std::isfinite(w) || !std::isfinite(h) || w <= 1 || h <= 1) continue;
        const float x1 = coord(xc - w/2, lb.left, lb.scale, frame.cols);
        const float y1 = coord(yc - h/2, lb.top, lb.scale, frame.rows);
        const float x2 = coord(xc + w/2, lb.left, lb.scale, frame.cols);
        const float y2 = coord(yc + h/2, lb.top, lb.scale, frame.rows);
        if (x2 - x1 < 12 || y2 - y1 < 20) continue;
        Pose p;
        p.box = vision::Rect(x1, y1, x2 - x1, y2 - y1);
        p.confidence = score;
        for (int j = 0; j < kJoints; ++j) {
            const float px = value(i, 5 + j*3);
            const float py = value(i, 6 + j*3);
            const float pc = value(i, 7 + j*3);
            p.points[j] = cv::Point2f(coord(px, lb.left, lb.scale, frame.cols),
                                      coord(py, lb.top, lb.scale, frame.rows));
            p.scores[j] = std::isfinite(pc) ? pc : 0;
        }
        poses.push_back(p);
        boxes.emplace_back(cvRound(x1), cvRound(y1), cvRound(x2 - x1), cvRound(y2 - y1));
        scores.push_back(score);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, threshold, 0.48f, keep);
    std::vector<Pose> result;
    for (int index : keep) result.push_back(poses[size_t(index)]);
    return result;
}
float overlap(const vision::Rect& a, const vision::Rect& b) {
    const float xx = std::max(a.x, b.x), yy = std::max(a.y, b.y);
    const float rr = std::min(a.x + a.w, b.x + b.w);
    const float bb = std::min(a.y + a.h, b.y + b.h);
    const float common = std::max(0.0f, rr - xx) * std::max(0.0f, bb - yy);
    const float area = a.w*a.h + b.w*b.h - common;
    return area <= 0 ? 0 : common/area;
}
bool legsCenter(const Pose& p, cv::Point2f& point, int& count) {
    // Knee and ankle keypoints, 13/14/15/16. Both left & right needed.
    count = 0;
    cv::Point2f sum(0, 0);
    const bool left = p.scores[13] >= kJointThreshold || p.scores[15] >= kJointThreshold;
    const bool right = p.scores[14] >= kJointThreshold || p.scores[16] >= kJointThreshold;
    if (!left || !right) return false;
    for (int i = 13; i <= 16; ++i) {
        if (p.scores[i] < kJointThreshold) continue;
        sum += p.points[i];
        ++count;
    }
    if (count < 2) return false;
    point = sum * (1.0f / count);
    return true;
}
void drawLegs(cv::Mat& frame, const Pose& pose, const cv::Scalar& color) {
    static const int pairs[][2] = {
        {11,13}, {13,15}, {12,14}, {14,16}
    };
    for (const auto& link : pairs) {
        const int a = link[0], b = link[1];
        if (pose.scores[a] < kJointThreshold || pose.scores[b] < kJointThreshold) continue;
        cv::line(frame, pose.points[a], pose.points[b], color, 3, cv::LINE_AA);
    }
    for (int j = 11; j <= 16; ++j) {
        if (pose.scores[j] < kJointThreshold) continue;
        cv::circle(frame, pose.points[j], 5, color, -1, cv::LINE_AA);
    }
}
} // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse(argc, argv, opt)) { help(); return 2; }
    std::ifstream model_file(opt.model, std::ios::binary);
    if (!model_file.good()) {
        std::cerr << "[MODEL] File not found: " << opt.model
                  << "\nExport yolo11n-pose.pt to ONNX using the commands in README.\n";
        return 1;
    }
    model_file.close();
    cv::dnn::Net net;
    try {
        net = cv::dnn::readNetFromONNX(opt.model);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const cv::Exception& e) {
        std::cerr << "[MODEL] Cannot load ONNX: " << e.what() << '\n';
        return 1;
    }
    cv::VideoCapture cam(opt.camera, cv::CAP_V4L2);
    if (!cam.isOpened()) {
        std::cerr << "[CAMERA] Cannot open /dev/video" << opt.camera << '\n';
        return 1;
    }
    cam.set(cv::CAP_PROP_FRAME_WIDTH, opt.width);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, opt.height);
    cam.set(cv::CAP_PROP_BUFFERSIZE, 1);
    vision::PersonTracker tracker(5);
    Selected selected;
    selected.tracker = &tracker;
    const std::string window = "LEG TRACKER - CAMERA ONLY - NO UDP";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setMouseCallback(window, onMouse, &selected);
    std::cout << "[POSE] Running model " << opt.model
              << "\n[SAFE] Camera-only: no PLC sockets, no UDP motion commands.\n";
    std::vector<Pose> poses;
    int frames = 0, missing_frames = 0;
    auto last = std::chrono::steady_clock::now();
    double fps = 0.0;
    while (true) {
        cv::Mat frame;
        if (!cam.read(frame) || frame.empty()) {
            std::cerr << "[CAMERA] Frame unavailable; exiting.\n";
            break;
        }
        ++frames;
        if ((frames - 1) % opt.every == 0) {
            try { poses = detect(net, frame, opt.score); }
            catch (const cv::Exception& e) {
                std::cerr << "[INFERENCE] " << e.what() << '\n';
                return 1;
            } catch (const std::exception& e) {
                std::cerr << "[INFERENCE] " << e.what() << '\n';
                return 1;
            }
            std::vector<vision::Detection> detections;
            for (const auto& p : poses) {
                vision::Detection d;
                d.box = p.box;
                d.score = p.confidence;
                detections.push_back(d);
            }
            tracker.update(detections, frame.cols, frame.rows);
            missing_frames = 0;
        } else {
            ++missing_frames;
        }
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now-last).count();
        if (dt > 0.0) fps = fps == 0 ? 1/dt : 0.85*fps + 0.15/dt;
        last = now;
        cv::line(frame, cv::Point(frame.cols/2, 0),
                 cv::Point(frame.cols/2, frame.rows), cv::Scalar(100,100,100), 1);
        for (const auto& t : tracker.tracks()) {
            if (!t.matched || missing_frames >= opt.every) continue;
            int pose_index = -1;
            float best = 0.15f;
            for (size_t j = 0; j < poses.size(); ++j) {
                const float s = overlap(t.box, poses[j].box);
                if (s > best) { best = s; pose_index = int(j); }
            }
            if (pose_index < 0) continue;
            const Pose& p = poses[size_t(pose_index)];
            const bool focus = t.id == selected.target_id;
            const cv::Scalar color = focus ? cv::Scalar(0,180,255) : cv::Scalar(50,230,70);
            const cv::Rect b(cvRound(t.box.x),cvRound(t.box.y),
                             cvRound(t.box.w),cvRound(t.box.h));
            cv::rectangle(frame, b, color, focus ? 3 : 2);
            drawLegs(frame, p, color);
            cv::Point2f leg;
            int valid = 0;
            const bool both = legsCenter(p, leg, valid);
            if (both) cv::drawMarker(frame, leg, color, cv::MARKER_CROSS, 20, 2);
            const float err = both ? (leg.x - frame.cols*0.5f)/(frame.cols*0.5f) : 0.0f;
            std::ostringstream label;
            label << "ID " << t.id << (both ? " LEGS OK " : " LEGS NOT VISIBLE ");
            if (both) label << std::fixed << std::setprecision(2) << "x=" << err;
            cv::putText(frame, label.str(),
                        cv::Point(std::max(0,b.x),std::max(48,b.y-7)),
                        cv::FONT_HERSHEY_SIMPLEX,0.46,color,2,cv::LINE_AA);
            if (focus && both && (frames % 15 == 0)) {
                std::cout << "[LEGS] target=" << t.id << " normalized_x="
                          << std::fixed << std::setprecision(3) << err
                          << " visible_knee_ankle=" << valid << '\n';
            }
        }
        cv::rectangle(frame, cv::Rect(0,0,frame.cols,33), cv::Scalar(30,30,30), -1);
        std::ostringstream header;
        header << "POSE / 2-LEG TRACKING | CAMERA ONLY | NO UDP | FPS "
               << std::fixed << std::setprecision(1) << fps;
        cv::putText(frame, header.str(),cv::Point(9,23),
                    cv::FONT_HERSHEY_SIMPLEX,0.49,cv::Scalar(255,255,255),1,cv::LINE_AA);
        cv::putText(frame,"Click person: select | C: clear | Q/Esc: exit",
                    cv::Point(10,frame.rows-10),cv::FONT_HERSHEY_SIMPLEX,
                    0.46,cv::Scalar(245,245,245),1,cv::LINE_AA);
        cv::imshow(window,frame);
        const int key = cv::waitKey(1)&0xff;
        if (key=='q' || key==27) break;
        if (key=='c') selected.target_id=-1;
    }
    cam.release();
    cv::destroyAllWindows();
    return 0;
}
