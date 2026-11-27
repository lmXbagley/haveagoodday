#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>
#include <algorithm>
#include <thread>

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float vlen(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 vnorm(const Vec3& a) { float l = vlen(a); return l > 1e-9f ? a * (1.0f / l) : Vec3(); }

struct Box { Vec3 c; float hx = 0, hy = 0, hz = 0; uint32_t color = 0; };
struct SphereObj { Vec3 c; float r = 0; uint32_t color = 0; };
struct CylObj { Vec3 c; float r = 0, h = 0; uint32_t color = 0; };

struct RobotState {
    Vec3 pos;
    float yaw = 0, pitch = 0;
    float speed = 0, om = 0, yawRate = 0;
    float spinL = 0, spinR = 0;
    float vl = 0, vr = 0;
    bool fallen = false;
};

struct Scene {
    std::vector<Box> boxes;
    std::vector<SphereObj> spheres;
    std::vector<CylObj> cylinders;
    RobotState robot;
    Vec3 soundSource;
};

struct MotorCmd { float l = 0, r = 0; };

struct LogEntry { double t = 0; int dir = 0; std::string msg; };

struct Logger {
    std::mutex mu;
    std::deque<LogEntry> entries;
    void add(int dir, const std::string& msg) {
        std::lock_guard<std::mutex> g(mu);
        entries.push_back({nowMs(), dir, msg});
        while (entries.size() > 600) entries.pop_front();
    }
    static double nowMs() {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count() * 1000.0;
    }
};

struct Camera { Vec3 pos; float yaw = 0, pitch = 0, fov = 1.15f; };

struct FrameTarget {
    std::vector<uint32_t> pix;
    int w = 0, h = 0;
    void resize(int w_, int h_) {
        if (w_ != w || h_ != h) { w = w_; h = h_; pix.assign((size_t)w_ * h_, 0xFF000000u); }
    }
    void clear(uint32_t c) { std::fill(pix.begin(), pix.end(), c); }
};

inline uint32_t shade(uint32_t col, float f) {
    if (f < 0) f = 0;
    if (f > 1.6f) f = 1.6f;
    int r = (int)(((col >> 16) & 0xFF) * f), g = (int)(((col >> 8) & 0xFF) * f), b = (int)((col & 0xFF) * f);
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    return (r << 16) | (g << 8) | b;
}
