#pragma once
#include "sim.hpp"

struct Api {
    std::mutex mu;
    MotorCmd motors;
    int touchMask = 0;

    std::function<RobotState()> snapshot;
    std::function<bool(std::vector<uint8_t>& out, int& w, int& h)> camFrame;
    std::function<bool(const std::string& path, std::string& err)> playSound;
    std::function<void()> stopSound;

    std::atomic<bool> running{false};
    int port = 4242;

    void pushEvent(const std::string& e);
    std::string panelStateString();
    void setTouchMask(int mask);
};

void startApiServer(Api& api, Logger& log);
void stopApiServer(Api& api);
