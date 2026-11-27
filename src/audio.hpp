#pragma once
#include "sim.hpp"

class AudioEngine {
public:
    void setListener(std::function<Vec3()> pos, std::function<float()> yaw);
    void setSource(const Vec3& s) { source = s; }

    bool loadFile(const std::string& path, std::string& err);
    bool play(std::string& err);
    void stop();
    bool playing() const { return playingFlag; }

    bool startRecording(std::string& err);
    void stopRecording();
    bool recording() const { return recFlag; }
    std::string recordingPath() const { return recPath; }
    std::string pollFinished();      // returns saved path when recording thread ends, else ""
    std::string pollError();         // returns last error and clears it

    std::function<void(const std::string&)> onDone;

    std::string loadedPath;

private:
    void playLoop();
    std::vector<float> buf;
    std::mutex bufMu;
    std::atomic<bool> playingFlag{false};
    std::atomic<bool> stopPlay{false};
    std::thread playThr;

    std::atomic<bool> recFlag{false};
    std::thread recThr;
    std::atomic<pid_t> recPid{-1};
    std::string recPath;
    std::atomic<bool> recDone{false};
    std::string recDonePath;
    std::string recErr;
    std::mutex recMu;

    Vec3 source = {0, 0.4f, 0};
    std::function<Vec3()> listenerPos;
    std::function<float()> listenerYaw;

    static std::string uniqueRecName();
};
