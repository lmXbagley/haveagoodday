#include "audio.hpp"
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>

static const int OUT_RATE = 48000;

bool AudioEngine::loadFile(const std::string& path, std::string& err) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { err = "cannot open " + path; return false; }
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsz < 44 || fsz > 200L * 1024 * 1024) { fclose(f); err = "bad file size"; return false; }
    std::vector<uint8_t> data((size_t)fsz);
    if (fread(data.data(), 1, (size_t)fsz, f) != (size_t)fsz) { fclose(f); err = "read failed"; return false; }
    fclose(f);

    if (fsz < 12 || memcmp(data.data(), "RIFF", 4) != 0 || memcmp(data.data() + 8, "WAVE", 4) != 0) {
        err = "not a RIFF/WAVE file"; return false;
    }
    size_t off = 12;
    int fmt = -1, ch = 0, rate = 0, bits = 0;
    const uint8_t* rawData = nullptr;
    size_t rawLen = 0;
    while (off + 8 <= data.size()) {
        char id[5] = {0};
        memcpy(id, data.data() + off, 4);
        uint32_t sz = 0;
        memcpy(&sz, data.data() + off + 4, 4);
        size_t body = off + 8;
        if (body + sz > data.size()) sz = data.size() - body;
        if (memcmp(id, "fmt ", 4) == 0 && sz >= 16) {
            uint16_t v;
            memcpy(&v, data.data() + body + 0, 2); fmt = v;
            memcpy(&v, data.data() + body + 2, 2); ch = v;
            uint32_t v32;
            memcpy(&v32, data.data() + body + 4, 4); rate = (int)v32;
            memcpy(&v, data.data() + body + 14, 2); bits = v;
        } else if (memcmp(id, "data", 4) == 0) {
            rawData = data.data() + body;
            rawLen = sz;
        }
        off = body + sz + (sz & 1);
    }
    if (fmt != 1 && fmt != 3) { err = "unsupported WAV format (need PCM)"; return false; }
    if (ch < 1 || ch > 2) { err = "unsupported channel count"; return false; }
    if (rate < 4000 || rate > 192000) { err = "unsupported sample rate"; return false; }
    if (fmt == 1 && bits != 8 && bits != 16 && bits != 24 && bits != 32) { err = "unsupported bit depth"; return false; }
    if (fmt == 3 && bits != 32) { err = "unsupported float depth"; return false; }
    if (!rawData || rawLen == 0) { err = "no data chunk"; return false; }

    int bytesPer = bits / 8;
    size_t frames = rawLen / ((size_t)bytesPer * ch);
    std::vector<float> mono(frames);
    for (size_t i = 0; i < frames; i++) {
        const uint8_t* p = rawData + i * (size_t)bytesPer * ch;
        float acc = 0;
        for (int c = 0; c < ch; c++) {
            const uint8_t* q = p + (size_t)c * bytesPer;
            float v = 0;
            if (fmt == 3) {
                float fv; memcpy(&fv, q, 4); v = fv;
            } else if (bits == 8) {
                v = ((int)q[0] - 128) / 128.0f;
            } else if (bits == 16) {
                int16_t sv; memcpy(&sv, q, 2); v = sv / 32768.0f;
            } else if (bits == 24) {
                int32_t iv = (int32_t)q[0] | ((int32_t)q[1] << 8) | ((int32_t)q[2] << 16);
                if (iv & 0x800000) iv |= (int32_t)0xFF000000;
                v = iv / 8388608.0f;
            } else {
                int32_t iv; memcpy(&iv, q, 4); v = iv / 2147483648.0f;
            }
            acc += v;
        }
        mono[i] = acc / ch;
    }

    size_t outFrames = (size_t)((double)frames * OUT_RATE / rate);
    if (outFrames < 1) outFrames = 1;
    std::vector<float> stereo((size_t)outFrames * 2);
    for (size_t i = 0; i < outFrames; i++) {
        double srcPos = (double)i * rate / OUT_RATE;
        size_t i0 = (size_t)srcPos;
        if (i0 >= frames) i0 = frames - 1;
        size_t i1 = (i0 + 1 < frames) ? i0 + 1 : i0;
        float fr = (float)(srcPos - (double)i0);
        float v = mono[i0] * (1.0f - fr) + mono[i1] * fr;
        stereo[i * 2] = v;
        stereo[i * 2 + 1] = v;
    }
    std::lock_guard<std::mutex> g(bufMu);
    buf = std::move(stereo);
    loadedPath = path;
    return true;
}

void AudioEngine::setListener(std::function<Vec3()> pos, std::function<float()> yaw) {
    listenerPos = std::move(pos);
    listenerYaw = std::move(yaw);
}

static bool mixerChunk(const std::vector<float>& src, size_t pos, std::vector<int16_t>& out,
                       const Vec3& lst, float lyaw, const Vec3& srcPos) {
    float dx = srcPos.x - lst.x, dz = srcPos.z - lst.z;
    float dist = std::sqrt(dx * dx + dz * dz);
    float ang = std::atan2(dx, dz) - lyaw;
    while (ang >  3.14159265f) ang -= 6.2831853f;
    while (ang < -3.14159265f) ang += 6.2831853f;
    float pan = std::sin(ang) * 0.8f;
    float gain = 1.0f / (1.0f + dist * dist);
    if (gain > 1.0f) gain = 1.0f;
    float gl = gain * (1.0f - pan * 0.7f);
    float gr = gain * (1.0f + pan * 0.7f);
    if (gl < 0) gl = 0;
    if (gl > 1.2f) gl = 1.2f;
    if (gr < 0) gr = 0;
    if (gr > 1.2f) gr = 1.2f;
    size_t frames = std::min((size_t)1024, (src.size() - pos) / 2);
    if (frames == 0) return false;
    out.resize(frames * 2);
    for (size_t i = 0; i < frames; i++) {
        float l = src[pos + i * 2] * gl, r = src[pos + i * 2 + 1] * gr;
        if (l > 1) l = 1;
        if (l < -1) l = -1;
        if (r > 1) r = 1;
        if (r < -1) r = -1;
        out[i * 2] = (int16_t)(l * 32767.0f);
        out[i * 2 + 1] = (int16_t)(r * 32767.0f);
    }
    return true;
}

void AudioEngine::playLoop() {
    Vec3 srcPos = source;
    FILE* p = popen("aplay -q -f S16_LE -r 48000 -c 2 -t raw", "we");
    if (!p) {
        std::lock_guard<std::mutex> g(recMu);
        recErr = "aplay not available";
        playingFlag = false;
        return;
    }
    std::vector<int16_t> chunk;
    size_t pos = 0;
    {
        std::lock_guard<std::mutex> g(bufMu);
        const std::vector<float>& src = buf;
        while (pos < src.size() && !stopPlay) {
            Vec3 lst = listenerPos ? listenerPos() : Vec3();
            float lyaw = listenerYaw ? listenerYaw() : 0;
            if (!mixerChunk(src, pos, chunk, lst, lyaw, srcPos)) break;
            if (fwrite(chunk.data(), 2, chunk.size(), p) != chunk.size()) break;
            pos += chunk.size();
        }
    }
    pclose(p);
    bool stopped = stopPlay;
    playingFlag = false;
    if (!stopped && onDone) onDone(loadedPath);
}

bool AudioEngine::play(std::string& err) {
    std::lock_guard<std::mutex> g(bufMu);
    if (buf.empty()) { err = "no audio loaded"; return false; }
    if (playingFlag) stopPlay = true;
    if (playThr.joinable()) playThr.join();
    stopPlay = false;
    playingFlag = true;
    playThr = std::thread([this]() { playLoop(); });
    return true;
}

void AudioEngine::stop() {
    if (playingFlag) stopPlay = true;
    if (playThr.joinable()) playThr.join();
    stopPlay = false;
    playingFlag = false;
}

std::string AudioEngine::uniqueRecName() {
    const char* dir = "recordings";
    mkdir(dir, 0755);
    char name[128];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    snprintf(name, sizeof(name), "%s/rec_%04d%02d%02d_%02d%02d%02d.wav",
             dir, tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    return name;
}

bool AudioEngine::startRecording(std::string& err) {
    if (recFlag) { err = "already recording"; return false; }
    recPath = uniqueRecName();
    pid_t pid = fork();
    if (pid < 0) { err = "fork failed"; return false; }
    if (pid == 0) {
        execlp("arecord", "arecord", "-q", "-f", "S16_LE", "-r", "48000", "-c", "1",
               recPath.c_str(), (char*)nullptr);
        _exit(127);
    }
    recPid = pid;
    recDone = false;
    recFlag = true;
    if (recThr.joinable()) recThr.join();
    recThr = std::thread([this]() {
        pid_t rpid = recPid;
        int status = 0;
        bool exited = false;
        for (int i = 0; i < 6000 && !exited; i++) {
            pid_t r = waitpid(rpid, &status, WNOHANG);
            if (r == rpid) exited = true;
            else if (r < 0) exited = true;
            else usleep(10000);
            if (!recFlag && !exited) kill(rpid, SIGINT);
        }
        if (!exited) { kill(rpid, SIGKILL); waitpid(rpid, &status, 0); exited = true; }
        bool ok = WIFEXITED(status) && (WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == 255);
        {
            std::lock_guard<std::mutex> g(recMu);
            if (!ok) recErr = "arecord failed (no mic or no permission?)";
            else recDonePath = recPath;
        }
        recDone = true;
        recFlag = false;
    });
    return true;
}

void AudioEngine::stopRecording() {
    recFlag = false;
    pid_t pid = recPid;
    if (pid > 0) kill(pid, SIGINT);
    if (recThr.joinable()) recThr.join();
}

std::string AudioEngine::pollFinished() {
    if (!recDone) return "";
    recDone = false;
    std::lock_guard<std::mutex> g(recMu);
    std::string p = recDonePath;
    recDonePath.clear();
    return p;
}

std::string AudioEngine::pollError() {
    std::lock_guard<std::mutex> g(recMu);
    std::string e = recErr;
    recErr.clear();
    return e;
}
