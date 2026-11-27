#include "api.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <thread>

namespace {

struct Client {
    std::string buf;
    bool stream = false;
    bool telem = false;
    bool dead = false;
};

struct Server {
    Api* api = nullptr;
    Logger* log = nullptr;
    int lfd = -1;
    std::mutex evMu;
    std::deque<std::string> events;
    std::map<int, Client> clients;
    int touchSent = -1;
    double lastTelem = 0;
    double lastStream = 0;
    std::thread thr;

    void broadcast(const std::string& data) {
        for (auto& [fd, c] : clients) {
            if (c.dead) continue;
            if (!sendAll(fd, data)) c.dead = true;
        }
    }
    static bool sendAll(int fd, const std::string& data) {
        size_t off = 0;
        while (off < data.size()) {
            ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
            if (n <= 0) {
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { usleep(1000); continue; }
                return false;
            }
            off += (size_t)n;
        }
        return true;
    }
    void drainEvents() {
        std::deque<std::string> ev;
        { std::lock_guard<std::mutex> g(evMu); ev.swap(events); }
        for (auto& e : ev) {
            for (auto& [fd, c] : clients) {
                if (c.dead) continue;
                log->add(1, e);
                if (!sendAll(fd, e + "\n")) c.dead = true;
            }
        }
    }
    void pushTouchEvents() {
        int cur;
        { std::lock_guard<std::mutex> g(api->mu); cur = api->touchMask; }
        if (touchSent == -1) { touchSent = cur; return; }
        int diff = cur ^ touchSent;
        for (int i = 0; i < 3; i++) {
            if (diff & (1 << i))
                broadcast(std::string("PANEL ") + std::to_string(i + 1) + ((cur & (1 << i)) ? " DOWN" : " UP"));
        }
        touchSent = cur;
    }
    void sendTelem() {
        RobotState r = api->snapshot();
        char b[256];
        snprintf(b, sizeof(b), "TELEM %.3f %.3f %.3f %.4f %.4f %.3f %.3f",
                 r.pos.x, r.pos.y, r.pos.z, r.yaw, r.pitch, r.vl, r.vr);
        broadcast(b);
    }
    void sendFrameTo(Client& c, int fd) {
        std::vector<uint8_t> raw;
        int w = 0, h = 0;
        if (!api->camFrame || !api->camFrame(raw, w, h) || raw.empty()) return;
        char hdr[64];
        snprintf(hdr, sizeof(hdr), "FRAME %d %d %zu\n", w, h, raw.size());
        if (!sendAll(fd, hdr)) { c.dead = true; return; }
        if (!sendAll(fd, std::string((const char*)raw.data(), raw.size()))) c.dead = true;
    }
    void handleLine(int fd, Client& c, const std::string& line) {
        if (line.empty()) return;
        log->add(0, line);
        std::vector<std::string> tok;
        size_t pos = 0;
        while (pos < line.size()) {
            size_t sp = line.find(' ', pos);
            if (sp == std::string::npos) { tok.push_back(line.substr(pos)); break; }
            tok.push_back(line.substr(pos, sp - pos));
            pos = sp + 1;
        }
        const std::string& cmd = tok[0];
        auto reply = [&](const std::string& s) {
            log->add(1, s);
            if (!sendAll(fd, s + "\n")) c.dead = true;
        };
        if (cmd == "MOTOR" && tok.size() >= 3) {
            std::lock_guard<std::mutex> g(api->mu);
            api->motors.l = std::clamp(std::strtof(tok[1].c_str(), nullptr), -100.0f, 100.0f);
            api->motors.r = std::clamp(std::strtof(tok[2].c_str(), nullptr), -100.0f, 100.0f);
            reply("OK MOTOR");
        } else if (cmd == "STOP") {
            std::lock_guard<std::mutex> g(api->mu);
            api->motors = {0, 0};
            reply("OK STOP");
        } else if (cmd == "SOUND" && tok.size() >= 2) {
            std::string path = line.substr(line.find(' ') + 1);
            std::string err;
            if (api->playSound && api->playSound(path, err)) reply("OK SOUND " + path);
            else reply("ERR SOUND " + (err.empty() ? "cannot play" : err));
        } else if (cmd == "SOUND_STOP") {
            if (api->stopSound) api->stopSound();
            reply("OK SOUND_STOP");
        } else if (cmd == "FRAME") {
            sendFrameTo(c, fd);
        } else if (cmd == "STREAM" && tok.size() >= 2) {
            c.stream = (std::strtol(tok[1].c_str(), nullptr, 10) != 0);
            reply(c.stream ? "OK STREAM 1" : "OK STREAM 0");
        } else if (cmd == "TELEM" && tok.size() >= 2) {
            c.telem = (std::strtol(tok[1].c_str(), nullptr, 10) != 0);
            reply(c.telem ? "OK TELEM 1" : "OK TELEM 0");
        } else if (cmd == "PANELS") {
            reply("PANELS " + api->panelStateString());
        } else if (cmd == "PING") {
            reply("PONG");
        } else if (cmd == "INFO") {
            std::string info =
                "ORGANOID-SIM API v1\n"
                "MOTOR <left> <right>  | wheel speed -100..100 %\n"
                "STOP                  | motors 0\n"
                "SOUND <path>          | play WAV at room center\n"
                "SOUND_STOP\n"
                "FRAME                 | one camera frame: FRAME <w> <h> <size>\\n + RGB bytes\n"
                "STREAM <0|1>          | camera stream 15 fps\n"
                "TELEM <0|1>           | pose telemetry 10 Hz\n"
                "PANELS                | touch panel state\n"
                "Events: PANEL <1..3> <DOWN|UP>, TELEM ..., SOUND_DONE <path>\n"
                "INFO_END";
            if (!sendAll(fd, info + "\n")) c.dead = true;
        } else {
            reply("ERR UNKNOWN CMD");
        }
    }
    void loop() {
        struct timeval tv;
        while (api->running) {
            fd_set rs;
            FD_ZERO(&rs);
            FD_SET(lfd, &rs);
            int maxfd = lfd;
            for (auto& [fd, c] : clients) { FD_SET(fd, &rs); if (fd > maxfd) maxfd = fd; }
            tv.tv_sec = 0; tv.tv_usec = 50000;
            int n = ::select(maxfd + 1, &rs, nullptr, nullptr, &tv);
            if (n < 0) { if (errno == EINTR) continue; break; }
            if (FD_ISSET(lfd, &rs)) {
                int cfd = ::accept(lfd, nullptr, nullptr);
                if (cfd >= 0) {
                    int fl = fcntl(cfd, F_GETFL, 0);
                    fcntl(cfd, F_SETFL, fl | O_NONBLOCK);
                    clients[cfd] = Client();
                    log->add(2, "client connected");
                }
            }
            std::vector<int> toErase;
            for (auto& [fd, c] : clients) {
                if (c.dead) { toErase.push_back(fd); continue; }
                if (!FD_ISSET(fd, &rs)) continue;
                char tmp[4096];
                bool closed = false;
                while (true) {
                    ssize_t n2 = ::recv(fd, tmp, sizeof(tmp), 0);
                    if (n2 > 0) {
                        c.buf.append(tmp, (size_t)n2);
                        size_t p;
                        while ((p = c.buf.find('\n')) != std::string::npos) {
                            std::string line = c.buf.substr(0, p);
                            c.buf.erase(0, p + 1);
                            if (!line.empty() && line.back() == '\r') line.pop_back();
                            handleLine(fd, c, line);
                            if (c.dead) break;
                        }
                    } else if (n2 == 0) { closed = true; break; }
                    else {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                        closed = true; break;
                    }
                    if (c.dead) break;
                }
                if (closed || c.dead) toErase.push_back(fd);
            }
            for (int fd : toErase) { ::close(fd); clients.erase(fd); }
            drainEvents();
            pushTouchEvents();
            double now = Logger::nowMs();
            if (now - lastTelem >= 100.0) { lastTelem = now; sendTelem(); }
            if (now - lastStream >= 66.0) {
                lastStream = now;
                for (auto& [fd, c] : clients)
                    if (!c.dead && c.stream) sendFrameTo(c, fd);
            }
            for (auto it = clients.begin(); it != clients.end();) {
                if (it->second.dead) { ::close(it->first); it = clients.erase(it); }
                else ++it;
            }
        }
    }
};

Server* g_srv = nullptr;

} // namespace

void Api::pushEvent(const std::string& e) {
    if (!g_srv) return;
    std::lock_guard<std::mutex> g(g_srv->evMu);
    g_srv->events.push_back(e);
    while (g_srv->events.size() > 200) g_srv->events.pop_front();
}

std::string Api::panelStateString() {
    std::lock_guard<std::mutex> g(mu);
    int m = touchMask;
    std::string s;
    for (int i = 0; i < 3; i++) { s += ((m & (1 << i)) ? "1" : "0"); if (i < 2) s += " "; }
    return s;
}

void Api::setTouchMask(int mask) {
    std::lock_guard<std::mutex> g(mu);
    touchMask = mask;
}

void startApiServer(Api& api, Logger& log) {
    api.running = true;
    g_srv = new Server();
    g_srv->api = &api;
    g_srv->log = &log;
    g_srv->thr = std::thread([srv = g_srv]() {
        srv->lfd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (srv->lfd < 0) { srv->log->add(2, "API: socket() failed"); return; }
        int one = 1;
        setsockopt(srv->lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        addr.sin_port = htons((uint16_t)srv->api->port);
        if (::bind(srv->lfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            srv->log->add(2, "API: bind 127.0.0.1:" + std::to_string(srv->api->port) + " failed");
            return;
        }
        if (::listen(srv->lfd, 8) < 0) { srv->log->add(2, "API: listen failed"); return; }
        srv->log->add(2, "API: listening on 127.0.0.1:" + std::to_string(srv->api->port));
        srv->loop();
    });
}

void stopApiServer(Api& api) {
    api.running = false;
    if (g_srv) {
        Server* srv = g_srv;
        g_srv = nullptr;
        for (auto& [fd, c] : srv->clients) ::shutdown(fd, SHUT_RDWR);
        if (srv->lfd >= 0) { ::shutdown(srv->lfd, SHUT_RDWR); ::close(srv->lfd); }
        if (srv->thr.joinable()) srv->thr.join();
        for (auto& [fd, c] : srv->clients) ::close(fd);
        delete srv;
    }
}
