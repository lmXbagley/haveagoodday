#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <chrono>

static double nowMs() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count() * 1000.0;
}

static bool sendAll(int fd, const std::string& data) {
    size_t off = 0;
    while (off < data.size()) {
        ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
        if (n <= 0) return false;
        off += (size_t)n;
    }
    return true;
}

int main(int argc, char** argv) {
    int port = 4242;
    if (argc > 1) port = atoi(argv[1]);

    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) { fprintf(stderr, "cannot open X display\n"); return 1; }
    int scr = DefaultScreen(dpy);
    Window win = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 60, 60, 440, 260, 1,
                                     WhitePixel(dpy, scr), BlackPixel(dpy, scr));
    XSelectInput(dpy, win, ExposureMask | KeyPressMask | KeyReleaseMask | StructureNotifyMask);
    Atom del = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, win, &del, 1);
    XStoreName(dpy, win, "ORGANOID DRIVE");
    XMapWindow(dpy, win);

    XFontStruct* font = XLoadQueryFont(dpy, "9x15");
    if (!font) font = XLoadQueryFont(dpy, "fixed");
    if (!font) { fprintf(stderr, "no X fonts found (apt install xfonts-base)\n"); return 1; }
    GC gc = XCreateGC(dpy, win, 0, nullptr);
    XSetFont(dpy, gc, font->fid);

    int sock = -1;
    bool connected = false;
    std::string rbuf;
    double lastSend = 0, lastConnectTry = 0;

    bool fwd = false, back = false, left = false, right = false;
    int speed = 50;
    int lastL = 0, lastR = 0;
    char telem[128] = "no telemetry yet";

    auto connectSim = [&]() {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t)port);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        if (::connect(sock, (sockaddr*)&addr, sizeof(addr)) == 0) {
            int fl = fcntl(sock, F_GETFL, 0);
            fcntl(sock, F_SETFL, fl | O_NONBLOCK);
            connected = true;
            rbuf.clear();
        } else {
            ::close(sock);
            sock = -1;
            connected = false;
        }
    };

    auto compute = [&](int& l, int& r) {
        float f = (fwd ? 1.0f : 0.0f) - (back ? 1.0f : 0.0f);
        float t = (right ? 1.0f : 0.0f) - (left ? 1.0f : 0.0f);
        l = (int)(speed * f - speed * t * 0.75f);
        r = (int)(speed * f + speed * t * 0.75f);
        if (l > 100) l = 100;
        if (l < -100) l = -100;
        if (r > 100) r = 100;
        if (r < -100) r = -100;
    };

    auto pushMotors = [&](bool force) {
        if (!connected) return;
        int l, r;
        compute(l, r);
        if (force || l != lastL || r != lastR || nowMs() - lastSend > 200.0) {
            char b[64];
            snprintf(b, sizeof(b), "MOTOR %d %d\n", l, r);
            if (sendAll(sock, b)) { lastL = l; lastR = r; lastSend = nowMs(); }
            else { ::close(sock); sock = -1; connected = false; }
        }
    };

    auto draw = [&]() {
        XSetForeground(dpy, gc, BlackPixel(dpy, scr));
        XFillRectangle(dpy, win, gc, 0, 0, 460, 280);
        unsigned long C_OK = 0x40E060, C_BAD = 0xE05050, C_DIM = 0xA0A0B0,
                      C_SPD = 0xF0D060, C_MOT = 0x60C0F0, C_TLM = 0x60F0A0;
        auto text = [&](int x, int y, const std::string& s, unsigned long c) {
            XSetForeground(dpy, gc, c);
            XDrawString(dpy, win, gc, x, y, s.c_str(), (int)s.size());
        };
        int y = 30;
        text(16, y, connected ? "ORGANOID DRIVE  [connected]"
                              : "ORGANOID DRIVE  [DISCONNECTED, retrying...]",
             connected ? C_OK : C_BAD);
        text(16, y + 30, "W/S - forward/back   A/D - turn", C_DIM);
        text(16, y + 50, "1..9,0 - speed 10..100%   Space - stop", C_DIM);
        text(16, y + 70, "Esc/Q - quit", C_DIM);
        char b[128];
        snprintf(b, sizeof(b), "SPEED: %d%%", speed);
        text(16, y + 110, b, C_SPD);
        snprintf(b, sizeof(b), "MOTORS: L %+d  R %+d", lastL, lastR);
        text(16, y + 140, b, C_MOT);
        text(16, y + 170, telem, C_TLM);
        XFlush(dpy);
    };

    bool running = true;
    double lastDraw = 0;
    while (running) {
        while (XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);
            if (ev.type == ClientMessage) {
                if ((Atom)ev.xclient.data.l[0] == del) running = false;
            } else if (ev.type == KeyPress || ev.type == KeyRelease) {
                bool down = ev.type == KeyPress;
                KeySym ks = XLookupKeysym(&ev.xkey, 0);
                switch (ks) {
                case XK_w: case XK_W: case XK_Up: fwd = down; break;
                case XK_s: case XK_S: case XK_Down: back = down; break;
                case XK_a: case XK_A: case XK_Left: left = down; break;
                case XK_d: case XK_D: case XK_Right: right = down; break;
                default: break;
                }
                if (down) {
                    if (ks >= XK_1 && ks <= XK_9) speed = (int)(ks - XK_0) * 10;
                    else if (ks == XK_0) speed = 100;
                    else if (ks == XK_plus || ks == XK_equal) { speed += 10; if (speed > 100) speed = 100; }
                    else if (ks == XK_minus) { speed -= 10; if (speed < 10) speed = 10; }
                    else if (ks == XK_space) { fwd = back = left = right = false; }
                    else if (ks == XK_Escape || ks == XK_q || ks == XK_Q) running = false;
                }
                pushMotors(false);
            }
        }

        if (!connected && nowMs() - lastConnectTry > 1500.0) {
            lastConnectTry = nowMs();
            connectSim();
            if (connected) pushMotors(true);
        }

        if (connected) {
            char tmp[4096];
            while (true) {
                ssize_t n = ::recv(sock, tmp, sizeof(tmp), 0);
                if (n > 0) {
                    rbuf.append(tmp, (size_t)n);
                    size_t p;
                    while ((p = rbuf.find('\n')) != std::string::npos) {
                        std::string line = rbuf.substr(0, p);
                        rbuf.erase(0, p + 1);
                        if (line.compare(0, 6, "TELEM ") == 0) {
                            float x, y2, z, yaw, pitch, vl, vr;
                            if (sscanf(line.c_str(), "TELEM %f %f %f %f %f %f %f",
                                       &x, &y2, &z, &yaw, &pitch, &vl, &vr) == 7)
                                snprintf(telem, sizeof(telem),
                                         "x=%.2f z=%.2f yaw=%.2f pitch=%.3f vl=%.2f vr=%.2f",
                                         x, z, yaw, pitch, vl, vr);
                        }
                    }
                } else if (n == 0) { ::close(sock); sock = -1; connected = false; break; }
                else break;
            }
            pushMotors(false);
        }

        if (nowMs() - lastDraw > 100.0) { lastDraw = nowMs(); draw(); }
        usleep(10000);
    }

    if (connected) {
        sendAll(sock, "STOP\n");
        ::close(sock);
    }
    XFreeGC(dpy, gc);
    XFreeFont(dpy, font);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    return 0;
}
