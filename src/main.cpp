#include "sim.hpp"
#include "physics.hpp"
#include "render.hpp"
#include "api.hpp"
#include "audio.hpp"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

static const int  PANEL_W   = 300;
static const int  DEF_W     = 1024;
static const int  DEF_H     = 640;
static const int  CAM_W     = 128;
static const int  CAM_H     = 96;

struct Rect { int x, y, w, h; bool in(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; } };

static double g_t0 = 0;

static double tnow() { return Logger::nowMs(); }

struct Ui {
    Rect camView;
    Rect logger;
    Rect touch[3];
    Rect pathField;
    Rect btn[4];
};

int main(int argc, char** argv) {
    g_t0 = tnow();
    Api api;
    api.port = 4242;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) api.port = atoi(argv[++i]);
    }

    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) { fprintf(stderr, "cannot open X display\n"); return 1; }
    int scr = DefaultScreen(dpy);
    Window win = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 40, 40, DEF_W, DEF_H, 1,
                                     BlackPixel(dpy, scr), WhitePixel(dpy, scr));
    XSelectInput(dpy, win, ExposureMask | StructureNotifyMask | ButtonPressMask |
                 ButtonReleaseMask | PointerMotionMask | KeyPressMask | KeyReleaseMask);
    Atom del = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, win, &del, 1);
    XStoreName(dpy, win, "ORGANOID SIM");
    XMapWindow(dpy, win);

    XFontStruct* font = XLoadQueryFont(dpy, "9x15");
    if (!font) font = XLoadQueryFont(dpy, "fixed");
    GC gc = XCreateGC(dpy, win, 0, nullptr);
    if (font) XSetFont(dpy, gc, font->fid);
    else fprintf(stderr, "warning: X fonts not found (xfonts-base) — panel text will be hidden\n");

    auto col = [&](int r, int g2, int b) {
        XColor c;
        c.red = r * 257; c.green = g2 * 257; c.blue = b * 257;
        XAllocColor(dpy, DefaultColormap(dpy, scr), &c);
        return c.pixel;
    };
    unsigned long C_WHITE = col(230, 230, 235), C_GRAY = col(150, 150, 160),
                  C_GREEN = col(110, 220, 120), C_CYAN = col(110, 190, 230),
                  C_UI = col(160, 160, 170), C_RED = col(230, 90, 90),
                  C_YEL = col(220, 200, 90);

    FrameTarget fb, camFB;
    camFB.resize(CAM_W, CAM_H);
    XImage* img = nullptr;
    int curW = 0, curH = 0;

    Scene scene;
    std::mutex sceneMu;
    initScene(scene);

    Logger log;
    AudioEngine audio;
    audio.setListener([&]() { std::lock_guard<std::mutex> g(sceneMu); return scene.robot.pos; },
                      [&]() { std::lock_guard<std::mutex> g(sceneMu); return scene.robot.yaw; });
    audio.onDone = [&](const std::string& p) { api.pushEvent("SOUND_DONE " + p); };

    api.snapshot = [&]() { std::lock_guard<std::mutex> g(sceneMu); return scene.robot; };
    api.camFrame = [&](std::vector<uint8_t>& out, int& w, int& h) {
        out.resize((size_t)CAM_W * CAM_H * 3);
        for (int i = 0; i < CAM_W * CAM_H; i++) {
            uint32_t px = camFB.pix[i];
            out[i * 3 + 0] = (px >> 16) & 0xFF;
            out[i * 3 + 1] = (px >> 8) & 0xFF;
            out[i * 3 + 2] = px & 0xFF;
        }
        w = CAM_W; h = CAM_H;
        return true;
    };
    api.playSound = [&](const std::string& path, std::string& err) {
        if (!audio.loadFile(path, err)) { log.add(2, "API sound load: " + err); return false; }
        if (!audio.play(err)) { log.add(2, "API sound play: " + err); return false; }
        return true;
    };
    api.stopSound = [&]() { audio.stop(); };

    startApiServer(api, log);

    Ui ui;
    bool dragging = false, mousePressed = false;
    int lastMX = 0, lastMY = 0;
    float camYaw = 0.7f, camPitch = 0.55f, camDist = 1.6f;
    bool followCam = true;
    bool keyDrive[4] = {false, false, false, false};
    bool pathFocus = false;
    std::string pathInput;
    int touchSent = 0;

    auto layout = [&](int W, int) {
        int x0 = W - PANEL_W;
        ui.camView = {x0 + 12, 34, 256, 192};
        ui.logger = {x0 + 12, 250, 276, 200};
        for (int i = 0; i < 3; i++) ui.touch[i] = {x0 + 12 + i * 92, 460, 84, 56};
        ui.pathField = {x0 + 12, 530, 276, 26};
        for (int i = 0; i < 4; i++) ui.btn[i] = {x0 + 12 + i * 70, 566, 66, 26};
    };

    auto updateTouch = [&]() {
        int mask = 0;
        if (mousePressed)
            for (int i = 0; i < 3; i++) if (ui.touch[i].in(lastMX, lastMY)) mask |= (1 << i);
        api.setTouchMask(mask);
        int diff = mask ^ touchSent;
        for (int i = 0; i < 3; i++)
            if (diff & (1 << i))
                log.add(2, std::string("touch S") + std::to_string(i + 1) + ((mask & (1 << i)) ? " DOWN" : " UP"));
        touchSent = mask;
    };

    auto drawText = [&](int x, int y, const std::string& s, unsigned long c) {
        if (!font || s.empty()) return;
        XSetForeground(dpy, gc, c);
        XDrawString(dpy, win, gc, x, y, s.c_str(), (int)s.size());
    };

    auto fillRectFb = [&](FrameTarget& t, Rect r, uint32_t c) {
        for (int y = r.y; y < r.y + r.h && y < t.h; y++) {
            if (y < 0) continue;
            uint32_t* row = &t.pix[(size_t)y * t.w];
            for (int x = r.x; x < r.x + r.w && x < t.w; x++) {
                if (x < 0) continue;
                row[x] = c;
            }
        }
    };
    auto borderRectFb = [&](FrameTarget& t, Rect r, uint32_t c) {
        fillRectFb(t, {r.x, r.y, r.w, 1}, c);
        fillRectFb(t, {r.x, r.y + r.h - 1, r.w, 1}, c);
        fillRectFb(t, {r.x, r.y, 1, r.h}, c);
        fillRectFb(t, {r.x + r.w - 1, r.y, 1, r.h}, c);
    };

    bool running = true;
    double lastT = tnow();
    double lastFrame = lastT;
    int fps = 0, fpsCnt = 0;

    auto robotCam = [&](Camera& cam) {
        std::lock_guard<std::mutex> g(sceneMu);
        const RobotState& r = scene.robot;
        float cpy = std::cos(r.pitch), spy = std::sin(r.pitch);
        float cyw = std::cos(r.yaw), syw = std::sin(r.yaw);
        float y1 = 0.055f * cpy + 0.05f * spy;
        float z1 = 0.05f * cpy - 0.055f * spy;
        cam.pos = {r.pos.x + z1 * syw, r.pos.y + y1, r.pos.z + z1 * cyw};
        cam.yaw = r.yaw;
        cam.pitch = std::clamp(r.pitch, -1.2f, 1.2f);
        cam.fov = 1.3f;
    };

    while (running) {
        while (XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);
            if (ev.type == ClientMessage) {
                if ((Atom)ev.xclient.data.l[0] == del) running = false;
            }
            else if (ev.type == ConfigureNotify) {
                curW = 0;
            } else if (ev.type == ButtonPress) {
                lastMX = ev.xbutton.x; lastMY = ev.xbutton.y;
                if (ev.xbutton.button == Button1) {
                    bool inPanel = lastMX >= curW - PANEL_W;
                    bool inTouch = false;
                    for (int i = 0; i < 3; i++) if (ui.touch[i].in(lastMX, lastMY)) inTouch = true;
                    if (inTouch) { mousePressed = true; updateTouch(); }
                    else if (!inPanel) dragging = true;
                    else if (ui.pathField.in(lastMX, lastMY)) pathFocus = true;
                    else pathFocus = false;
                    for (int i = 0; i < 4; i++) {
                        if (ui.btn[i].in(lastMX, lastMY)) {
                            std::string err;
                            if (i == 0) {
                                if (!audio.recording() && audio.startRecording(err)) {
                                    log.add(2, "recording started");
                                } else if (!err.empty()) log.add(2, "rec: " + err);
                            } else if (i == 1) {
                                if (audio.recording()) { audio.stopRecording(); log.add(2, "recording stop..."); }
                            } else if (i == 2) {
                                if (audio.play(err)) log.add(2, "play: " + audio.loadedPath);
                                else log.add(2, "play: " + err);
                            } else {
                                audio.stop();
                                log.add(2, "playback stop");
                            }
                        }
                    }
                } else if (ev.xbutton.button == Button4) { camDist *= 0.9f; if (camDist < 0.35f) camDist = 0.35f; }
                else if (ev.xbutton.button == Button5) { camDist *= 1.1f; if (camDist > 6.0f) camDist = 6.0f; }
            } else if (ev.type == ButtonRelease) {
                if (ev.xbutton.button == Button1) { dragging = false; mousePressed = false; updateTouch(); }
            } else if (ev.type == MotionNotify) {
                int nx = ev.xmotion.x, ny = ev.xmotion.y;
                if (dragging) {
                    camYaw += (nx - lastMX) * 0.006f;
                    camPitch += (ny - lastMY) * 0.006f;
                    if (camPitch < -0.1f) camPitch = -0.1f;
                    if (camPitch > 1.45f) camPitch = 1.45f;
                }
                lastMX = nx; lastMY = ny;
                if (mousePressed) updateTouch();
            } else if (ev.type == KeyPress) {
                KeySym ks = XLookupKeysym(&ev.xkey, 0);
                if (pathFocus) {
                    char txt[16] = {0};
                    int n = XLookupString(&ev.xkey, txt, sizeof(txt) - 1, &ks, nullptr);
                    if (n > 0 && txt[0] >= 32 && txt[0] < 127 && txt[0] != '\n') pathInput += txt;
                    else if (ks == XK_BackSpace && !pathInput.empty()) pathInput.pop_back();
                    else if (ks == XK_Return || ks == XK_KP_Enter) {
                        std::string err;
                        if (!pathInput.empty() && audio.loadFile(pathInput, err)) log.add(2, "loaded: " + pathInput);
                        else log.add(2, "load: " + err);
                    } else if (ks == XK_Escape) pathFocus = false;
                } else {
                    switch (ks) {
                    case XK_Escape: running = false; break;
                    case XK_Up: case XK_w: keyDrive[0] = true; break;
                    case XK_Down: case XK_s: keyDrive[1] = true; break;
                    case XK_Left: case XK_a: keyDrive[2] = true; break;
                    case XK_Right: case XK_d: keyDrive[3] = true; break;
                    case XK_space: break;
                    case XK_f: followCam = !followCam; break;
                    case XK_r: { std::lock_guard<std::mutex> g(sceneMu); resetRobot(scene); log.add(2, "robot reset"); } break;
                    }
                }
            } else if (ev.type == KeyRelease) {
                KeySym ks = XLookupKeysym(&ev.xkey, 0);
                switch (ks) {
                case XK_Up: case XK_w: keyDrive[0] = false; break;
                case XK_Down: case XK_s: keyDrive[1] = false; break;
                case XK_Left: case XK_a: keyDrive[2] = false; break;
                case XK_Right: case XK_d: keyDrive[3] = false; break;
                }
            }
        }

        if (curW == 0) {
            Window r;
            int x, y;
            unsigned int bw, d;
            XGetGeometry(dpy, win, &r, &x, &y, (unsigned int*)&curW, (unsigned int*)&curH, &bw, &d);
            if (curW < 640) curW = 640;
            if (curH < 480) curH = 480;
            fb.resize(curW, curH);
            if (img) { img->data = nullptr; XDestroyImage(img); }
            img = XCreateImage(dpy, DefaultVisual(dpy, scr), (unsigned int)DefaultDepth(dpy, scr),
                               ZPixmap, 0, (char*)fb.pix.data(), (unsigned int)curW, (unsigned int)curH, 32, curW * 4);
            if (!img) { fprintf(stderr, "XCreateImage failed\n"); return 1; }
            layout(curW, curH);
        }

        double now = tnow();
        float dt = (float)((now - lastT) / 1000.0);
        if (dt > 0.05f) dt = 0.05f;
        lastT = now;

        MotorCmd mc;
        {
            std::lock_guard<std::mutex> g(api.mu);
            mc = api.motors;
        }
        if (keyDrive[0] || keyDrive[1] || keyDrive[2] || keyDrive[3]) {
            float f = (keyDrive[0] ? 60.0f : 0.0f) - (keyDrive[1] ? 60.0f : 0.0f);
            float t = (keyDrive[3] ? 45.0f : 0.0f) - (keyDrive[2] ? 45.0f : 0.0f);
            mc.l = f - t; mc.r = f + t;
        }
        {
            std::lock_guard<std::mutex> g(sceneMu);
            stepPhysics(scene, mc, dt * 0.5f);
            stepPhysics(scene, mc, dt * 0.5f);
        }

        Camera cam;
        {
            std::lock_guard<std::mutex> g(sceneMu);
            Vec3 t = followCam ? scene.robot.pos : Vec3(0, 0.3f, 0);
            float cp2 = std::cos(camPitch);
            cam.pos = {t.x + camDist * cp2 * std::sin(camYaw),
                       t.y + 0.06f + camDist * std::sin(camPitch),
                       t.z + camDist * cp2 * std::cos(camYaw)};
            cam.yaw = camYaw; cam.pitch = camPitch; cam.fov = 1.15f;
            renderScene(scene, cam, fb);
        }
        Camera rcam;
        robotCam(rcam);
        {
            std::lock_guard<std::mutex> g(sceneMu);
            renderScene(scene, rcam, camFB);
        }

        fpsCnt++;
        if (now - lastFrame >= 1000.0) { fps = fpsCnt; fpsCnt = 0; lastFrame = now; }

        std::string recErr = audio.pollError();
        if (!recErr.empty()) log.add(2, "rec: " + recErr);
        std::string recFile = audio.pollFinished();
        if (!recFile.empty()) {
            log.add(2, "saved: " + recFile);
            std::string err;
            if (audio.loadFile(recFile, err)) pathInput = recFile;
        }

        int x0 = curW - PANEL_W;
        fillRectFb(fb, {x0, 0, PANEL_W, curH}, 0x181820u);
        fillRectFb(fb, {x0, 0, 2, curH}, 0x2A2A36u);
        borderRectFb(fb, ui.camView, 0x3A3A4Au);
        {
            for (int y = 0; y < ui.camView.h; y++) {
                uint32_t* dst = &fb.pix[(size_t)(ui.camView.y + y) * curW];
                const uint32_t* src = &camFB.pix[(size_t)(y * CAM_H / ui.camView.h) * CAM_W];
                for (int x = 0; x < ui.camView.w; x++)
                    dst[ui.camView.x + x] = src[(size_t)x * CAM_W / ui.camView.w];
            }
        }
        fillRectFb(fb, ui.logger, 0x101016u);
        borderRectFb(fb, ui.logger, 0x3A3A4Au);
        fillRectFb(fb, ui.pathField, 0x101016u);
        borderRectFb(fb, ui.pathField, pathFocus ? 0x60A0FFu : 0x3A3A4Au);
        int mask = 0;
        { std::lock_guard<std::mutex> g(api.mu); mask = api.touchMask; }
        for (int i = 0; i < 3; i++) {
            uint32_t bg = (mask & (1 << i)) ? 0x2F7F3Fu : 0x20202Cu;
            fillRectFb(fb, ui.touch[i], bg);
            borderRectFb(fb, ui.touch[i], 0x4A4A5Au);
        }
        const char* btnNames[4] = {"REC", "STOP", "PLAY", "STOP"};
        for (int i = 0; i < 4; i++) {
            bool active = (i == 0 && audio.recording());
            fillRectFb(fb, ui.btn[i], active ? 0x7F2F2Fu : 0x20202Cu);
            borderRectFb(fb, ui.btn[i], 0x4A4A5Au);
        }

        XPutImage(dpy, win, gc, img, 0, 0, 0, 0, (unsigned int)curW, (unsigned int)curH);

        XRectangle clip;
        clip.x = (short)ui.logger.x; clip.y = (short)ui.logger.y;
        clip.width = (unsigned short)ui.logger.w; clip.height = (unsigned short)ui.logger.h;
        XSetClipRectangles(dpy, gc, 0, 0, &clip, 1, Unsorted);
        {
            std::lock_guard<std::mutex> g(log.mu);
            int yy = ui.logger.y + ui.logger.h - 6;
            int lh = 15;
            for (auto it = log.entries.rbegin(); it != log.entries.rend() && yy > ui.logger.y + 2; ++it) {
                char ts[32];
                snprintf(ts, sizeof(ts), "%6.1f ", (it->t - g_t0) / 1000.0);
                unsigned long c = it->dir == 0 ? C_GREEN : (it->dir == 1 ? C_CYAN : C_UI);
                const char* arrow = it->dir == 0 ? "<< " : (it->dir == 1 ? ">> " : " * ");
                drawText(ui.logger.x + 4, yy, ts + (std::string(arrow) + it->msg), c);
                yy -= lh;
            }
        }
        XSetClipMask(dpy, gc, None);

        drawText(x0 + 12, 24, "ORGANOID SIM", C_WHITE);
        drawText(ui.camView.x + 4, ui.camView.y + 205, "robot cam", C_GRAY);
        for (int i = 0; i < 3; i++) {
            std::string s = std::string("S") + std::to_string(i + 1);
            drawText(ui.touch[i].x + 34, ui.touch[i].y + 33, s,
                     (mask & (1 << i)) ? C_WHITE : C_GRAY);
        }
        drawText(ui.pathField.x + 6, ui.pathField.y + 18,
                 pathFocus ? (pathInput + "_") : (pathInput.empty() ? "path to wav + Enter" : pathInput),
                 C_YEL);
        for (int i = 0; i < 4; i++)
            drawText(ui.btn[i].x + 18, ui.btn[i].y + 18, btnNames[i], C_GRAY);
        if (audio.recording()) {
            XSetForeground(dpy, gc, C_RED);
            XFillRectangle(dpy, win, gc, ui.pathField.x + ui.pathField.w - 14, ui.pathField.y + 7, 8, 8);
        }
        {
            std::lock_guard<std::mutex> g(sceneMu);
            const RobotState& r = scene.robot;
            char st[160];
            snprintf(st, sizeof(st), "L%+.0f R%+.0f | %.2f %.2f %.2f | y%.2f p%.2f%s | %d fps",
                     mc.l, mc.r, r.pos.x, r.pos.y, r.pos.z, r.yaw, r.pitch,
                     r.fallen ? " FALLEN" : "", fps);
            drawText(x0 + 12, curH - 14, st, C_WHITE);
        }
        XFlush(dpy);

        double spent = tnow() - now;
        if (spent < 15.0) usleep((useconds_t)(15.0 - spent) * 1000);
    }

    audio.stop();
    if (audio.recording()) audio.stopRecording();
    stopApiServer(api);
    if (img) { img->data = nullptr; XDestroyImage(img); }
    XFreeGC(dpy, gc);
    if (font) XFreeFont(dpy, font);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    return 0;
}
