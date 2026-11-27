#include "physics.hpp"

static const float ROOM_HALF = 2.0f;
static const float WHEEL_R   = 0.03f;
static const float HALF_L    = 0.05f;
static const float HALF_W    = 0.025f;
static const float AXLE_H    = WHEEL_R;
static const float H_COM     = 0.022f;
static const float MASS      = 0.4f;
static const float TRACK     = 0.07f;
static const float THETA_MAX = std::atan2(AXLE_H, HALF_L) * 0.98f;
static const float INER      = MASS * (H_COM * H_COM + (0.1f * 0.1f + 0.05f * 0.05f) / 12.0f);
static const float K_STAB    = MASS * 9.81f * H_COM;
static const float K_DAMP    = 1.8f * std::sqrt(K_STAB * INER);
static const float VMAX      = 1.2f;
static const float TAU_ACC   = 1.0f;
static const float TAU_DEC   = 0.15f;
static const float ROBOT_RAD = 0.06f;

void initScene(Scene& s) {
    s.boxes = {
        {{ 0.8f,  0.15f, -0.7f}, 0.20f, 0.15f, 0.20f, 0xC05050},
        {{ 1.2f,  0.30f,  0.6f}, 0.125f, 0.30f, 0.125f, 0x5070C0},
        {{-0.8f,  0.10f,  0.9f}, 0.25f, 0.10f, 0.25f, 0x50A060},
        {{-1.3f,  0.25f, -1.2f}, 0.10f, 0.25f, 0.10f, 0xB09040},
        {{ 0.1f,  0.05f, -1.5f}, 0.30f, 0.05f, 0.18f, 0x8060A0},
    };
    s.spheres   = { {{0.2f, 0.15f, 1.35f}, 0.15f, 0xC0B030} };
    s.cylinders = { {{-1.2f, 0.0f, -0.4f}, 0.12f, 0.50f, 0x40A0A0} };
    s.soundSource = {0.0f, 0.4f, 0.0f};
    resetRobot(s);
}

void resetRobot(Scene& s) {
    RobotState& r = s.robot;
    r.pos = {-1.5f, WHEEL_R, 0.0f};
    r.yaw = 0; r.pitch = 0; r.speed = 0; r.om = 0; r.yawRate = 0;
    r.spinL = 0; r.spinR = 0; r.vl = 0; r.vr = 0; r.fallen = false;
}

static void collideCircle(const Scene& s, Vec3& p, Vec3& v, float rad, bool& hit) {
    auto resolve = [&](float nx, float nz) {
        float vn = v.x * nx + v.z * nz;
        if (vn < 0) { v.x -= (1.3f * vn) * nx; v.z -= (1.3f * vn) * nz; }
        hit = true;
    };
    if (p.x >  ROOM_HALF - rad) { p.x =  ROOM_HALF - rad; resolve(-1, 0); }
    if (p.x < -ROOM_HALF + rad) { p.x = -ROOM_HALF + rad; resolve( 1, 0); }
    if (p.z >  ROOM_HALF - rad) { p.z =  ROOM_HALF - rad; resolve(0, -1); }
    if (p.z < -ROOM_HALF + rad) { p.z = -ROOM_HALF + rad; resolve(0,  1); }
    for (const Box& b : s.boxes) {
        float dx = p.x - b.c.x, dz = p.z - b.c.z;
        float cx = std::clamp(dx, -b.hx, b.hx), cz = std::clamp(dz, -b.hz, b.hz);
        float qx = dx - cx, qz = dz - cz;
        float d2 = qx * qx + qz * qz;
        if (d2 < rad * rad) {
            float nx, nz;
            if (d2 > 1e-12f) {
                float d = std::sqrt(d2);
                nx = qx / d; nz = qz / d;
            } else {
                float ox = b.hx + rad - std::fabs(dx), oz = b.hz + rad - std::fabs(dz);
                if (ox < oz) { nx = (dx > 0) ? 1 : -1; nz = 0; }
                else         { nx = 0; nz = (dz > 0) ? 1 : -1; }
            }
            p.x = b.c.x + cx + nx * (rad + 1e-4f);
            p.z = b.c.z + cz + nz * (rad + 1e-4f);
            resolve(nx, nz);
        }
    }
    for (const SphereObj& sp : s.spheres) {
        float dx = p.x - sp.c.x, dz = p.z - sp.c.z;
        float rr = rad + sp.r, d2 = dx * dx + dz * dz;
        if (d2 < rr * rr && d2 > 1e-12f) {
            float d = std::sqrt(d2);
            float nx = dx / d, nz = dz / d;
            p.x = sp.c.x + nx * rr;
            p.z = sp.c.z + nz * rr;
            resolve(nx, nz);
        }
    }
    for (const CylObj& cy : s.cylinders) {
        float dx = p.x - cy.c.x, dz = p.z - cy.c.z;
        float rr = rad + cy.r, d2 = dx * dx + dz * dz;
        if (d2 < rr * rr && d2 > 1e-12f) {
            float d = std::sqrt(d2);
            float nx = dx / d, nz = dz / d;
            p.x = cy.c.x + nx * rr;
            p.z = cy.c.z + nz * rr;
            resolve(nx, nz);
        }
    }
}

void stepPhysics(Scene& s, const MotorCmd& m, float dt) {
    RobotState& r = s.robot;
    float cl = std::clamp(m.l, -100.0f, 100.0f);
    float cr = std::clamp(m.r, -100.0f, 100.0f);
    float tl = cl / 100.0f * VMAX;
    float tr = cr / 100.0f * VMAX;

    bool fallen = r.fallen;
    float grip = fallen ? 0.15f : 1.0f;
    float s_old = r.speed;
    float target = 0.5f * (tl + tr);
    bool decel = (std::fabs(target) < std::fabs(s_old)) || (target * s_old < 0);
    float tau_m = decel ? TAU_DEC : TAU_ACC;
    r.speed += (target - r.speed) * std::min(1.0f, dt / tau_m) * grip;
    float a = (r.speed - s_old) / dt;

    r.yawRate = std::clamp((tr - tl) / TRACK * 0.8f * grip, -4.0f, 4.0f);
    r.yaw += r.yawRate * dt;
    float fx = std::sin(r.yaw), fz = std::cos(r.yaw);

    Vec3 vel = {fx * r.speed, 0, fz * r.speed};
    Vec3 p = r.pos;
    p.x += vel.x * dt;
    p.z += vel.z * dt;
    bool hit = false;
    collideCircle(s, p, vel, ROBOT_RAD, hit);
    r.pos = p;
    r.speed = vel.x * fx + vel.z * fz;
    if (hit) a = (r.speed - s_old) / dt;

    float th = r.pitch, om = r.om;
    if (!fallen) {
        float tau = MASS * a * H_COM * std::cos(th) - K_STAB * th - K_DAMP * om;
        om += tau / INER * dt;
        th += om * dt;
        if (std::fabs(th) >= THETA_MAX) {
            th = std::clamp(th, -THETA_MAX, THETA_MAX);
            om = 0;
            fallen = true;
        }
    } else {
        th = std::clamp(th, -THETA_MAX, THETA_MAX);
        om = 0;
    }
    r.pitch = th; r.om = om;
    r.fallen = fallen;

    r.spinL += tl / WHEEL_R * dt;
    r.spinR += tr / WHEEL_R * dt;
    r.vl = tl; r.vr = tr;
}
