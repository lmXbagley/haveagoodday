#include "render.hpp"

static const float ROOM_HALF = 2.0f;
static const float WALL_H    = 2.5f;
static const float NEAR      = 0.02f;

struct Poly {
    Vec3 v[16];
    int n = 0;
    uint32_t col = 0;
    float depth = 0;
};

struct MeshBuilder {
    std::vector<Poly> polys;
    void reserve(size_t n) { polys.reserve(n); }
    void add(const Vec3* pts, int n, uint32_t col) {
        if (n < 3 || n > 16) return;
        Poly p;
        p.n = n;
        for (int i = 0; i < n; i++) p.v[i] = pts[i];
        p.col = col;
        polys.push_back(p);
    }
    void addBox(const Vec3& c, float hx, float hy, float hz, uint32_t col) {
        Vec3 pts[4];
        for (int side = 0; side < 6; side++) {
            switch (side) {
            case 0: pts[0] = {c.x-hx,c.y+hy,c.z+hz}; pts[1] = {c.x+hx,c.y+hy,c.z+hz}; pts[2] = {c.x+hx,c.y+hy,c.z-hz}; pts[3] = {c.x-hx,c.y+hy,c.z-hz}; break;
            case 1: pts[0] = {c.x-hx,c.y-hy,c.z-hz}; pts[1] = {c.x+hx,c.y-hy,c.z-hz}; pts[2] = {c.x+hx,c.y-hy,c.z+hz}; pts[3] = {c.x-hx,c.y-hy,c.z+hz}; break;
            case 2: pts[0] = {c.x+hx,c.y-hy,c.z+hz}; pts[1] = {c.x+hx,c.y-hy,c.z-hz}; pts[2] = {c.x+hx,c.y+hy,c.z-hz}; pts[3] = {c.x+hx,c.y+hy,c.z+hz}; break;
            case 3: pts[0] = {c.x-hx,c.y-hy,c.z-hz}; pts[1] = {c.x-hx,c.y-hy,c.z+hz}; pts[2] = {c.x-hx,c.y+hy,c.z+hz}; pts[3] = {c.x-hx,c.y+hy,c.z-hz}; break;
            case 4: pts[0] = {c.x+hx,c.y-hy,c.z-hz}; pts[1] = {c.x-hx,c.y-hy,c.z-hz}; pts[2] = {c.x-hx,c.y+hy,c.z-hz}; pts[3] = {c.x+hx,c.y+hy,c.z-hz}; break;
            default: pts[0] = {c.x-hx,c.y-hy,c.z+hz}; pts[1] = {c.x+hx,c.y-hy,c.z+hz}; pts[2] = {c.x+hx,c.y+hy,c.z+hz}; pts[3] = {c.x-hx,c.y+hy,c.z+hz}; break;
            }
            add(pts, 4, col);
        }
    }
    void addSphere(const Vec3& c, float r, uint32_t col, int lat = 8, int lon = 14) {
        for (int i = 0; i < lat; i++) {
            float t0 = 3.14159265f * (float)i / lat - 1.5707963f;
            float t1 = 3.14159265f * (float)(i + 1) / lat - 1.5707963f;
            for (int j = 0; j < lon; j++) {
                float p0 = 6.2831853f * (float)j / lon;
                float p1 = 6.2831853f * (float)(j + 1) / lon;
                Vec3 a  = {c.x + r*std::cos(t0)*std::cos(p0), c.y + r*std::sin(t0), c.z + r*std::cos(t0)*std::sin(p0)};
                Vec3 b  = {c.x + r*std::cos(t0)*std::cos(p1), c.y + r*std::sin(t0), c.z + r*std::cos(t0)*std::sin(p1)};
                Vec3 c2 = {c.x + r*std::cos(t1)*std::cos(p1), c.y + r*std::sin(t1), c.z + r*std::cos(t1)*std::sin(p1)};
                Vec3 d  = {c.x + r*std::cos(t1)*std::cos(p0), c.y + r*std::sin(t1), c.z + r*std::cos(t1)*std::sin(p0)};
                if (i == 0)          { Vec3 q[3] = {a, b, c2}; add(q, 3, col); }
                else if (i == lat-1) { Vec3 q[3] = {a, b, d};  add(q, 3, col); }
                else                 { Vec3 q[4] = {a, b, c2, d}; add(q, 4, col); }
            }
        }
    }
    void addCylinder(const Vec3& base, float r, float h, uint32_t col, int seg = 14) {
        for (int j = 0; j < seg; j++) {
            float p0 = 6.2831853f * (float)j / seg;
            float p1 = 6.2831853f * (float)(j + 1) / seg;
            Vec3 a  = {base.x + r*std::cos(p0), base.y,      base.z + r*std::sin(p0)};
            Vec3 b  = {base.x + r*std::cos(p1), base.y,      base.z + r*std::sin(p1)};
            Vec3 c2 = {base.x + r*std::cos(p1), base.y + h,  base.z + r*std::sin(p1)};
            Vec3 d  = {base.x + r*std::cos(p0), base.y + h,  base.z + r*std::sin(p0)};
            Vec3 q[4] = {a, b, c2, d};
            add(q, 4, col);
            Vec3 t0 = {base.x, base.y + h, base.z};
            Vec3 tri[3] = {t0, d, c2};
            add(tri, 3, shade(col, 0.8f));
            Vec3 bb = {base.x, base.y, base.z};
            Vec3 tri2[3] = {bb, a, b};
            add(tri2, 3, shade(col, 0.6f));
        }
    }
};

static void robotLoc2World(const RobotState& r, const Vec3& l, Vec3& w) {
    float cp = std::cos(r.pitch), sp = std::sin(r.pitch);
    float cy = std::cos(r.yaw),  sy = std::sin(r.yaw);
    float y1 = l.y * cp + l.z * sp;
    float z1 = l.z * cp - l.y * sp;
    w.x = r.pos.x + l.x * cy + z1 * sy;
    w.y = r.pos.y + y1;
    w.z = r.pos.z + z1 * cy - l.x * sy;
}

static void buildScene(const Scene& s, MeshBuilder& mb) {
    const float T = 0.5f;
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++) {
            float x0 = -ROOM_HALF + i * T, z0 = -ROOM_HALF + j * T;
            uint32_t col = ((i + j) & 1) ? 0x9A9AA2 : 0xB4B4BC;
            Vec3 q[4] = {{x0,0,z0},{x0+T,0,z0},{x0+T,0,z0+T},{x0,0,z0+T}};
            mb.add(q, 4, col);
        }
    uint32_t wc = 0xC8CDD8;
    Vec3 w0[4] = {{-ROOM_HALF,0,-ROOM_HALF},{ROOM_HALF,0,-ROOM_HALF},{ROOM_HALF,WALL_H,-ROOM_HALF},{-ROOM_HALF,WALL_H,-ROOM_HALF}};
    Vec3 w1[4] = {{ROOM_HALF,0,ROOM_HALF},{-ROOM_HALF,0,ROOM_HALF},{-ROOM_HALF,WALL_H,ROOM_HALF},{ROOM_HALF,WALL_H,ROOM_HALF}};
    Vec3 w2[4] = {{ROOM_HALF,0,-ROOM_HALF},{ROOM_HALF,0,ROOM_HALF},{ROOM_HALF,WALL_H,ROOM_HALF},{ROOM_HALF,WALL_H,-ROOM_HALF}};
    Vec3 w3[4] = {{-ROOM_HALF,0,ROOM_HALF},{-ROOM_HALF,0,-ROOM_HALF},{-ROOM_HALF,WALL_H,-ROOM_HALF},{-ROOM_HALF,WALL_H,ROOM_HALF}};
    mb.add(w0, 4, shade(wc, 0.92f));
    mb.add(w1, 4, shade(wc, 0.92f));
    mb.add(w2, 4, shade(wc, 0.85f));
    mb.add(w3, 4, shade(wc, 0.85f));

    for (const Box& b : s.boxes) mb.addBox(b.c, b.hx, b.hy, b.hz, b.color);
    for (const SphereObj& sp : s.spheres) mb.addSphere(sp.c, sp.r, sp.color);
    for (const CylObj& cy : s.cylinders) mb.addCylinder(cy.c, cy.r, cy.h, cy.color);

    const RobotState& r = s.robot;
    {
        float sh = std::clamp(1.0f - (std::fabs(r.pitch) / 0.6f) * 0.5f, 0.4f, 1.0f);
        Vec3 q[8];
        for (int k = 0; k < 8; k++) {
            float ang = 6.2831853f * (float)k / 8.0f;
            Vec3 lp = {0.05f * std::cos(ang), 0, 0.06f * std::sin(ang)};
            robotLoc2World(r, lp, q[k]);
            q[k].y = 0.003f;
        }
        Vec3 qa[4] = {q[0],q[1],q[2],q[3]};
        Vec3 qb[4] = {q[4],q[5],q[6],q[7]};
        mb.add(qa, 4, shade(0x101014, 0.9f * sh));
        mb.add(qb, 4, shade(0x101014, 0.9f * sh));
    }
    {
        Vec3 q[4];
        Vec3 c = {0, 0.015f, 0};
        float hx = 0.025f, hy = 0.015f, hz = 0.05f;
        uint32_t col = 0x2A4BC0;
        for (int side = 0; side < 6; side++) {
            Vec3 pts[4];
            switch (side) {
            case 0: pts[0]={c.x-hx,c.y+hy,c.z+hz}; pts[1]={c.x+hx,c.y+hy,c.z+hz}; pts[2]={c.x+hx,c.y+hy,c.z-hz}; pts[3]={c.x-hx,c.y+hy,c.z-hz}; break;
            case 1: pts[0]={c.x-hx,c.y-hy,c.z-hz}; pts[1]={c.x+hx,c.y-hy,c.z-hz}; pts[2]={c.x+hx,c.y-hy,c.z+hz}; pts[3]={c.x-hx,c.y-hy,c.z+hz}; break;
            case 2: pts[0]={c.x+hx,c.y-hy,c.z+hz}; pts[1]={c.x+hx,c.y-hy,c.z-hz}; pts[2]={c.x+hx,c.y+hy,c.z-hz}; pts[3]={c.x+hx,c.y+hy,c.z+hz}; break;
            case 3: pts[0]={c.x-hx,c.y-hy,c.z-hz}; pts[1]={c.x-hx,c.y-hy,c.z+hz}; pts[2]={c.x-hx,c.y+hy,c.z+hz}; pts[3]={c.x-hx,c.y+hy,c.z-hz}; break;
            case 4: pts[0]={c.x+hx,c.y-hy,c.z-hz}; pts[1]={c.x-hx,c.y-hy,c.z-hz}; pts[2]={c.x-hx,c.y+hy,c.z-hz}; pts[3]={c.x+hx,c.y+hy,c.z-hz}; break;
            default: pts[0]={c.x-hx,c.y-hy,c.z+hz}; pts[1]={c.x+hx,c.y-hy,c.z+hz}; pts[2]={c.x+hx,c.y+hy,c.z+hz}; pts[3]={c.x-hx,c.y+hy,c.z+hz}; break;
            }
            for (int i = 0; i < 4; i++) robotLoc2World(r, pts[i], q[i]);
            mb.add(q, 4, (side == 0) ? shade(col, 1.25f) : col);
        }
        for (int sideW = 0; sideW < 2; sideW++) {
            float wx = sideW ? 0.031f : -0.031f;
            float spin = sideW ? r.spinR : r.spinL;
            float woff = sideW ? 0.008f : -0.008f;
            for (int j = 0; j < 12; j++) {
                float p0 = 6.2831853f * (float)j / 12.0f;
                float p1 = 6.2831853f * (float)(j + 1) / 12.0f;
                Vec3 loc[4] = {
                    {wx, 0.03f*std::cos(p0+spin), 0.03f*std::sin(p0+spin)},
                    {wx, 0.03f*std::cos(p1+spin), 0.03f*std::sin(p1+spin)},
                    {wx+woff, 0.03f*std::cos(p1+spin), 0.03f*std::sin(p1+spin)},
                    {wx+woff, 0.03f*std::cos(p0+spin), 0.03f*std::sin(p0+spin)}
                };
                for (int i = 0; i < 4; i++) robotLoc2World(r, loc[i], q[i]);
                mb.add(q, 4, (j % 2) ? 0x202024 : 0x303036);
            }
            Vec3 hub[4];
            for (int i = 0; i < 4; i++) {
                float ang = 6.2831853f * (float)i / 4.0f + spin;
                Vec3 lp = {wx + (sideW ? 0.009f : -0.009f), 0.012f*std::cos(ang), 0.012f*std::sin(ang)};
                robotLoc2World(r, lp, hub[i]);
            }
            mb.add(hub, 4, 0xD0D0D8);
        }
        {
            Vec3 ccam = {0, 0.045f, 0.045f};
            for (int face = 0; face < 2; face++) {
                Vec3 pts[4];
                float zoff = face ? 0.012f : -0.012f;
                for (int i = 0; i < 4; i++) {
                    Vec3 lp = {ccam.x + ((i & 1) ? 0.012f : -0.012f), ccam.y + ((i & 2) ? 0.012f : -0.012f), ccam.z + zoff};
                    robotLoc2World(r, lp, pts[i]);
                }
                mb.add(pts, 4, face ? 0x30C050 : 0x209040);
            }
        }
    }
}

struct VPt { float x, y, z; };

static inline void projectView(const VPt& v, float F, float cx, float h, float& sx, float& sy) {
    float iz = 1.0f / std::max(v.z, 1e-6f);
    sx = cx + F * v.x * iz;
    sy = (0.5f * h) - F * v.y * iz;
}

void renderScene(const Scene& s, const Camera& cam, FrameTarget& tgt) {
    tgt.clear(0x0E0E12);
    int w = tgt.w, h = tgt.h;
    MeshBuilder mb;
    mb.reserve(1024);
    buildScene(s, mb);

    float cy = std::cos(cam.yaw), sy = std::sin(cam.yaw);
    float cp = std::cos(cam.pitch), sp = std::sin(cam.pitch);
    float F = 0.5f * (float)h / std::tan(cam.fov * 0.5f);
    float cx = 0.5f * w;

    struct Draw { int idx; float depth; std::vector<VPt> vs; };
    std::vector<Draw> order;
    order.reserve(mb.polys.size());

    for (size_t i = 0; i < mb.polys.size(); i++) {
        const Poly& p = mb.polys[i];
        std::vector<VPt> in, out;
        in.reserve(p.n);
        for (int k = 0; k < p.n; k++) {
            const Vec3& wp = p.v[k];
            float dx = wp.x - cam.pos.x, dy = wp.y - cam.pos.y, dz = wp.z - cam.pos.z;
            float x1 = dx * cy - dz * sy;
            float z1 = dx * sy + dz * cy;
            float y2 = dy * cp - z1 * sp;
            float z2 = dy * sp + z1 * cp;
            in.push_back({x1, y2, z2});
        }
        bool anyFront = false, allFront = true;
        for (auto& v : in) { if (v.z > NEAR) anyFront = true; else allFront = false; }
        if (!anyFront) continue;
        if (!allFront) {
            size_t cnt = in.size();
            for (size_t k = 0; k < cnt; k++) {
                const VPt& A = in[k];
                const VPt& B = in[(k + 1) % cnt];
                bool aIn = A.z > NEAR, bIn = B.z > NEAR;
                if (aIn) out.push_back(A);
                if (aIn != bIn) {
                    float t = (NEAR - A.z) / (B.z - A.z);
                    out.push_back({A.x + (B.x - A.x) * t, A.y + (B.y - A.y) * t, NEAR});
                }
            }
        } else out = in;
        if (out.size() < 3) continue;

        float zsum = 0;
        for (auto& v : out) zsum += v.z;
        Draw d;
        d.idx = (int)i;
        d.depth = zsum / (float)out.size();
        d.vs = std::move(out);
        order.push_back(d);
    }

    std::sort(order.begin(), order.end(), [](const Draw& a, const Draw& b) { return a.depth > b.depth; });

    const Vec3 light = vnorm({-0.4f, 0.85f, 0.3f});
    std::vector<float> xs;
    for (const Draw& d : order) {
        const Poly& p = mb.polys[d.idx];
        const std::vector<VPt>& vs = d.vs;
        Vec3 e1 = p.v[1] - p.v[0], e2 = p.v[2] - p.v[0];
        Vec3 nrm = cross(e1, e2);
        float nl = vlen(nrm);
        if (nl > 1e-9f) {
            nrm = nrm * (1.0f / nl);
            Vec3 toCam = cam.pos - p.v[0];
            if (dot(nrm, toCam) < 0) nrm = nrm * -1.0f;
        } else nrm = {0, 1, 0};
        float f = 0.35f + 0.65f * std::max(0.0f, dot(nrm, light));
        uint32_t col = shade(p.col, f);

        float ymin = 1e9f, ymax = -1e9f;
        for (auto& v : vs) {
            float sxx, syy;
            projectView(v, F, cx, (float)h, sxx, syy);
            if (syy < ymin) ymin = syy;
            if (syy > ymax) ymax = syy;
        }
        int iy0 = std::max(0, (int)std::floor(ymin));
        int iy1 = std::min(h - 1, (int)std::ceil(ymax));
        size_t cnt = vs.size();
        for (int yy = iy0; yy <= iy1; yy++) {
            float yc = yy + 0.5f;
            xs.clear();
            for (size_t k = 0; k < cnt; k++) {
                float ax, ay, bx, by;
                projectView(vs[k], F, cx, (float)h, ax, ay);
                projectView(vs[(k + 1) % cnt], F, cx, (float)h, bx, by);
                if ((ay <= yc) != (by <= yc)) {
                    float t = (yc - ay) / (by - ay);
                    xs.push_back(ax + (bx - ax) * t);
                }
            }
            if (xs.size() < 2) continue;
            std::sort(xs.begin(), xs.end());
            int x0 = std::max(0, (int)std::ceil(xs.front() - 0.5f));
            int x1 = std::min(w - 1, (int)std::floor(xs.back() - 0.5f));
            uint32_t* row = &tgt.pix[(size_t)yy * w];
            for (int xx = x0; xx <= x1; xx++) row[xx] = col;
        }
    }
}
