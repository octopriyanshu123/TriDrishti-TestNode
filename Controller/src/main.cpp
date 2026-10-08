/*
 * Differential-Drive Robot  -  PID Path Follower  -  OpenGL (freeglut)
 * ====================================================================
 *  Blue   : robot body, wheels and its centre / heading line (arrow)
 *  Yellow : target path - a straight line OR a random closed curve
 *
 *  linear.x is constant (5.0); angular.z = v*kappa (curvature feed-forward) - PID(error)
 *  cmd_vel is drawn on screen and printed to the terminal at 10 Hz.
 *
 *  Controls
 *    [Random Curve] button / C : turn the target into a new random closed curvy loop
 *    [Polyline Weld] button / P : random weld of straight legs with 30/60/90 deg corners
 *                                 (track -> slow into corner -> stop -> pivot in place -> next leg)
 *    L                          : back to a straight line
 *    L-drag on endpoint         : move that endpoint          (line mode)
 *    L-drag on the line         : move the whole line          (line mode)
 *    L-drag on empty space      : draw a new straight line (start -> end = travel direction)
 *    Mouse wheel, LEFT/RIGHT    : rotate line   UP/DOWN : shift line   (line mode)
 *    T : random straight line   F : flip travel direction
 *    R-click : place robot at cursor     R : reset robot
 *    TAB : select gain   + / - : change gain     SPACE : pause   ESC : quit
 */
#include <GL/freeglut.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

// ------------------------------------------------------------------ config
namespace cfg {
constexpr int    WIN_W = 1400, WIN_H = 640;
constexpr double SCALE = 8.0;                       // px per world unit
constexpr double WORLD_W = WIN_W / SCALE;           // 175
constexpr double WORLD_H = WIN_H / SCALE;           // 80
constexpr double LINEAR_VEL  = 5.0;                 // constant forward speed
constexpr double MAX_ANG_VEL = 3.0;                 // rad/s clamp
constexpr double WHEEL_BASE  = 6.0;
constexpr double BODY_HL = 5.0, BODY_HW = 3.5;
// polyline-weld corner sequencing
constexpr double A_DEC = 4.0;         // decel into a corner (units/s^2)
constexpr double A_ACC = 4.0;         // accel out of a corner (units/s^2)
constexpr double V_CREEP = 0.4;       // minimum speed while closing on a corner
constexpr double STOP_TOL = 0.05;     // "at the corner" tolerance along the leg
constexpr double PIVOT_TOL_DEG = 1.0; // heading tolerance to finish a pivot
constexpr double PIVOT_SETTLE = 0.25; // s inside tolerance before moving on
constexpr double MAX_PIVOT_W = 2.0;   // rad/s clamp while pivoting
}  // namespace cfg

constexpr double PI = 3.14159265358979323846;
const float BLUE[3]   = {0.36f, 0.66f, 0.96f};
const float YELLOW[3] = {1.00f, 0.70f, 0.10f};
const float RED[3]    = {0.95f, 0.35f, 0.35f};

struct Vec2 { double x = 0, y = 0; };

static double wrapAngle(double a) {
    a = std::fmod(a + PI, 2.0 * PI);
    if (a < 0) a += 2.0 * PI;
    return a - PI;
}
static double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
static double dist(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
static double deg2rad(double d) { return d * PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / PI; }

// Result of asking a path "where am I relative to you?"
struct PathQuery {
    Vec2   closest;       // closest point on path
    double tangent = 0;   // path direction at that point (rad)
    double cte = 0;       // signed cross-track error, + = robot LEFT of path
    double curvature = 0; // 1/radius, + = path turns left
};

// ------------------------------------------------------------------ PID
class PID {
public:
    double kp, ki, kd, iLimit;
    double p = 0, i = 0, d = 0;

    PID(double kp_, double ki_, double kd_, double iLimit_ = 1.5)
        : kp(kp_), ki(ki_), kd(kd_), iLimit(iLimit_) {}

    void reset() { integral_ = 0; hasPrev_ = false; p = i = d = 0; }

    double update(double err, double dt) {
        integral_ = clampd(integral_ + err * dt, -iLimit, iLimit);   // anti-windup
        double deriv = hasPrev_ ? wrapAngle(err - prev_) / dt : 0.0;
        prev_ = err; hasPrev_ = true;
        p = kp * err; i = ki * integral_; d = kd * deriv;
        return p + i + d;
    }

private:
    double integral_ = 0, prev_ = 0;
    bool hasPrev_ = false;
};

// ------------------------------------------------------------------ straight target line
class TargetLine {
public:
    Vec2 p1, p2;
    TargetLine(Vec2 a = {5, 42}, Vec2 b = {170, 32}) : p1(a), p2(b) {}

    double angle()  const { return std::atan2(p2.y - p1.y, p2.x - p1.x); }
    Vec2   dir()    const { double a = angle(); return {std::cos(a), std::sin(a)}; }
    Vec2   normal() const { Vec2 d = dir(); return {-d.y, d.x}; }          // left normal
    Vec2   mid()    const { return {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2}; }

    double signedDistance(double x, double y) const {
        Vec2 n = normal(); return (x - p1.x) * n.x + (y - p1.y) * n.y;
    }
    double project(double x, double y) const {
        Vec2 d = dir(); return (x - p1.x) * d.x + (y - p1.y) * d.y;
    }
    Vec2 pointAt(double s) const { Vec2 d = dir(); return {p1.x + s * d.x, p1.y + s * d.y}; }

    PathQuery query(double x, double y) const {
        PathQuery q;
        q.closest = pointAt(project(x, y));
        q.tangent = angle();
        q.cte = signedDistance(x, y);
        q.curvature = 0.0;
        return q;
    }

    bool clipRange(double& tmin, double& tmax) const {
        Vec2 d = dir();
        tmin = -1e9; tmax = 1e9;
        const double o[2] = {p1.x, p1.y}, dd[2] = {d.x, d.y}, hi[2] = {cfg::WORLD_W, cfg::WORLD_H};
        for (int k = 0; k < 2; ++k) {
            if (std::fabs(dd[k]) < 1e-9) {
                if (o[k] < 0 || o[k] > hi[k]) return false;
            } else {
                double t1 = (0 - o[k]) / dd[k], t2 = (hi[k] - o[k]) / dd[k];
                tmin = std::max(tmin, std::min(t1, t2));
                tmax = std::min(tmax, std::max(t1, t2));
            }
        }
        return tmin < tmax;
    }

    void rotate(double da) {
        Vec2 c = mid();
        for (Vec2* p : {&p1, &p2}) {
            double x = p->x - c.x, y = p->y - c.y;
            p->x = c.x + x * std::cos(da) - y * std::sin(da);
            p->y = c.y + x * std::sin(da) + y * std::cos(da);
        }
        clampToWorld();
    }
    void shift(double dx, double dy) {
        p1.x += dx; p1.y += dy; p2.x += dx; p2.y += dy;
        clampToWorld();
    }
    void clampToWorld() {
        for (Vec2* p : {&p1, &p2}) {
            p->x = clampd(p->x, 2, cfg::WORLD_W - 2);
            p->y = clampd(p->y, 2, cfg::WORLD_H - 2);
        }
    }
};

// ------------------------------------------------------------------ random closed curve
/*
 * r(phi) = 1 + sum_k a_k cos(k*phi + ph_k),  k = 2..5,  sum|a_k| <= 0.32
 * x = cx + Rx r cos(phi),  y = cy + Ry r sin(phi)
 * r > 0 everywhere -> star-shaped -> always a simple (non self-intersecting) closed loop.
 */
class CurvePath {
public:
    std::vector<Vec2>   pts;
    std::vector<double> segAng, segLen, segKappa;
    double totalLen = 0;

    void generate(std::mt19937& rng) {
        std::uniform_real_distribution<double> U(0.0, 1.0);
        auto ur = [&](double a, double b) { return a + (b - a) * U(rng); };

        double cx = cfg::WORLD_W / 2 + ur(-6, 6), cy = cfg::WORLD_H / 2 + ur(-2, 2);
        double rx = ur(48, 58), ry = ur(22, 26);
        const int K = 4;
        double amp[K], ph[K], sum = 0;
        for (int k = 0; k < K; ++k) { amp[k] = ur(0.03, 0.15); ph[k] = ur(0, 2 * PI); sum += amp[k]; }
        if (sum > 0.32) for (double& a : amp) a *= 0.32 / sum;

        const int N = 720;
        pts.clear();
        for (int i = 0; i < N; ++i) {
            double phi = 2 * PI * i / N, r = 1.0;
            for (int k = 0; k < K; ++k) r += amp[k] * std::cos((k + 2) * phi + ph[k]);
            pts.push_back({clampd(cx + rx * r * std::cos(phi), 3, cfg::WORLD_W - 3),
                           clampd(cy + ry * r * std::sin(phi), 3, cfg::WORLD_H - 3)});
        }
        if (U(rng) < 0.5) std::reverse(pts.begin(), pts.end());   // random CW / CCW travel
        rebuild();
    }

    void reverse() { std::reverse(pts.begin(), pts.end()); rebuild(); }

    void rebuild() {
        const int N = int(pts.size());
        segAng.assign(N, 0); segLen.assign(N, 0); segKappa.assign(N, 0);
        totalLen = 0;
        for (int i = 0; i < N; ++i) {
            const Vec2 &a = pts[i], &b = pts[(i + 1) % N];
            segAng[i] = std::atan2(b.y - a.y, b.x - a.x);
            segLen[i] = std::max(std::hypot(b.x - a.x, b.y - a.y), 1e-9);
            totalLen += segLen[i];
        }
        // curvature = d(heading)/ds over a small window (smooths discretisation noise)
        const int W = 4;
        for (int i = 0; i < N; ++i) {
            double dAng = wrapAngle(segAng[(i + W) % N] - segAng[(i - W + N) % N]);
            double len = 0;
            for (int j = -W + 1; j <= W; ++j) len += segLen[(i + j + N) % N];
            segKappa[i] = dAng / len;
        }
    }

    PathQuery query(double x, double y) const {
        const int N = int(pts.size());
        double best = 1e18; int bi = 0; Vec2 bp;
        for (int i = 0; i < N; ++i) {
            const Vec2 &a = pts[i], &b = pts[(i + 1) % N];
            double dx = b.x - a.x, dy = b.y - a.y;
            double t = clampd(((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy + 1e-12), 0, 1);
            Vec2 p{a.x + t * dx, a.y + t * dy};
            double d2 = (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y);
            if (d2 < best) { best = d2; bi = i; bp = p; }
        }
        PathQuery q;
        q.closest = bp;
        q.tangent = segAng[bi];
        double tx = std::cos(q.tangent), ty = std::sin(q.tangent);
        q.cte = tx * (y - bp.y) - ty * (x - bp.x);              // + = left of path
        q.curvature = segKappa[bi];
        return q;
    }
};

// ------------------------------------------------------------------ polyline weld
/*
 * Open polyline: straight legs joined by corners of exactly 30, 60 or 90 deg (random sign).
 * Generated by random walk, rejecting legs that leave the window or come within
 * CLEAR units of an earlier leg, so the weld never crosses itself.
 */
static double segSegDist(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    auto cross = [](Vec2 o, Vec2 p, Vec2 q) { return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x); };
    double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
    if (((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0))) return 0.0;      // proper intersection
    auto ptSeg = [](Vec2 p, Vec2 s0, Vec2 s1) {
        double dx = s1.x - s0.x, dy = s1.y - s0.y;
        double t = clampd(((p.x - s0.x) * dx + (p.y - s0.y) * dy) / (dx * dx + dy * dy + 1e-12), 0, 1);
        return std::hypot(p.x - (s0.x + t * dx), p.y - (s0.y + t * dy));
    };
    return std::min({ptSeg(a, c, d), ptSeg(b, c, d), ptSeg(c, a, b), ptSeg(d, a, b)});
}

class PolylinePath {
public:
    std::vector<Vec2>   v;        // vertices
    std::vector<double> ang, len; // per leg
    std::vector<int>    turnDeg;  // signed turn at vertex i+1 (between leg i and i+1)

    int numSeg() const { return int(v.size()) - 1; }

    void generate(std::mt19937& rng) {
        std::uniform_real_distribution<double> U(0.0, 1.0);
        auto ur = [&](double a, double b) { return a + (b - a) * U(rng); };
        const double M = 9, CLEAR = 9;
        const int TURNS[3] = {30, 60, 90};

        for (int attempt = 0; attempt < 400; ++attempt) {
            std::vector<Vec2> pts{{ur(14, 30), ur(14, cfg::WORLD_H - 14)}};
            std::vector<int> turns;
            double h = ur(-0.35, 0.35);
            // first leg
            double L0 = ur(30, 45);
            Vec2 p0{pts[0].x + L0 * std::cos(h), pts[0].y + L0 * std::sin(h)};
            if (p0.x < M || p0.x > cfg::WORLD_W - M || p0.y < M || p0.y > cfg::WORLD_H - M) continue;
            pts.push_back(p0);

            const int wantSeg = 6 + int(U(rng) * 4);      // 6..9 legs
            while (int(pts.size()) - 1 < wantSeg) {
                bool ok = false;
                for (int tries = 0; tries < 60 && !ok; ++tries) {
                    int t = TURNS[int(U(rng) * 3) % 3] * (U(rng) < 0.5 ? 1 : -1);
                    double h2 = h + deg2rad(t), L = ur(20, 42);
                    Vec2 a = pts.back(), b{a.x + L * std::cos(h2), a.y + L * std::sin(h2)};
                    if (b.x < M || b.x > cfg::WORLD_W - M || b.y < M || b.y > cfg::WORLD_H - M) continue;
                    bool clear = true;
                    for (size_t k = 0; k + 2 < pts.size() && clear; ++k)       // skip the adjacent leg
                        if (segSegDist(a, b, pts[k], pts[k + 1]) < CLEAR) clear = false;
                    if (!clear) continue;
                    pts.push_back(b); turns.push_back(t); h = h2; ok = true;
                }
                if (!ok) break;
            }
            if (int(pts.size()) - 1 >= 6) { v = pts; turnDeg = turns; rebuild(); return; }
        }
        // fallback (practically never used): a simple zig-zag
        v = {{20, 20}, {60, 20}, {60, 55}, {100, 55}, {130, 37.7}, {160, 37.7}};
        turnDeg = {90, -90, -30, 30};
        rebuild();
    }

    void reverse() {
        std::reverse(v.begin(), v.end());
        std::reverse(turnDeg.begin(), turnDeg.end());
        for (int& t : turnDeg) t = -t;
        rebuild();
    }

    void rebuild() {
        ang.clear(); len.clear();
        for (int i = 0; i < numSeg(); ++i) {
            ang.push_back(std::atan2(v[i + 1].y - v[i].y, v[i + 1].x - v[i].x));
            len.push_back(dist(v[i], v[i + 1]));
        }
    }

    double along(int i, double x, double y) const {     // distance along leg i from its start
        return (x - v[i].x) * std::cos(ang[i]) + (y - v[i].y) * std::sin(ang[i]);
    }
    double remaining(int i, double x, double y) const { return len[i] - along(i, x, y); }

    // error w.r.t. ONE active leg (treated as an infinite line) - no snapping to the next leg
    PathQuery query(int i, double x, double y) const {
        PathQuery q;
        double s = clampd(along(i, x, y), 0, len[i]);
        q.tangent = ang[i];
        q.closest = {v[i].x + s * std::cos(ang[i]), v[i].y + s * std::sin(ang[i])};
        q.cte = std::cos(ang[i]) * (y - v[i].y) - std::sin(ang[i]) * (x - v[i].x);
        q.curvature = 0.0;
        return q;
    }

    int nearestSeg(double x, double y) const {
        int best = 0; double bd = 1e18;
        for (int i = 0; i < numSeg(); ++i) {
            double d = segSegDist({x, y}, {x, y}, v[i], v[i + 1]);
            if (d < bd) { bd = d; best = i; }
        }
        return best;
    }
};

// ------------------------------------------------------------------ robot
class DiffDriveRobot {
public:
    double x = 0, y = 0, theta = 0;
    double v = 0, w = 0, vLeft = 0, vRight = 0;
    std::vector<std::vector<Vec2>> trail;

    void reset(double x0, double y0, double th) {
        x = x0; y = y0; theta = th;
        v = w = vLeft = vRight = 0;
        trail.clear();
        trail.push_back({{x, y}});
    }

    void step(double vCmd, double wCmd, double dt) {
        // cmd_vel -> wheel speeds (inverse kinematics)
        vLeft  = vCmd - wCmd * cfg::WHEEL_BASE / 2.0;
        vRight = vCmd + wCmd * cfg::WHEEL_BASE / 2.0;
        // wheel speeds -> body twist (forward kinematics)
        v = (vRight + vLeft) / 2.0;
        w = (vRight - vLeft) / cfg::WHEEL_BASE;
        // exact arc integration
        if (std::fabs(w) > 1e-6) {
            double th2 = theta + w * dt;
            x += v / w * (std::sin(th2) - std::sin(theta));
            y -= v / w * (std::cos(th2) - std::cos(theta));
            theta = wrapAngle(th2);
        } else {
            x += v * std::cos(theta) * dt;
            y += v * std::sin(theta) * dt;
        }
        auto& seg = trail.back();
        if (std::hypot(seg.back().x - x, seg.back().y - y) > 0.5) {
            seg.push_back({x, y});
            size_t total = 0;
            for (auto& s : trail) total += s.size();
            if (total > 3000) {
                trail.front().erase(trail.front().begin());
                if (trail.front().empty()) trail.erase(trail.begin());
            }
        }
    }
};

// ------------------------------------------------------------------ controller
/*
 * error     = heading_error + atan(Kcte * cte / v)     (cte term bounded to +-90 deg)
 * angular.z = v * kappa  -  PID(error)                 (kappa = path curvature feed-forward)
 * linear.x  = 5.0
 * On a straight line kappa = 0, so it is a pure PID. On curves the feed-forward
 * supplies the turn rate the curve needs and the PID only corrects the error.
 */
class PathFollowController {
public:
    PID pid{2.0, 0.05, 0.3};
    double kCte = 0.8;
    double cte = 0, headErr = 0, err = 0, wFF = 0;
    PathQuery last;

    // returns angular.z for the given forward speed v
    double compute(const DiffDriveRobot& r, const PathQuery& q, double v, double dt) {
        last    = q;
        cte     = q.cte;
        headErr = wrapAngle(r.theta - q.tangent);
        // floor on v inside atan keeps the cte term from becoming a bang-bang switch at crawl speed
        err     = wrapAngle(headErr + std::atan2(kCte * cte, std::max(v, 1.0)));
        wFF     = v * q.curvature;
        return clampd(wFF - pid.update(err, dt), -cfg::MAX_ANG_VEL, cfg::MAX_ANG_VEL);
    }
};

// ------------------------------------------------------------------ app state
enum class Drag { NONE, P1, P2, MOVE, NEW };
enum class PathMode { LINE, CURVE, POLYLINE };
enum class WeldState { TRACK, PIVOT, DONE };

struct WeldSequencer {
    WeldState state = WeldState::TRACK;
    int seg = 0;                         // active leg
    double vPrev = 0;                    // for accel limiting
    double pivotTarget = 0, pivotErr = 0, settle = 0;
    double remaining = 0, scanned = 0;
    int cornersDone = 0;
    PID pivotPid{3.0, 0.0, 0.25};        // heading PID used only while pivoting in place
};

struct App {
    int winW = cfg::WIN_W, winH = cfg::WIN_H;
    PathMode mode = PathMode::LINE;
    TargetLine line;
    CurvePath curve;
    PolylinePath poly;
    WeldSequencer weld;
    DiffDriveRobot robot;
    PathFollowController ctrl;
    int selGain = 0;
    bool paused = false;
    Drag drag = Drag::NONE;
    PathMode modeBeforeDrag = PathMode::LINE;
    Vec2 dragOff, backupP1, backupP2;
    double cmdV = 0, cmdW = 0, printTimer = 0;
    bool keyLeft = false, keyRight = false, keyUp = false, keyDown = false;
    int btnHover = -1;
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
    std::mt19937 rng{std::random_device{}()};
} app;

static const char* GAIN_NAMES[4] = {"Kp", "Ki", "Kd", "Kcte"};
static double* gainPtr(int i) {
    switch (i) {
        case 0: return &app.ctrl.pid.kp;
        case 1: return &app.ctrl.pid.ki;
        case 2: return &app.ctrl.pid.kd;
        default: return &app.ctrl.kCte;
    }
}

static PathQuery queryPath(double x, double y) {
    switch (app.mode) {
        case PathMode::CURVE:    return app.curve.query(x, y);
        case PathMode::POLYLINE: return app.poly.query(app.weld.seg, x, y);
        default:                 return app.line.query(x, y);
    }
}

static void resetRobot(double x = 40.0, double y = 58.0, double th = 0.0) {
    app.robot.reset(x, y, th);
    app.ctrl.pid.reset();
}
static void pathChanged() { app.ctrl.pid.reset(); }

static void makeRandomCurve() {
    app.curve.generate(app.rng);
    app.mode = PathMode::CURVE;
    app.drag = Drag::NONE;
    pathChanged();                       // robot stays where it is and steers onto the new loop
    std::printf("[path] new random closed curve, length %.1f\n", app.curve.totalLen);
}
// start a weld scan: robot a little off the weld and mis-aligned, so the PID has work to do
static void resetWeldRobot() {
    const auto& P = app.poly;
    double a = P.ang[0];
    app.robot.reset(P.v[0].x - 4 * std::cos(a) - 3 * std::sin(a),
                    P.v[0].y - 4 * std::sin(a) + 3 * std::cos(a), a - deg2rad(15));
    app.weld = WeldSequencer{};
    app.ctrl.pid.reset();
}

static void makePolylineWeld() {
    app.poly.generate(app.rng);
    app.mode = PathMode::POLYLINE;
    app.drag = Drag::NONE;
    resetWeldRobot();
    std::printf("[path] polyline weld, %d legs, corners:", app.poly.numSeg());
    for (int t : app.poly.turnDeg) std::printf(" %+d", t);
    std::printf(" deg\n");
}

static void makeStraightLine() {
    app.mode = PathMode::LINE;
    pathChanged();
    std::printf("[path] straight line\n");
}

static Vec2 toWorld(int mx, int my) {
    return {mx / double(app.winW) * cfg::WORLD_W, (app.winH - my) / double(app.winH) * cfg::WORLD_H};
}

// robot left the window -> bring it back onto the path
static void wrapRobot() {
    const double m = 6;
    auto& r = app.robot;
    if (r.x >= -m && r.x <= cfg::WORLD_W + m && r.y >= -m && r.y <= cfg::WORLD_H + m) return;

    if (app.mode == PathMode::POLYLINE) { resetWeldRobot(); return; }
    if (app.mode == PathMode::CURVE) {          // closed loop: just drop it on the loop start
        r.x = app.curve.pts[0].x; r.y = app.curve.pts[0].y; r.theta = app.curve.segAng[0];
        r.trail.push_back({{r.x, r.y}});
        app.ctrl.pid.reset();
        return;
    }
    double tmin, tmax;                          // line: re-enter at start, keep offset & heading
    if (!app.line.clipRange(tmin, tmax)) return;
    double e = clampd(app.line.signedDistance(r.x, r.y), -20, 20);
    Vec2 p = app.line.pointAt(tmin + 3), n = app.line.normal();
    r.x = p.x + n.x * e;
    r.y = p.y + n.y * e;
    r.trail.push_back({{r.x, r.y}});
}

// ------------------------------------------------------------------ button (pixel coords, top-left origin)
// button 0 = Random Curve (right-most), button 1 = Polyline Weld (left of it)
constexpr int NUM_BUTTONS = 2;
static void buttonRect(int idx, int& x0, int& y0, int& x1, int& y1) {
    x1 = app.winW - 18; x0 = x1 - 210;
    if (idx == 1) { x1 = x0 - 12; x0 = x1 - 210; }
    y0 = 14; y1 = y0 + 40;
}
static int buttonAt(int mx, int my) {
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        int x0, y0, x1, y1; buttonRect(i, x0, y0, x1, y1);
        if (mx >= x0 && mx <= x1 && my >= y0 && my <= y1) return i;
    }
    return -1;
}

// ------------------------------------------------------------------ drawing helpers
static std::vector<Vec2> roundedRectPts(double hl, double hw, double rad, int seg = 8) {
    std::vector<Vec2> pts;
    const double c[4][3] = {{hl - rad, hw - rad, 0}, {-hl + rad, hw - rad, 90},
                            {-hl + rad, -hw + rad, 180}, {hl - rad, -hw + rad, 270}};
    for (auto& k : c)
        for (int i = 0; i <= seg; ++i) {
            double a = deg2rad(k[2] + 90.0 * i / seg);
            pts.push_back({k[0] + rad * std::cos(a), k[1] + rad * std::sin(a)});
        }
    return pts;
}
static const std::vector<Vec2> BODY_PTS = roundedRectPts(cfg::BODY_HL, cfg::BODY_HW, 1.2);

static void drawEllipse(double cx, double cy, double rx, double ry, GLenum mode = GL_LINE_LOOP, int n = 32) {
    glBegin(mode);
    for (int i = 0; i < n; ++i) {
        double a = 2 * PI * i / n;
        glVertex2d(cx + rx * std::cos(a), cy + ry * std::sin(a));
    }
    glEnd();
}

static void drawText(int px, int pyTop, const std::string& s, float r, float g, float b,
                     void* font = GLUT_BITMAP_9_BY_15) {
    glColor3f(r, g, b);   // must be set before glRasterPos
    int h = (font == GLUT_BITMAP_HELVETICA_18) ? 18 : 15;
    glRasterPos2d(px / double(app.winW) * cfg::WORLD_W,
                  (app.winH - pyTop - h) / double(app.winH) * cfg::WORLD_H);
    glutBitmapString(font, reinterpret_cast<const unsigned char*>(s.c_str()));
}

static void drawGrid() {
    glColor3f(0.12f, 0.12f, 0.13f);
    glLineWidth(1);
    glBegin(GL_LINES);
    for (double x = 0; x <= cfg::WORLD_W; x += 10) { glVertex2d(x, 0); glVertex2d(x, cfg::WORLD_H); }
    for (double y = 0; y <= cfg::WORLD_H; y += 10) { glVertex2d(0, y); glVertex2d(cfg::WORLD_W, y); }
    glEnd();
}

static void chevron(const Vec2& p, double ang) {
    double dx = std::cos(ang), dy = std::sin(ang), nx = -dy, ny = dx;
    glVertex2d(p.x, p.y); glVertex2d(p.x - dx * 1.5 + nx * 1.2, p.y - dy * 1.5 + ny * 1.2);
    glVertex2d(p.x, p.y); glVertex2d(p.x - dx * 1.5 - nx * 1.2, p.y - dy * 1.5 - ny * 1.2);
}

static void drawTargetLine() {
    const auto& L = app.line;
    Vec2 d = L.dir();
    glColor3fv(YELLOW);
    glLineWidth(2.5f);
    glBegin(GL_LINES);
    glVertex2d(L.p1.x - d.x * 1000, L.p1.y - d.y * 1000);
    glVertex2d(L.p1.x + d.x * 1000, L.p1.y + d.y * 1000);
    glEnd();

    double tmin, tmax;
    if (L.clipRange(tmin, tmax)) {
        glLineWidth(1.5f);
        glColor3f(YELLOW[0] * .6f, YELLOW[1] * .6f, YELLOW[2] * .6f);
        glBegin(GL_LINES);
        for (double s = tmin + 6; s < tmax; s += 15) chevron(L.pointAt(s), L.angle());
        glEnd();
    }
    glColor3fv(YELLOW);
    glLineWidth(2);
    drawEllipse(L.p1.x, L.p1.y, 1.2, 1.2);
    drawEllipse(L.p2.x, L.p2.y, 1.2, 1.2, GL_POLYGON);
}

static void drawCurve() {
    const auto& C = app.curve;
    glColor3fv(YELLOW);
    glLineWidth(2.5f);
    glBegin(GL_LINE_LOOP);
    for (const auto& p : C.pts) glVertex2d(p.x, p.y);
    glEnd();

    glLineWidth(1.5f);
    glColor3f(YELLOW[0] * .6f, YELLOW[1] * .6f, YELLOW[2] * .6f);
    glBegin(GL_LINES);
    double acc = 0;
    for (size_t i = 0; i < C.pts.size(); ++i) {
        acc += C.segLen[i];
        if (acc >= 15) { acc = 0; chevron(C.pts[i], C.segAng[i]); }
    }
    glEnd();
}

static void drawTrail() {
    const auto& r = app.robot;
    glColor4f(BLUE[0], BLUE[1], BLUE[2], 0.35f);
    glLineWidth(1.5f);
    for (size_t k = 0; k < r.trail.size(); ++k) {
        glBegin(GL_LINE_STRIP);
        for (const auto& p : r.trail[k]) glVertex2d(p.x, p.y);
        if (k + 1 == r.trail.size()) glVertex2d(r.x, r.y);
        glEnd();
    }
}

static void drawError() {
    const auto& r = app.robot;
    Vec2 p = queryPath(r.x, r.y).closest;
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0x00FF);
    glColor3fv(RED);
    glLineWidth(1.5f);
    glBegin(GL_LINES);
    glVertex2d(r.x, r.y); 
    glVertex2d(p.x, p.y);
    glEnd();
    glDisable(GL_LINE_STIPPLE);
    drawEllipse(p.x, p.y, 0.6, 0.6, GL_POLYGON);
}

static void drawRobot() {
    const auto& r = app.robot;
    glPushMatrix();
    glTranslated(r.x, r.y, 0);
    glRotated(rad2deg(r.theta), 0, 0, 1);

    // glColor3f(0.09f, 0.13f, 0.19f);
    // glBegin(GL_POLYGON);
    // for (const auto& p : BODY_PTS) glVertex2d(p.x, p.y);
    // glEnd();
    glColor3fv(BLUE);
    // glLineWidth(2);
    // glBegin(GL_LINE_LOOP);
    // for (const auto& p : BODY_PTS) glVertex2d(p.x, p.y);
    // glEnd();

    // double wy = cfg::WHEEL_BASE / 2 + 1.5;          // wheels: left = +y, right = -y
    // drawEllipse(0, wy, 2.8, 0.9);
    // drawEllipse(0, -wy, 2.8, 0.9);

    double tip = cfg::BODY_HL + 30;                 // centre / heading line
    glBegin(GL_LINES);
    glVertex2d(-cfg::BODY_HL, 0); glVertex2d(tip, 0);
    glVertex2d(tip, 0); glVertex2d(tip - 2.2, 1.2);
    glVertex2d(tip, 0); glVertex2d(tip - 2.2, -1.2);
    glEnd();
    drawEllipse(0, 0, 0.5, 0.5, GL_POLYGON);
    glPopMatrix();
}

static void drawButtons() {
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, app.winW, app.winH, 0);           // pixel coords, y down
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

    for (int i = 0; i < NUM_BUTTONS; ++i) {
        int x0, y0, x1, y1; buttonRect(i, x0, y0, x1, y1);
        bool active = (i == 0 && app.mode == PathMode::CURVE) || (i == 1 && app.mode == PathMode::POLYLINE);
        if (app.btnHover == i) glColor3f(0.30f, 0.22f, 0.06f);
        else if (active)      glColor3f(0.22f, 0.16f, 0.05f);
        else                  glColor3f(0.13f, 0.11f, 0.06f);
        glBegin(GL_QUADS);
        glVertex2i(x0, y0); glVertex2i(x1, y0); glVertex2i(x1, y1); glVertex2i(x0, y1);
        glEnd();
        glColor3fv(YELLOW);
        glLineWidth(active ? 2.5f : 1.5f);
        glBegin(GL_LINE_LOOP);
        glVertex2i(x0, y0); glVertex2i(x1, y0); glVertex2i(x1, y1); glVertex2i(x0, y1);
        glEnd();

        const char* label = (i == 0) ? (app.mode == PathMode::CURVE ? "New Random Curve (C)" : "Random Curve (C)")
                                     : (app.mode == PathMode::POLYLINE ? "New Polyline Weld (P)" : "Polyline Weld (P)");
        int tw = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>(label));
        glRasterPos2i((x0 + x1 - tw) / 2, y0 + 26);
        glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>(label));
    }

    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

static void drawTextWorld(double wx, double wy, const std::string& s, float r, float g, float b) {
    glColor3f(r, g, b);
    glRasterPos2d(wx, wy);
    glutBitmapString(GLUT_BITMAP_9_BY_15, reinterpret_cast<const unsigned char*>(s.c_str()));
}

static void drawPolyline() {
    const auto& P = app.poly;
    const auto& W = app.weld;

    // done legs dimmer, active + upcoming legs full yellow
    glLineWidth(2.5f);
    for (int i = 0; i < P.numSeg(); ++i) {
        if (i < W.seg) glColor3f(YELLOW[0] * .45f, YELLOW[1] * .45f, YELLOW[2] * .45f);
        else           glColor3fv(YELLOW);
        glLineWidth(i == W.seg ? 3.5f : 2.5f);
        glBegin(GL_LINES);
        glVertex2d(P.v[i].x, P.v[i].y); glVertex2d(P.v[i + 1].x, P.v[i + 1].y);
        glEnd();
    }
    // chevrons
    glLineWidth(1.5f);
    glColor3f(YELLOW[0] * .6f, YELLOW[1] * .6f, YELLOW[2] * .6f);
    glBegin(GL_LINES);
    for (int i = 0; i < P.numSeg(); ++i)
        for (double s = 7; s < P.len[i] - 4; s += 12)
            chevron({P.v[i].x + s * std::cos(P.ang[i]), P.v[i].y + s * std::sin(P.ang[i])}, P.ang[i]);
    glEnd();
    // vertices + corner angle labels (placed on the outside of the turn)
    glColor3fv(YELLOW);
    glLineWidth(2);
    drawEllipse(P.v.front().x, P.v.front().y, 1.2, 1.2);
    drawEllipse(P.v.back().x, P.v.back().y, 1.2, 1.2, GL_POLYGON);
    for (int i = 1; i < P.numSeg(); ++i) {
        bool next = (i == W.seg + 1 && W.state != WeldState::DONE);
        if (next && W.state == WeldState::PIVOT) glColor3fv(RED); else glColor3fv(YELLOW);
        drawEllipse(P.v[i].x, P.v[i].y, next ? 1.1 : 0.7, next ? 1.1 : 0.7, GL_POLYGON);
        // label on the outside of the corner: opposite to (outgoing dir - incoming dir)
        double ix = std::cos(P.ang[i]) - std::cos(P.ang[i - 1]);
        double iy = std::sin(P.ang[i]) - std::sin(P.ang[i - 1]);
        double n = std::hypot(ix, iy) + 1e-9;
        char buf[16]; std::snprintf(buf, sizeof buf, "%d", std::abs(P.turnDeg[i - 1]));
        drawTextWorld(P.v[i].x - 4.5 * ix / n - 1.6, P.v[i].y - 4.5 * iy / n - 0.8, buf, 0.85f, 0.85f, 0.85f);
    }
}

static void drawHud() {
    char buf[256];
    const auto& c = app.ctrl;
    const auto& r = app.robot;
    int y = 10;

    std::snprintf(buf, sizeof buf, "cmd_vel   linear.x = %6.3f   angular.z = %+7.3f rad/s", app.cmdV, app.cmdW);
    drawText(12, y, buf, 0.47f, 0.78f, 1.0f, GLUT_BITMAP_HELVETICA_18); y += 26;

    const char* pathName = app.mode == PathMode::CURVE ? "closed curve (L = line)"
                         : app.mode == PathMode::POLYLINE ? "polyline weld (L = line)" : "straight line";
    std::snprintf(buf, sizeof buf, "wheels    v_left = %+6.2f   v_right = %+6.2f      path: %s",
                  r.vLeft, r.vRight, pathName);
    drawText(12, y, buf, 0.84f, 0.84f, 0.84f); y += 19;

    std::snprintf(buf, sizeof buf, "cross-track err = %+7.2f    heading err = %+7.2f deg    combined = %+.3f",
                  c.cte, rad2deg(c.headErr), c.err);
    drawText(12, y, buf, 0.94f, 0.55f, 0.55f); y += 19;

    std::snprintf(buf, sizeof buf, "PID terms  P = %+.3f   I = %+.3f   D = %+.3f    feed-fwd v*kappa = %+.3f",
                  c.pid.p, c.pid.i, c.pid.d, c.wFF);
    drawText(12, y, buf, 0.84f, 0.84f, 0.84f); y += 19;

    std::string g = "Gains: ";
    for (int i = 0; i < 4; ++i) {
        std::snprintf(buf, sizeof buf, "%s%s = %.3f   ", i == app.selGain ? "> " : "  ", GAIN_NAMES[i], *gainPtr(i));
        g += buf;
    }
    drawText(12, y, g, 1.0f, 0.75f, 0.31f); y += 19;

    if (app.mode == PathMode::POLYLINE) {
        const auto& W = app.weld;
        const char* st = W.state == WeldState::TRACK ? "TRACK" : W.state == WeldState::PIVOT ? "PIVOT" : "DONE";
        const char* paut = W.state == WeldState::TRACK ? "SCANNING" : W.state == WeldState::PIVOT ? "PAUSED (corner)" : "COMPLETE";
        if (W.state == WeldState::PIVOT)
            std::snprintf(buf, sizeof buf, "weld: %s  leg %d/%d   pivot err = %+6.1f deg   PAUT: %s   scanned %.1f",
                          st, W.seg + 1, app.poly.numSeg(), rad2deg(W.pivotErr), paut, W.scanned);
        else
            std::snprintf(buf, sizeof buf, "weld: %s  leg %d/%d   to corner = %6.2f   corners %d/%d   PAUT: %s   scanned %.1f%s",
                          st, std::min(W.seg + 1, app.poly.numSeg()), app.poly.numSeg(), std::max(W.remaining, 0.0),
                          W.cornersDone, app.poly.numSeg() - 1, paut, W.scanned,
                          W.state == WeldState::DONE ? "   (R = rescan)" : "");
        drawText(12, y, buf, 0.55f, 0.9f, 0.6f);
    }

    drawText(12, app.winH - 26,
             "C: curve   P: polyline weld   L: line   L-drag: edit/draw line   wheel,arrows: move line   T: random line   "
             "F: flip   R-click: place robot   R: reset   TAB +/-: gains   SPACE: pause",
             0.5f, 0.5f, 0.5f);
    if (app.paused) drawText(app.winW - 110, 64, "PAUSED", 1.0f, 0.47f, 0.47f, GLUT_BITMAP_HELVETICA_18);
}

// ------------------------------------------------------------------ simulation
/*
 * Polyline weld sequencer (PID only):
 *   TRACK : PID holds the robot on the active leg. Speed follows v = sqrt(2*a*d) so the
 *           rotation centre stops exactly on the corner vertex (accel-limited out of corners).
 *   PIVOT : v = 0, separate heading PID turns the robot in place onto the next leg.
 *   DONE  : end of weld reached.
 */
static void updateWeld(double dt) {
    auto& W = app.weld;
    const auto& P = app.poly;
    const auto& r = app.robot;
    double v = 0, w = 0;

    switch (W.state) {
        case WeldState::TRACK: {
            W.remaining = P.remaining(W.seg, r.x, r.y);
            double vTarget = std::sqrt(2.0 * cfg::A_DEC * std::max(W.remaining, 0.0));
            vTarget = clampd(vTarget, cfg::V_CREEP, cfg::LINEAR_VEL);
            v = std::min(vTarget, W.vPrev + cfg::A_ACC * dt);
            w = app.ctrl.compute(r, P.query(W.seg, r.x, r.y), v, dt);
            W.scanned += v * dt;

            if (W.remaining <= cfg::STOP_TOL) {
                if (W.seg == P.numSeg() - 1) {
                    W.state = WeldState::DONE; v = w = 0;
                    std::printf("[weld] scan complete, %.1f units scanned\n", W.scanned);
                } else {
                    W.state = WeldState::PIVOT;
                    W.pivotTarget = P.ang[W.seg + 1];
                    W.pivotPid.reset();
                    W.settle = 0; v = w = 0;
                    std::printf("[weld] corner %d reached (%+d deg) -> pivot, PAUT paused\n",
                                W.seg + 1, P.turnDeg[W.seg]);
                }
            }
            break;
        }
        case WeldState::PIVOT: {
            W.pivotErr = wrapAngle(r.theta - W.pivotTarget);
            w = clampd(-W.pivotPid.update(W.pivotErr, dt), -cfg::MAX_PIVOT_W, cfg::MAX_PIVOT_W);
            v = 0;
            W.settle = (std::fabs(W.pivotErr) < deg2rad(cfg::PIVOT_TOL_DEG)) ? W.settle + dt : 0.0;
            if (W.settle >= cfg::PIVOT_SETTLE) {
                W.state = WeldState::TRACK;
                W.seg++; W.cornersDone++;
                app.ctrl.pid.reset();
                w = 0;
                std::printf("[weld] pivot done -> leg %d, PAUT resumed\n", W.seg + 1);
            }
            break;
        }
        case WeldState::DONE:
            v = w = 0;
            break;
    }
    W.vPrev = v;
    app.cmdV = v;
    app.cmdW = w;
}

static void update(double dt) {
    if (app.mode == PathMode::LINE) {
        if (app.keyLeft)  { app.line.rotate(deg2rad(0.8));  pathChanged(); }
        if (app.keyRight) { app.line.rotate(-deg2rad(0.8)); pathChanged(); }
        if (app.keyUp)    { app.line.shift(0, 0.4);         pathChanged(); }
        if (app.keyDown)  { app.line.shift(0, -0.4);        pathChanged(); }
    }
    if (app.paused) return;

    if (app.mode == PathMode::POLYLINE) {
        updateWeld(dt);
    } else {
        app.cmdV = cfg::LINEAR_VEL;
        app.cmdW = app.ctrl.compute(app.robot, queryPath(app.robot.x, app.robot.y), app.cmdV, dt);
    }
    const int sub = 4;
    for (int i = 0; i < sub; ++i) app.robot.step(app.cmdV, app.cmdW, dt / sub);
    wrapRobot();

    app.printTimer += dt;
    if (app.printTimer >= 0.1) {
        app.printTimer = 0;
        std::printf("/cmd_vel  linear: {x: %.3f, y: 0.0, z: 0.0}  angular: {x: 0.0, y: 0.0, z: %+.3f}"
                    "   cte: %+.2f  heading_err: %+.1f deg\n",
                    app.cmdV, app.cmdW, app.ctrl.cte, rad2deg(app.ctrl.headErr));
        std::fflush(stdout);
    }
}

// ------------------------------------------------------------------ GLUT callbacks
static void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    drawGrid();
    if (app.mode == PathMode::CURVE) drawCurve();
    else if (app.mode == PathMode::POLYLINE) drawPolyline();
    else drawTargetLine();
    drawTrail();
    drawError();
    drawRobot();
    drawHud();
    drawButtons();
    glutSwapBuffers();
}

static void reshape(int w, int h) {
    app.winW = std::max(w, 1);
    app.winH = std::max(h, 1);
    glViewport(0, 0, app.winW, app.winH);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, cfg::WORLD_W, 0, cfg::WORLD_H);
    glMatrixMode(GL_MODELVIEW);
}

static void timer(int) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - app.last).count();
    app.last = now;
    update(std::min(std::max(dt, 1e-4), 0.05));
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

static void keyboard(unsigned char key, int, int) {
    switch (key) {
        case 27: glutLeaveMainLoop(); break;
        case ' ': app.paused = !app.paused; break;
        case 'r': case 'R':
            if (app.mode == PathMode::POLYLINE) resetWeldRobot(); else resetRobot();
            break;
        case 'p': case 'P': makePolylineWeld(); break;
        case 'c': case 'C': makeRandomCurve(); break;
        case 'l': case 'L': makeStraightLine(); break;
        case 'f': case 'F':
            if (app.mode == PathMode::CURVE) app.curve.reverse();
            else if (app.mode == PathMode::POLYLINE) { app.poly.reverse(); resetWeldRobot(); }
            else std::swap(app.line.p1, app.line.p2);
            pathChanged();
            break;
        case 't': case 'T': {
            std::uniform_real_distribution<double> ua(-0.6, 0.6), ux(60, 110), uy(20, 55);
            double a = ua(app.rng), cx = ux(app.rng), cy = uy(app.rng);
            app.line = TargetLine({cx - 70 * std::cos(a), cy - 70 * std::sin(a)},
                                  {cx + 70 * std::cos(a), cy + 70 * std::sin(a)});
            app.line.clampToWorld();
            app.mode = PathMode::LINE;
            pathChanged();
            break;
        }
        case '\t': app.selGain = (app.selGain + 1) % 4; break;
        case '+': case '=': { double* g = gainPtr(app.selGain); *g = std::max(*g, 0.01) * 1.1; break; }
        case '-': case '_': { double* g = gainPtr(app.selGain); *g /= 1.1; if (*g < 0.01) *g = 0.0; break; }
        default: break;
    }
}

static void special(int key, int, int) {
    if (key == GLUT_KEY_LEFT) app.keyLeft = true;
    if (key == GLUT_KEY_RIGHT) app.keyRight = true;
    if (key == GLUT_KEY_UP) app.keyUp = true;
    if (key == GLUT_KEY_DOWN) app.keyDown = true;
}
static void specialUp(int key, int, int) {
    if (key == GLUT_KEY_LEFT) app.keyLeft = false;
    if (key == GLUT_KEY_RIGHT) app.keyRight = false;
    if (key == GLUT_KEY_UP) app.keyUp = false;
    if (key == GLUT_KEY_DOWN) app.keyDown = false;
}

static void mouse(int button, int state, int mx, int my) {
    Vec2 w = toWorld(mx, my);
    auto& L = app.line;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        int b = buttonAt(mx, my);
        if (b == 0) { makeRandomCurve(); return; }
        if (b == 1) { makePolylineWeld(); return; }
    }
    if (button == 3 || button == 4) {               // wheel up / down (freeglut on X11)
        if (state == GLUT_DOWN && app.mode == PathMode::LINE) {
            L.rotate(deg2rad(3.0) * (button == 3 ? 1 : -1));
            pathChanged();
        }
        return;
    }
    if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
        resetRobot(w.x, w.y, app.robot.theta);
        if (app.mode == PathMode::POLYLINE) {           // continue the weld from the nearest leg
            app.weld = WeldSequencer{};
            app.weld.seg = app.poly.nearestSeg(w.x, w.y);
        }
        return;
    }
    if (button != GLUT_LEFT_BUTTON) return;

    if (state == GLUT_DOWN) {
        bool lineMode = app.mode == PathMode::LINE;
        if (lineMode && dist(w, L.p1) < 3)                           app.drag = Drag::P1;
        else if (lineMode && dist(w, L.p2) < 3)                      app.drag = Drag::P2;
        else if (lineMode && std::fabs(L.signedDistance(w.x, w.y)) < 2) { app.drag = Drag::MOVE; app.dragOff = w; }
        else {                                       // draw a brand-new straight line
            app.drag = Drag::NEW;
            app.modeBeforeDrag = app.mode;
            app.backupP1 = L.p1; app.backupP2 = L.p2;
            L.p1 = w; L.p2 = {w.x + 0.01, w.y};
        }
    } else if (app.drag != Drag::NONE) {
        if (app.drag == Drag::NEW) {
            if (dist(L.p1, L.p2) < 3) {              // just a click: cancel
                L.p1 = app.backupP1; L.p2 = app.backupP2;
                app.mode = app.modeBeforeDrag;
            }
        }
        if (dist(L.p1, L.p2) < 1) L.p2 = {L.p1.x + 10, L.p1.y};
        app.drag = Drag::NONE;
        pathChanged();
    }
}

static void motion(int mx, int my) {
    app.btnHover = buttonAt(mx, my);
    if (app.drag == Drag::NONE) return;
    Vec2 w = toWorld(mx, my);
    w.x = clampd(w.x, 2, cfg::WORLD_W - 2);
    w.y = clampd(w.y, 2, cfg::WORLD_H - 2);
    auto& L = app.line;
    switch (app.drag) {
        case Drag::P1: L.p1 = w; break;
        case Drag::P2: L.p2 = w; break;
        case Drag::NEW:
            L.p2 = w;
            if (dist(L.p1, L.p2) >= 3) app.mode = PathMode::LINE;   // switch only once really dragging
            break;
        case Drag::MOVE: L.shift(w.x - app.dragOff.x, w.y - app.dragOff.y); app.dragOff = w; break;
        default: break;
    }
    pathChanged();
}

static void passiveMotion(int mx, int my) { app.btnHover = buttonAt(mx, my); }

// ------------------------------------------------------------------ main
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
    glutInitWindowSize(cfg::WIN_W, cfg::WIN_H);
    glutCreateWindow("Diff-Drive PID Path Follower (OpenGL)");
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutIgnoreKeyRepeat(1);

    glClearColor(0.07f, 0.07f, 0.07f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
#ifdef GL_MULTISAMPLE
    glEnable(GL_MULTISAMPLE);
#endif

    resetRobot();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutSpecialUpFunc(specialUp);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutPassiveMotionFunc(passiveMotion);
    glutTimerFunc(16, timer, 0);

    std::printf("Diff-drive PID path follower started. linear.x fixed at %.1f\n", cfg::LINEAR_VEL);
    glutMainLoop();
    return 0;
}