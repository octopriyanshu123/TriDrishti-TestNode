// Diff-drive robot heading PID with a minimal OpenGL (GLFW) visualization.
//
// Build (Linux):   g++ -std=c++17 -O2 diff_drive_pid_viz.cpp -o robot_viz -lglfw -lGL -lpthread
// Build (macOS):   g++ -std=c++17 -O2 diff_drive_pid_viz.cpp -o robot_viz -lglfw -framework OpenGL
// Build (Windows, MSYS2/MinGW): g++ -std=c++17 -O2 diff_drive_pid_viz.cpp -o robot_viz -lglfw3 -lopengl32
//
// Type a new target heading (degrees) in the terminal and press Enter. "q" quits.

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cctype>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }
inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// ============================================================
// PID controller
// ============================================================
struct PIDConfig
{
    double kp = 0.0, ki = 0.0, kd = 0.0;
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max =  std::numeric_limits<double>::infinity();
    double i_min   = -std::numeric_limits<double>::infinity();
    double i_max   =  std::numeric_limits<double>::infinity();
    double d_filter_tau = 0.0;
    bool   angular = false;
};

struct PIDTerms { double p = 0, i = 0, d = 0, out = 0; };

class PID
{
public:
    explicit PID(const PIDConfig& cfg) : cfg_(cfg) {}

    double compute(double setpoint, double measurement, double dt)
    {
        if (dt <= 0.0) throw std::invalid_argument("PID: dt must be > 0");

        double error = setpoint - measurement;
        if (cfg_.angular) error = wrapAngle(error);

        const double p = cfg_.kp * error;

        double d = 0.0;
        if (initialized_)
        {
            double dm = measurement - prev_meas_;
            if (cfg_.angular) dm = wrapAngle(dm);
            const double raw = -dm / dt;
            const double alpha = cfg_.d_filter_tau > 0.0 ? dt / (cfg_.d_filter_tau + dt) : 1.0;
            d_filt_ += alpha * (raw - d_filt_);
            d = cfg_.kd * d_filt_;
        }
        prev_meas_ = measurement;
        initialized_ = true;

        const double cand = std::clamp(i_ + cfg_.ki * error * dt, cfg_.i_min, cfg_.i_max);
        const double unsat = p + cand + d;
        if (!((unsat > cfg_.out_max && error > 0) || (unsat < cfg_.out_min && error < 0)))
            i_ = cand;

        const double out = std::clamp(p + i_ + d, cfg_.out_min, cfg_.out_max);
        last_ = {p, i_, d, out};
        return out;
    }

    const PIDTerms& terms() const { return last_; }

private:
    PIDConfig cfg_;
    double i_ = 0.0, d_filt_ = 0.0, prev_meas_ = 0.0;
    bool initialized_ = false;
    PIDTerms last_;
};

// ============================================================
// Differential-drive robot
// ============================================================
struct RobotParams
{
    double wheel_radius     = 0.05;  // [m]
    double wheel_separation = 0.30;  // [m]
    double max_wheel_speed  = 10.0;  // [rad/s]
    double motor_tau        = 0.05;  // [s]
};

struct RobotState { double x = 0, y = 0, theta = 0, wl = 0, wr = 0; };

class DiffDriveRobot
{
public:
    explicit DiffDriveRobot(const RobotParams& p) : p_(p) {}

    void step(double wl_cmd, double wr_cmd, double dt)
    {
        wl_cmd = std::clamp(wl_cmd, -p_.max_wheel_speed, p_.max_wheel_speed);
        wr_cmd = std::clamp(wr_cmd, -p_.max_wheel_speed, p_.max_wheel_speed);

        const double a = dt / (p_.motor_tau + dt);
        s_.wl += a * (wl_cmd - s_.wl);
        s_.wr += a * (wr_cmd - s_.wr);

        const double v = p_.wheel_radius * (s_.wr + s_.wl) / 2.0;
        const double w = p_.wheel_radius * (s_.wr - s_.wl) / p_.wheel_separation;

        s_.x += v * std::cos(s_.theta) * dt;
        s_.y += v * std::sin(s_.theta) * dt;
        s_.theta = wrapAngle(s_.theta + w * dt);
    }

    void inverse(double v, double w, double& wl, double& wr) const
    {
        wl = (v - w * p_.wheel_separation / 2.0) / p_.wheel_radius;
        wr = (v + w * p_.wheel_separation / 2.0) / p_.wheel_radius;
    }

    double maxYawRate() const { return 2.0 * p_.wheel_radius * p_.max_wheel_speed / p_.wheel_separation; }
    const RobotState&  state()  const { return s_; }
    const RobotParams& params() const { return p_; }

private:
    RobotParams p_;
    RobotState s_;
};

// ============================================================
// Terminal input (runs in its own thread)
// ============================================================
struct InputBox
{
    std::mutex m;
    bool   has_new = false;
    double value   = 0.0;
    std::atomic<bool> quit{false};
};

void inputThread(InputBox* box)
{
    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line == "q" || line == "Q") { box->quit = true; return; }
        if (line.empty()) continue;
        try
        {
            const double d = std::stod(line);
            std::lock_guard<std::mutex> lk(box->m);
            box->value = d;
            box->has_new = true;
        }
        catch (const std::exception&)
        {
            std::cout << "Invalid number: " << line << "\n> " << std::flush;
        }
    }
    box->quit = true;   // stdin closed
}

// ============================================================
// Drawing helpers (legacy OpenGL, kept minimal)
// ============================================================
struct Sample { double t, target, heading, p, i, d, out; };
struct Col { float r, g, b; };

const Col C_TARGET  {0.95f, 0.25f, 0.25f};
const Col C_HEADING {0.25f, 0.95f, 0.35f};
const Col C_P       {0.35f, 0.55f, 1.00f};
const Col C_I       {1.00f, 0.85f, 0.20f};
const Col C_D       {0.95f, 0.35f, 0.95f};
const Col C_OUT     {1.00f, 1.00f, 1.00f};
const Col C_TEXT    {0.80f, 0.80f, 0.80f};
const Col C_DIM     {0.55f, 0.55f, 0.55f};

inline void color(const Col& c) { glColor3f(c.r, c.g, c.b); }

template <typename... A>
std::string fmt(const char* f, A... a)
{
    char buf[160];
    std::snprintf(buf, sizeof(buf), f, a...);
    return buf;
}

// ---------------- Tiny built-in 5x7 bitmap font ----------------
struct Glyph { char c; const char* rows[7]; };

static const Glyph FONT[] = {
    {' ', {".....",".....",".....",".....",".....",".....","....."}},
    {'0', {".###.","#...#","#..##","#.#.#","##..#","#...#",".###."}},
    {'1', {"..#..",".##..","..#..","..#..","..#..","..#..",".###."}},
    {'2', {".###.","#...#","....#","...#.","..#..",".#...","#####"}},
    {'3', {"#####","...#.","..#..","...#.","....#","#...#",".###."}},
    {'4', {"...#.","..##.",".#.#.","#..#.","#####","...#.","...#."}},
    {'5', {"#####","#....","####.","....#","....#","#...#",".###."}},
    {'6', {"..##.",".#...","#....","####.","#...#","#...#",".###."}},
    {'7', {"#####","....#","...#.","..#..",".#...",".#...",".#..."}},
    {'8', {".###.","#...#","#...#",".###.","#...#","#...#",".###."}},
    {'9', {".###.","#...#","#...#",".####","....#","...#.",".##.."}},
    {'A', {".###.","#...#","#...#","#####","#...#","#...#","#...#"}},
    {'B', {"####.","#...#","#...#","####.","#...#","#...#","####."}},
    {'C', {".###.","#...#","#....","#....","#....","#...#",".###."}},
    {'D', {"###..","#..#.","#...#","#...#","#...#","#..#.","###.."}},
    {'E', {"#####","#....","#....","####.","#....","#....","#####"}},
    {'F', {"#####","#....","#....","####.","#....","#....","#...."}},
    {'G', {".###.","#...#","#....","#.###","#...#","#...#",".####"}},
    {'H', {"#...#","#...#","#...#","#####","#...#","#...#","#...#"}},
    {'I', {".###.","..#..","..#..","..#..","..#..","..#..",".###."}},
    {'J', {"..###","...#.","...#.","...#.","...#.","#..#.",".##.."}},
    {'K', {"#...#","#..#.","#.#..","##...","#.#..","#..#.","#...#"}},
    {'L', {"#....","#....","#....","#....","#....","#....","#####"}},
    {'M', {"#...#","##.##","#.#.#","#.#.#","#...#","#...#","#...#"}},
    {'N', {"#...#","#...#","##..#","#.#.#","#..##","#...#","#...#"}},
    {'O', {".###.","#...#","#...#","#...#","#...#","#...#",".###."}},
    {'P', {"####.","#...#","#...#","####.","#....","#....","#...."}},
    {'Q', {".###.","#...#","#...#","#...#","#.#.#","#..#.",".##.#"}},
    {'R', {"####.","#...#","#...#","####.","#.#..","#..#.","#...#"}},
    {'S', {".####","#....","#....",".###.","....#","....#","####."}},
    {'T', {"#####","..#..","..#..","..#..","..#..","..#..","..#.."}},
    {'U', {"#...#","#...#","#...#","#...#","#...#","#...#",".###."}},
    {'V', {"#...#","#...#","#...#","#...#","#...#",".#.#.","..#.."}},
    {'W', {"#...#","#...#","#...#","#.#.#","#.#.#","#.#.#",".#.#."}},
    {'X', {"#...#","#...#",".#.#.","..#..",".#.#.","#...#","#...#"}},
    {'Y', {"#...#","#...#",".#.#.","..#..","..#..","..#..","..#.."}},
    {'Z', {"#####","....#","...#.","..#..",".#...","#....","#####"}},
    {'.', {".....",".....",".....",".....",".....",".##..",".##.."}},
    {'-', {".....",".....",".....",".###.",".....",".....","....."}},
    {'+', {".....","..#..","..#..","#####","..#..","..#..","....."}},
    {'=', {".....",".....","#####",".....","#####",".....","....."}},
    {':', {".....",".##..",".##..",".....",".##..",".##..","....."}},
    {'/', {"....#","....#","...#.","..#..",".#...","#....","#...."}},
    {'[', {".###.",".#...",".#...",".#...",".#...",".#...",".###."}},
    {']', {".###.","...#.","...#.","...#.","...#.","...#.",".###."}},
    {'(', {"...#.","..#..",".#...",".#...",".#...","..#..","...#."}},
    {')', {".#...","..#..","...#.","...#.","...#.","..#..",".#..."}},
    {'|', {"..#..","..#..","..#..","..#..","..#..","..#..","..#.."}},
};

const Glyph* findGlyph(char c)
{
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    for (const auto& g : FONT) if (g.c == c) return &g;
    return nullptr;
}

inline double textWidth(const std::string& s, double px) { return s.size() * 6.0 * px; }

// Draws text with its bottom-left corner at (x, y) in pixel coordinates.
void drawText(double x, double y, const std::string& s, double px)
{
    glBegin(GL_QUADS);
    for (char ch : s)
    {
        if (const Glyph* g = findGlyph(ch))
            for (int r = 0; r < 7; ++r)
                for (int c = 0; c < 5; ++c)
                    if (g->rows[r][c] == '#')
                    {
                        const double gx = x + c * px, gy = y + (6 - r) * px;
                        glVertex2d(gx, gy);          glVertex2d(gx + px, gy);
                        glVertex2d(gx + px, gy + px); glVertex2d(gx, gy + px);
                    }
        x += 6.0 * px;
    }
    glEnd();
}

// ---------------- Basic shapes ----------------
void setView(int x, int y, int w, int h, double l, double r, double b, double t)
{
    glViewport(x, y, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(l, r, b, t, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void circle(double cx, double cy, double r, bool filled, int n = 48)
{
    glBegin(filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP);
    for (int k = 0; k < n; ++k)
    {
        const double a = 2.0 * PI * k / n;
        glVertex2d(cx + r * std::cos(a), cy + r * std::sin(a));
    }
    glEnd();
}

void rect(double cx, double cy, double w, double h)
{
    glBegin(GL_QUADS);
    glVertex2d(cx - w / 2, cy - h / 2); glVertex2d(cx + w / 2, cy - h / 2);
    glVertex2d(cx + w / 2, cy + h / 2); glVertex2d(cx - w / 2, cy + h / 2);
    glEnd();
}

void line(double x0, double y0, double x1, double y1)
{
    glBegin(GL_LINES);
    glVertex2d(x0, y0);
    glVertex2d(x1, y1);
    glEnd();
}

// ---------------- Robot view (left half) ----------------
struct UI { int fw, fh; double s; };   // framebuffer size + HiDPI scale

void drawRobotView(const UI& ui, int x, int y, int w, int h,
                   const DiffDriveRobot& robot, double target, double target_deg, bool reached)
{
    const double half = 0.60;   // meters visible from center (vertical)
    const double aspect = double(w) / h;
    setView(x, y, w, h, -half * aspect, half * aspect, -half, half);

    const RobotState& s = robot.state();
    const RobotParams& p = robot.params();
    glTranslated(-s.x, -s.y, 0);   // keep robot centered

    // Compass ring
    color({0.3f, 0.3f, 0.3f});
    circle(s.x, s.y, 0.32, false, 72);
    for (int k = 0; k < 4; ++k)
    {
        const double a = k * PI / 2;
        line(s.x + 0.30 * std::cos(a), s.y + 0.30 * std::sin(a),
             s.x + 0.34 * std::cos(a), s.y + 0.34 * std::sin(a));
    }

    // Target direction
    color(C_TARGET);
    line(s.x, s.y, s.x + 0.36 * std::cos(target), s.y + 0.36 * std::sin(target));

    // Robot body
    glPushMatrix();
    glTranslated(s.x, s.y, 0);
    glRotated(rad2deg(s.theta), 0, 0, 1);

    color({0.35f, 0.45f, 0.6f});
    circle(0, 0, p.wheel_separation / 2 * 0.9, true);

    color({0.8f, 0.8f, 0.8f});
    rect(0,  p.wheel_separation / 2, 2 * p.wheel_radius, 0.03);   // left wheel
    rect(0, -p.wheel_separation / 2, 2 * p.wheel_radius, 0.03);   // right wheel

    color(C_HEADING);
    line(0, 0, 0.24, 0);
    glBegin(GL_TRIANGLES);
    glVertex2d(0.28, 0); glVertex2d(0.23, 0.025); glVertex2d(0.23, -0.025);
    glEnd();
    glPopMatrix();

    // ---- Text overlay (pixel coordinates) ----
    setView(0, 0, ui.fw, ui.fh, 0, ui.fw, 0, ui.fh);
    const double big = 2.0 * ui.s, sm = 1.6 * ui.s;
    const double lh = 11 * ui.s + 7 * big;  // line height (big)
    const double lhs = 8 * ui.s + 7 * sm;   // line height (small)

    // Compass labels
    const double k = (h / 2.0) / half;      // meters -> pixels
    const double cx = x + w / 2.0, cy = y + h / 2.0;
    color(C_DIM);
    const char* lbl[4] = {"0", "90", "180", "-90"};
    for (int i = 0; i < 4; ++i)
    {
        const double a = i * PI / 2;
        const double px = cx + 0.39 * k * std::cos(a), py = cy + 0.39 * k * std::sin(a);
        drawText(px - textWidth(lbl[i], sm) / 2, py - 3.5 * sm, lbl[i], sm);
    }

    // Top-left info
    double ty = y + h - 10 * ui.s - 7 * big;
    const double tx = x + 12 * ui.s;
    color(C_TEXT);    drawText(tx, ty, "ROBOT TOP VIEW", big);                         ty -= lh;
    color(C_TARGET);  drawText(tx, ty, fmt("TARGET  %7.1f DEG", target_deg), big);      ty -= lh;
    color(C_HEADING); drawText(tx, ty, fmt("HEADING %7.1f DEG", rad2deg(s.theta)), big); ty -= lh;
    color(C_TEXT);    drawText(tx, ty, fmt("ERROR   %7.1f DEG",
                                           rad2deg(wrapAngle(target - s.theta))), big);

    // Bottom-left info
    double by = y + 10 * ui.s;
    color(reached ? C_I : C_DIM);
    drawText(tx, by, reached ? "REACHED - TYPE NEW TARGET IN TERMINAL" : "MOVING...", sm);
    by += lhs;
    color(C_DIM);
    drawText(tx, by, fmt("WHEEL SPEED  L %6.2f  R %6.2f RAD/S", s.wl, s.wr), sm); by += lhs;
    drawText(tx, by, fmt("WHEEL R = %.3f M   SEPARATION = %.3f M",
                         p.wheel_radius, p.wheel_separation), sm);
}

// ---------------- Graph panels (right half) ----------------
struct Series { const char* name; Col col; double (*get)(const Sample&); };

void drawPanel(const UI& ui, int x, int y, int w, int h, const char* title, const char* fmtv,
               const std::deque<Sample>& hist, double t0, double t1,
               double ymin, double ymax, double ystep, const Series* ser, int n)
{
    const double big = 2.0 * ui.s, sm = 1.5 * ui.s;
    const int ml = int(48 * ui.s), mr = int(18 * ui.s);
    const int mt = int(14 * ui.s + 7 * big + 7 * sm + 10 * ui.s), mb = int(20 * ui.s);
    const int px = x + ml, py = y + mb, pw = w - ml - mr, ph = h - mt - mb;
    if (pw < 20 || ph < 20) return;

    // Data area
    setView(px, py, pw, ph, t0, t1, ymin, ymax);
    color({0.18f, 0.18f, 0.18f});
    for (double g = std::ceil(ymin / ystep) * ystep; g <= ymax + 1e-9; g += ystep) line(t0, g, t1, g);
    for (double tt = std::ceil(t0 / 2.0) * 2.0; tt <= t1 + 1e-9; tt += 2.0) line(tt, ymin, tt, ymax);
    color({0.45f, 0.45f, 0.45f});
    line(t0, 0, t1, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2d(t0, ymin); glVertex2d(t1, ymin); glVertex2d(t1, ymax); glVertex2d(t0, ymax);
    glEnd();

    for (int s = 0; s < n; ++s)
    {
        color(ser[s].col);
        glBegin(GL_LINE_STRIP);
        for (const auto& smp : hist) glVertex2d(smp.t, ser[s].get(smp));
        glEnd();
    }

    // Text overlay
    setView(0, 0, ui.fw, ui.fh, 0, ui.fw, 0, ui.fh);

    // Title
    const double title_y = y + h - 6 * ui.s - 7 * big;
    color(C_TEXT);
    drawText(px, title_y, title, big);

    // Legend with current values
    const double ly = title_y - 8 * ui.s - 7 * sm;
    double lx = px;
    for (int s = 0; s < n; ++s)
    {
        const double v = hist.empty() ? 0.0 : ser[s].get(hist.back());
        const std::string label = std::string(ser[s].name) + " " + fmt(fmtv, v);
        color(ser[s].col);
        glBegin(GL_QUADS);   // color swatch
        const double sw = 14 * ui.s, sy = ly + 3.5 * sm;
        glVertex2d(lx, sy - ui.s * 1.5); glVertex2d(lx + sw, sy - ui.s * 1.5);
        glVertex2d(lx + sw, sy + ui.s * 1.5); glVertex2d(lx, sy + ui.s * 1.5);
        glEnd();
        drawText(lx + sw + 5 * ui.s, ly, label, sm);
        lx += sw + 5 * ui.s + textWidth(label, sm) + 14 * ui.s;
    }

    // Y tick labels
    color(C_DIM);
    for (double g = std::ceil(ymin / ystep) * ystep; g <= ymax + 1e-9; g += ystep)
    {
        const std::string l = fmt("%.0f", g);
        const double yy = py + (g - ymin) / (ymax - ymin) * ph;
        drawText(px - 5 * ui.s - textWidth(l, sm), yy - 3.5 * sm, l, sm);
    }

    // X tick labels (time)
    for (double tt = std::ceil(t0 / 2.0) * 2.0; tt <= t1 + 1e-9; tt += 2.0)
    {
        const std::string l = fmt("%.0f", tt);
        const double xx = px + (tt - t0) / (t1 - t0) * pw;
        drawText(xx - textWidth(l, sm) / 2, y + 5 * ui.s, l, sm);
    }
}

void drawGraphs(const UI& ui, int x, int y, int w, int h,
                const std::deque<Sample>& hist, double window_s, double yaw_max)
{
    const double t1 = std::max(hist.empty() ? 0.0 : hist.back().t, window_s);
    const double t0 = t1 - window_s;

    static const Series heading_series[] = {
        {"TARGET",  C_TARGET,  +[](const Sample& s) { return s.target; }},
        {"HEADING", C_HEADING, +[](const Sample& s) { return s.heading; }},
    };
    static const Series pid_series[] = {
        {"P",   C_P,   +[](const Sample& s) { return s.p; }},
        {"I",   C_I,   +[](const Sample& s) { return s.i; }},
        {"D",   C_D,   +[](const Sample& s) { return s.d; }},
        {"OUT", C_OUT, +[](const Sample& s) { return s.out; }},
    };

    const double r = std::ceil(yaw_max * 2.0);
    drawPanel(ui, x, y + h / 2, w, h / 2, "HEADING [DEG]   VS TIME [S]", "%.1f",
              hist, t0, t1, -200, 200, 90, heading_series, 2);
    drawPanel(ui, x, y, w, h / 2, "PID TERMS [RAD/S]   OUT = YAW RATE CMD", "%+.2f",
              hist, t0, t1, -r, r, 2, pid_series, 4);
}

// ============================================================
// Main
// ============================================================
int main()
{
    // ---- Robot ----
    RobotParams rp;
    rp.wheel_radius     = 0.05;
    rp.wheel_separation = 0.30;
    DiffDriveRobot robot(rp);

    // ---- PID ----
    PIDConfig cfg;
    cfg.kp = 4.0;
    cfg.ki = 0.0;     // heading already integrates yaw rate; raise if you add disturbances
    cfg.kd = 0.3;
    cfg.out_min = -robot.maxYawRate();
    cfg.out_max =  robot.maxYawRate();
    cfg.i_min = -0.3;
    cfg.i_max =  0.3;
    cfg.d_filter_tau = 0.02;
    cfg.angular = true;
    PID pid(cfg);

    // ---- Sim settings ----
    const double dt          = 0.01;   // 100 Hz physics
    const double v           = 0.0;    // turn in place
    const double tol_deg     = 0.5;
    const double rate_tol    = 0.02;
    const double settle_time = 0.3;
    const double window_s    = 10.0;   // graph time window

    double target_deg = 90.0;
    double target     = deg2rad(target_deg);

    // ---- Window ----
    if (!glfwInit()) { std::cerr << "glfwInit failed\n"; return 1; }
    GLFWwindow* win = glfwCreateWindow(1200, 600, "Diff-Drive Heading PID", nullptr, nullptr);
    if (!win) { std::cerr << "Window creation failed\n"; glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glfwSetKeyCallback(win, [](GLFWwindow* w, int key, int, int action, int) {
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) glfwSetWindowShouldClose(w, 1);
    });

    std::cout << "Type a target heading in degrees + Enter (q or ESC to quit).\n"
              << "Target: " << target_deg << " deg\n" << std::flush;

    InputBox box;
    std::thread(inputThread, &box).detach();

    std::deque<Sample> hist;
    double t = 0.0, acc = 0.0, settled = 0.0;
    bool announced = false;
    double last = glfwGetTime();

    while (!glfwWindowShouldClose(win) && !box.quit)
    {
        // ---- New target from terminal ----
        {
            std::lock_guard<std::mutex> lk(box.m);
            if (box.has_new)
            {
                target_deg = box.value;
                target = wrapAngle(deg2rad(target_deg));
                box.has_new = false;
                settled = 0.0;
                announced = false;
                std::cout << "Target: " << target_deg << " deg\n" << std::flush;
            }
        }

        // ---- Physics at fixed dt, synced to real time ----
        const double now = glfwGetTime();
        acc = std::min(acc + (now - last), 0.25);
        last = now;

        while (acc >= dt)
        {
            const double heading = robot.state().theta;
            const double w_cmd = pid.compute(target, heading, dt);
            double wl, wr;
            robot.inverse(v, w_cmd, wl, wr);
            robot.step(wl, wr, dt);

            const PIDTerms& tm = pid.terms();
            hist.push_back({t, rad2deg(target), rad2deg(heading), tm.p, tm.i, tm.d, tm.out});
            while (!hist.empty() && hist.front().t < t - window_s) hist.pop_front();

            const double err_deg = rad2deg(wrapAngle(target - heading));
            settled = (std::abs(err_deg) < tol_deg && std::abs(w_cmd) < rate_tol) ? settled + dt : 0.0;
            if (!announced && settled >= settle_time)
            {
                announced = true;
                std::printf("Reached %.2f deg. Enter new target: ", rad2deg(robot.state().theta));
                std::fflush(stdout);
            }

            t += dt;
            acc -= dt;
        }

        // ---- Render ----
        int fw, fh, ww, wh;
        glfwGetFramebufferSize(win, &fw, &fh);
        glfwGetWindowSize(win, &ww, &wh);
        const UI ui{fw, fh, ww > 0 ? double(fw) / ww : 1.0};

        glViewport(0, 0, fw, fh);
        glClearColor(0.08f, 0.08f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLineWidth(2.0f);

        const int split = fw * 9 / 20;
        drawRobotView(ui, 0, 0, split, fh, robot, target, target_deg, announced);
        drawGraphs(ui, split, 0, fw - split, fh, hist, window_s, robot.maxYawRate());

        // Divider
        setView(0, 0, fw, fh, 0, fw, 0, fh);
        color({0.25f, 0.25f, 0.25f});
        line(split, 0, split, fh);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    std::cout << "\nBye.\n";
    return 0;   // input thread is detached; process exit cleans it up
}