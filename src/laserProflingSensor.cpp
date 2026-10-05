// Live OpenGL (GLUT) viewer for crawler_i2w_msgs::I2wScanControlProfile
//
// Threads:
//   - i2w thread : creates the node, Setup(), Tick() loop, subscription callback
//   - main thread: GLUT window / rendering
// The callback copies the profile into a mutex-protected buffer, the renderer
// picks up the latest one each frame (~60 Hz).
//
// Controls:
//   mouse wheel  zoom at cursor        left drag   pan
//   a  autoscale on/off                c  connect points with lines
//   i  intensity colouring on/off      f  flip Z axis (sensor at top)
//   v  show invalid points             space/p  pause
//   + / -  point size                  q / ESC  quit

#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/scan_control.hpp"

#include <GL/glut.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// Shared data between i2w callback and renderer
// ---------------------------------------------------------------------------
std::atomic_bool running{true};

void Stop(int) { running.store(false); }

struct ProfilePoint
{
    float x;
    float z;
    float intensity;
    bool valid;
};

struct SharedProfile
{
    std::mutex m;
    std::vector<ProfilePoint> pts;
    std::uint64_t sequence = 0;
    std::uint64_t callback_drops = 0;
    double encoder_mm = 0.0;
    std::uint64_t frames = 0; // number of profiles received
};

SharedProfile g_shared;

// ---------------------------------------------------------------------------
// i2w node
// ---------------------------------------------------------------------------
class LaserProfilingSensor final : public i2w::SystemBase
{
public:
    explicit LaserProfilingSensor(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:
    i2w::LifecycleResult OnSetup()
    {
        i2w::SubscriptionOptions opts;
        opts.plane = i2w::EndpointPlane::Local;
        opts.reliability = i2w::Reliability::BestEffort;
        opts.queue_depth = 32;
        opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

        auto subscription = runtime().subscribe<crawler_i2w_msgs::I2wScanControlProfile>(
            "profile",
            [](const i2w::Sample<crawler_i2w_msgs::I2wScanControlProfile> &sample)
            {
                const auto &msg = sample.value;

                std::vector<ProfilePoint> pts;
                pts.reserve(msg.point_count);
                for (std::uint32_t i = 0; i < msg.point_count; ++i)
                {
                    const auto &p = msg.points[i];
                    pts.push_back({static_cast<float>(p.x_m),
                                   static_cast<float>(p.z_m),
                                   static_cast<float>(p.intensity),
                                   p.valid != 0});
                }

                std::lock_guard<std::mutex> lk(g_shared.m);
                g_shared.pts.swap(pts);
                g_shared.sequence = msg.sequence;
                g_shared.callback_drops = msg.callback_drops;
                g_shared.encoder_mm = static_cast<double>(msg.encoder_displacement_mm);
                ++g_shared.frames;
            },
            opts);
        sub_ = std::move(subscription.value());

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return i2w::Ok();
    }

    i2w::Subscription<crawler_i2w_msgs::I2wScanControlProfile> sub_{};
};

std::thread g_i2w_thread;

void I2wThread()
{
    i2w::Config cfg;
    cfg.node_name = "laserProflingSensor";
    cfg.ns = "/scan_control";

    LaserProfilingSensor lps(cfg);
    lps.Setup();

    while (running)
    {
        lps.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ---------------------------------------------------------------------------
// View state (GLUT thread only)
// ---------------------------------------------------------------------------
struct ViewState
{
    std::vector<ProfilePoint> pts;
    std::uint64_t seq = 0;
    std::uint64_t drops = 0;
    double encoder_mm = 0.0;
    std::uint64_t frames_seen = 0;

    // rate measurement
    std::uint64_t rate_frames = 0;
    std::chrono::steady_clock::time_point rate_t = std::chrono::steady_clock::now();
    double rx_hz = 0.0;

    // camera (metres)
    double cx = 0.04, cz = 0.20;
    double half_x = 0.05;

    int win_w = 1280, win_h = 720;
    bool autoscale = true;
    bool color_by_intensity = true;
    bool connect = false;
    bool flip_z = false;
    bool show_invalid = false;
    bool paused = false;
    float point_size = 3.0f;

    bool dragging = false;
    int last_x = 0, last_y = 0;
    int mouse_x = -1, mouse_y = -1;
} g_view;

double Aspect() { return static_cast<double>(g_view.win_w) / std::max(1, g_view.win_h); }
double HalfZ() { return g_view.half_x / Aspect(); }

// GLUT window pixel (origin top-left) -> world metres
void ScreenToWorld(int px, int py, double &wx, double &wz)
{
    const auto &v = g_view;
    const double hx = v.half_x, hz = HalfZ();
    wx = v.cx - hx + px * (2.0 * hx / v.win_w);
    const double from_bottom = v.win_h - py;
    if (v.flip_z)
        wz = v.cz + hz - from_bottom * (2.0 * hz / v.win_h);
    else
        wz = v.cz - hz + from_bottom * (2.0 * hz / v.win_h);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void DrawText(double x, double y, const std::string &s, void *font = GLUT_BITMAP_9_BY_15)
{
    glRasterPos2d(x, y);
    for (char c : s)
        glutBitmapCharacter(font, c);
}

double NiceStep(double range)
{
    const double raw = range / 10.0;
    const double p = std::pow(10.0, std::floor(std::log10(raw)));
    const double f = raw / p;
    const double n = f < 1.5 ? 1.0 : f < 3.0 ? 2.0 : f < 7.0 ? 5.0 : 10.0;
    return n * p;
}

void JetColor(float t, float &r, float &g, float &b)
{
    t = std::clamp(t, 0.0f, 1.0f);
    r = std::clamp(1.5f - std::fabs(4.0f * t - 3.0f), 0.0f, 1.0f);
    g = std::clamp(1.5f - std::fabs(4.0f * t - 2.0f), 0.0f, 1.0f);
    b = std::clamp(1.5f - std::fabs(4.0f * t - 1.0f), 0.0f, 1.0f);
}

void Shutdown()
{
    running = false;
    std::exit(0); // atexit handler joins the i2w thread
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
void Display()
{
    auto &v = g_view;

    // 1. Grab latest profile
    if (!v.paused)
    {
        std::lock_guard<std::mutex> lk(g_shared.m);
        if (g_shared.frames != v.frames_seen)
        {
            v.pts = g_shared.pts;
            v.seq = g_shared.sequence;
            v.drops = g_shared.callback_drops;
            v.encoder_mm = g_shared.encoder_mm;
            v.frames_seen = g_shared.frames;
        }
    }

    // 2. Bounds of valid points
    std::uint32_t valid_count = 0;
    double minx = 1e9, maxx = -1e9, minz = 1e9, maxz = -1e9;
    float max_i = 1.0f;
    for (const auto &p : v.pts)
    {
        if (!p.valid)
            continue;
        ++valid_count;
        minx = std::min(minx, (double)p.x);
        maxx = std::max(maxx, (double)p.x);
        minz = std::min(minz, (double)p.z);
        maxz = std::max(maxz, (double)p.z);
        max_i = std::max(max_i, p.intensity);
    }

    const double aspect = Aspect();
    if (v.autoscale && valid_count > 0)
    {
        const double tx = 0.5 * (minx + maxx);
        const double tz = 0.5 * (minz + maxz);
        const double hx = 0.5 * (maxx - minx) * 1.15 + 1e-4;
        const double hz = 0.5 * (maxz - minz) * 1.15 + 1e-4;
        const double target_half = std::max(hx, hz * aspect); // keep 1:1 scale
        const double a = 0.25;                                // smoothing
        v.cx += a * (tx - v.cx);
        v.cz += a * (tz - v.cz);
        v.half_x += a * (target_half - v.half_x);
    }

    const double hx = v.half_x;
    const double hz = HalfZ();

    // 3. World projection (equal metres per pixel in X and Z)
    glViewport(0, 0, v.win_w, v.win_h);
    glClearColor(0.07f, 0.07f, 0.09f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double bottom = v.cz - hz, top = v.cz + hz;
    if (v.flip_z)
        std::swap(bottom, top);
    glOrtho(v.cx - hx, v.cx + hx, bottom, top, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 4. Grid
    const double step = NiceStep(2.0 * hx);
    const double x0 = std::floor((v.cx - hx) / step) * step;
    const double z0 = std::floor((v.cz - hz) / step) * step;

    glLineWidth(1.0f);
    glColor3f(0.18f, 0.18f, 0.23f);
    glBegin(GL_LINES);
    for (double x = x0; x <= v.cx + hx; x += step)
    {
        glVertex2d(x, v.cz - hz);
        glVertex2d(x, v.cz + hz);
    }
    for (double z = z0; z <= v.cz + hz; z += step)
    {
        glVertex2d(v.cx - hx, z);
        glVertex2d(v.cx + hx, z);
    }
    glEnd();

    // Grid labels (mm)
    char buf[256];
    glColor3f(0.55f, 0.55f, 0.62f);
    double lx, lz;
    for (double x = x0; x <= v.cx + hx; x += step)
    {
        if (x < v.cx - hx)
            continue;
        double dummy;
        ScreenToWorld(0, v.win_h - 6, dummy, lz);
        std::snprintf(buf, sizeof(buf), "%.1f", x * 1000.0);
        DrawText(x + 3.0 * (2.0 * hx / v.win_w), lz, buf, GLUT_BITMAP_8_BY_13);
    }
    for (double z = z0; z <= v.cz + hz; z += step)
    {
        if (z < v.cz - hz)
            continue;
        double dummy;
        ScreenToWorld(6, 0, lx, dummy);
        std::snprintf(buf, sizeof(buf), "%.1f", z * 1000.0);
        DrawText(lx, z, buf, GLUT_BITMAP_8_BY_13);
    }

    auto set_color = [&](const ProfilePoint &p)
    {
        if (!v.color_by_intensity)
        {
            glColor3f(0.2f, 1.0f, 0.4f);
            return;
        }
        float r, g, b;
        JetColor(p.intensity / max_i, r, g, b);
        glColor3f(r, g, b);
    };

    // 5. Polyline (broken at invalid points)
    if (v.connect)
    {
        glLineWidth(1.5f);
        glBegin(GL_LINE_STRIP);
        for (const auto &p : v.pts)
        {
            if (!p.valid)
            {
                glEnd();
                glBegin(GL_LINE_STRIP);
                continue;
            }
            set_color(p);
            glVertex2f(p.x, p.z);
        }
        glEnd();
    }

    // 6. Points
    glPointSize(v.point_size);
    glBegin(GL_POINTS);
    for (const auto &p : v.pts)
    {
        if (!p.valid)
        {
            if (!v.show_invalid)
                continue;
            glColor3f(0.4f, 0.4f, 0.4f);
        }
        else
        {
            set_color(p);
        }
        glVertex2f(p.x, p.z);
    }
    glEnd();

    // 7. Text overlay in pixel coordinates
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, v.win_w, 0, v.win_h);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int y = v.win_h - 20;
    glColor3f(0.95f, 0.95f, 0.95f);
    std::snprintf(buf, sizeof(buf), "seq %llu   points %zu (valid %u)   rx %.1f Hz   drops %llu   encoder %.2f mm%s",
                  (unsigned long long)v.seq, v.pts.size(), valid_count, v.rx_hz,
                  (unsigned long long)v.drops, v.encoder_mm, v.paused ? "   [PAUSED]" : "");
    DrawText(10, y, buf);
    y -= 18;

    if (valid_count > 0)
    {
        std::snprintf(buf, sizeof(buf), "X [%.2f .. %.2f] mm   Z [%.2f .. %.2f] mm   grid %.2f mm",
                      minx * 1000, maxx * 1000, minz * 1000, maxz * 1000, step * 1000);
        DrawText(10, y, buf);
        y -= 18;
    }

    if (v.mouse_x >= 0)
    {
        double mx, mz;
        ScreenToWorld(v.mouse_x, v.mouse_y, mx, mz);
        std::snprintf(buf, sizeof(buf), "cursor  x=%.2f mm  z=%.2f mm", mx * 1000, mz * 1000);
        DrawText(10, y, buf);
    }

    glColor3f(0.6f, 0.6f, 0.66f);
    std::snprintf(buf, sizeof(buf),
                  "wheel zoom | drag pan | a autoscale:%s | c lines:%s | i intensity:%s | f flip:%s | v invalid:%s | +/- size | space pause | q quit",
                  v.autoscale ? "on" : "off", v.connect ? "on" : "off",
                  v.color_by_intensity ? "on" : "off", v.flip_z ? "on" : "off",
                  v.show_invalid ? "on" : "off");
    DrawText(10, 10, buf, GLUT_BITMAP_8_BY_13);

    glutSwapBuffers();
}

// ---------------------------------------------------------------------------
// GLUT callbacks
// ---------------------------------------------------------------------------
void Reshape(int w, int h)
{
    g_view.win_w = std::max(1, w);
    g_view.win_h = std::max(1, h);
    glutPostRedisplay();
}

void Timer(int)
{
    if (!running)
        Shutdown();

    auto &v = g_view;
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - v.rate_t).count();
    if (dt >= 1.0)
    {
        std::uint64_t frames;
        {
            std::lock_guard<std::mutex> lk(g_shared.m);
            frames = g_shared.frames;
        }
        v.rx_hz = (frames - v.rate_frames) / dt;
        v.rate_frames = frames;
        v.rate_t = now;
    }

    glutPostRedisplay();
    glutTimerFunc(16, Timer, 0);
}

void Keyboard(unsigned char key, int, int)
{
    auto &v = g_view;
    switch (key)
    {
    case 'q':
    case 27:
        Shutdown();
        break;
    case 'a': v.autoscale = !v.autoscale; break;
    case 'c': v.connect = !v.connect; break;
    case 'i': v.color_by_intensity = !v.color_by_intensity; break;
    case 'f': v.flip_z = !v.flip_z; break;
    case 'v': v.show_invalid = !v.show_invalid; break;
    case ' ':
    case 'p': v.paused = !v.paused; break;
    case '+':
    case '=': v.point_size = std::min(20.0f, v.point_size + 1.0f); break;
    case '-': v.point_size = std::max(1.0f, v.point_size - 1.0f); break;
    default: break;
    }
    glutPostRedisplay();
}

void Mouse(int button, int state, int x, int y)
{
    auto &v = g_view;

    // Wheel: buttons 3 (up) / 4 (down) in GLUT/freeglut
    if (button == 3 || button == 4)
    {
        if (state != GLUT_DOWN)
            return;
        double bx, bz, ax, az;
        ScreenToWorld(x, y, bx, bz);
        v.half_x *= (button == 3) ? 0.9 : 1.1;
        v.half_x = std::clamp(v.half_x, 1e-5, 10.0);
        ScreenToWorld(x, y, ax, az);
        v.cx += bx - ax; // keep point under cursor fixed
        v.cz += bz - az;
        v.autoscale = false;
        return;
    }

    if (button == GLUT_LEFT_BUTTON)
    {
        v.dragging = (state == GLUT_DOWN);
        v.last_x = x;
        v.last_y = y;
        if (v.dragging)
            v.autoscale = false;
    }
}

void Motion(int x, int y)
{
    auto &v = g_view;
    if (v.dragging)
    {
        const double dx = (x - v.last_x) * (2.0 * v.half_x / v.win_w);
        const double dz = (y - v.last_y) * (2.0 * HalfZ() / v.win_h);
        v.cx -= dx;
        v.cz += v.flip_z ? -dz : dz;
        v.last_x = x;
        v.last_y = y;
    }
    v.mouse_x = x;
    v.mouse_y = y;
}

void PassiveMotion(int x, int y)
{
    g_view.mouse_x = x;
    g_view.mouse_y = y;
}

// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    g_i2w_thread = std::thread(I2wThread);
    std::atexit([]
                {
                    running = false;
                    if (g_i2w_thread.joinable())
                        g_i2w_thread.join();
                });

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(g_view.win_w, g_view.win_h);
    glutCreateWindow("scanCONTROL profile viewer");

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
    glutKeyboardFunc(Keyboard);
    glutMouseFunc(Mouse);
    glutMotionFunc(Motion);
    glutPassiveMotionFunc(PassiveMotion);
    glutTimerFunc(16, Timer, 0);

    glutMainLoop(); // does not return; exit goes through atexit handler
    return 0;
}