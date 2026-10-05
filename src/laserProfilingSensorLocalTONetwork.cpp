#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/scan_control.hpp"
#include "crawler_i2w_msgs/robot/edge_status.hpp"
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
#include <iostream>

std::atomic_bool running{true};

void Stop(int) { running.store(false); }

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
            [this](const i2w::Sample<crawler_i2w_msgs::I2wScanControlProfile> &sample)
            {
                publisher_.publish(sample.value, sample.header.stamp_ns);
                publisher_edge_status(sample.value);
            },
            opts);
        sub_ = std::move(subscription.value());

        i2w::PublisherOptions pub_opts;
        pub_opts.plane = i2w::EndpointPlane::Network;
        auto publisher = runtime().advertise<crawler_i2w_msgs::I2wScanControlProfile>("/profile", pub_opts);
        if (!publisher)
        {
            std::fprintf(stderr, "Failed to advertise publisher:\n");
            return i2w::Fail();
        }
        publisher_ = std::move(publisher.value());

        auto edge_publisher = runtime().advertise<crawler_i2w_msgs::edge_status>("/edge_status", pub_opts);
        if (!edge_publisher)
        {
            std::fprintf(stderr, "Failed to advertise Edge Publisher :\n");
            return i2w::Fail();
        }
        edge_publisher_ = std::move(edge_publisher.value());

        return i2w::Ok();
    }

    void publisher_edge_status(const crawler_i2w_msgs::I2wScanControlProfile &profile)
    {
        crawler_i2w_msgs::edge_status edge_status_msg = DetectEdges(profile.points);

        edge_publisher_.publish(edge_status_msg, runtime().clock().now().ns);
        // std::cout << "Edge Status Published: Found = " << edge_status_msg.found
        //           << ", Left X = " << edge_status_msg.left_x
        //           << ", Right X = " << edge_status_msg.right_x
        //           << ", Baseline Z = " << edge_status_msg.baseline_z
        //           << ", Height = " << edge_status_msg.height
        //           << std::endl;
    }

    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return i2w::Ok();
    }

    crawler_i2w_msgs::edge_status DetectEdges(const std::array<crawler_i2w_msgs::ScanControlPoint, crawler_i2w_msgs::kI2wScanControlMaxPoints> &pts)
    {
        crawler_i2w_msgs::edge_status e;

        // Base level = median Z of all valid points
        std::vector<float> zs;

        zs.reserve(pts.size());
        for (const auto &p : pts)
            if (p.valid)
                zs.push_back(p.z_m);
        if (zs.size() < 10)
            return e;
        std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
        e.baseline_z = zs[zs.size() / 2];

        // Find the longest run of consecutive "high" points
        int best_start = -1, best_end = -1, best_len = 0;
        int cur_start = -1, cur_end = -1, cur_len = 0;

        for (int i = 0; i < static_cast<int>(pts.size()); ++i)
        {
            const auto &p = pts[i];
            if (!p.valid)
                continue; // invalid points don't break the run
            const bool high = std::fabs(e.baseline_z - p.z_m) > kEdgeThreshold;
            if (high)
            {
                if (cur_len == 0)
                    cur_start = i;
                cur_end = i;
                ++cur_len;
                if (cur_len > best_len)
                {
                    best_len = cur_len;
                    best_start = cur_start;
                    best_end = cur_end;
                }
            }
            else
            {
                cur_len = 0;
            }
        }
        if (best_len < kEdgeMinRun)
            return e;

        // Left/right edge = smallest/largest X of the high run
        e.left_x = 1e9f;
        e.right_x = -1e9f;
        for (int i = best_start; i <= best_end; ++i)
        {
            const auto &p = pts[i];
            if (!p.valid)
                continue;
            const float d = std::fabs(e.baseline_z - p.z_m);
            if (d <= kEdgeThreshold)
                continue;
            e.left_x = std::min(e.left_x, p.x_m);
            e.right_x = std::max(e.right_x, p.x_m);
            e.height = std::max(e.height, d);
        }

        e.found = true;
        return e;
    }

    i2w::Subscription<crawler_i2w_msgs::I2wScanControlProfile> sub_{};
    i2w::Publisher<crawler_i2w_msgs::I2wScanControlProfile> publisher_{};
    i2w::Publisher<crawler_i2w_msgs::edge_status> edge_publisher_{};
    float kEdgeThreshold = 0.0005f; // 0.5 mm away from base level counts as "high"
    int kEdgeMinRun = 3;            // need >= 3 high points in a row (ignores noise)
};

int main()
{

    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    i2w::Config i2w_config;
    i2w_config.node_name = "ltn";
    i2w_config.ns = "/scan_control";
    i2w_config.transport.network_profile_file = "/home/octo/TriDristi-ws/src/TriDrishti-TestNode/config/ecal-network-udp.yaml";

    LaserProfilingSensor lps(i2w_config);
    lps.Setup();

    while (running)
    {
        lps.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}