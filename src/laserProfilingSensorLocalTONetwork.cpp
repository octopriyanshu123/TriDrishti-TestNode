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
                std::cout<<"Time Stamp "<<sample.header.stamp_ns<<std::endl;
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

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return i2w::Ok();
    }
    i2w::Subscription<crawler_i2w_msgs::I2wScanControlProfile> sub_{};
    i2w::Publisher<crawler_i2w_msgs::I2wScanControlProfile> publisher_{};
};

int main()
{

    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    i2w::Config i2w_config;
    i2w_config.node_name = "ltn";
    i2w_config.ns = "";
    i2w_config.transport.network_profile_file = "/home/octo/TriDristi-ws/src/TriDrishti-TestNode/config/ecal-network-udp.yaml";

    LaserProfilingSensor lps(i2w_config);
    lps.Setup();

    while (running)
    {
        lps.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}