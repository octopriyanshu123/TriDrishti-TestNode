#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/robot/cmd_vel.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/joystick.h>
#include <utility>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <signal.h>
std::atomic_bool running{true};

void Stop(int)
{
    running.store(false);
}
crawler_i2w_msgs::cmd_vel cmd_vel_;
float normalize(int16_t value, float max_output)
{

    return (static_cast<float>(value) / 32767.0f) * max_output;
}

class JoyNode final : public i2w::SystemBase
{
public:
    explicit JoyNode(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

    int linearScale = 1;
    int angularScale = 1;

private:
    i2w::LifecycleResult OnSetup()
    {

        i2w::SubscriptionOptions opts;
        opts.plane = i2w::EndpointPlane::Network;
        opts.reliability = i2w::Reliability::BestEffort;
        opts.queue_depth = 32;
        opts.overflow_policy = i2w::OverflowPolicy::DropOldest;
        auto subscription = runtime().subscribe<crawler_i2w_msgs::JoyMsgs>(
            "/joy",
            [this](const i2w::Sample<crawler_i2w_msgs::JoyMsgs> &sample)
            {
                crawler_i2w_msgs::JoyMsgs joy = sample.value;
               
                (void)publisher_.publish(joy, static_cast<std::int64_t>(cmd_vel_.timestamp));
                std::cout << "Received Joy Message: axis0=" << joy.axis0 << ", axis2=" << joy.axis2
                          << ", button0=" << joy.button0 << ", button1=" << joy.button1
                          << ", button3=" << joy.button3 << ", button4=" << joy.button4
                          << ", button5=" << joy.button5 << ", button6=" << joy.button6
                          << std::endl;

            },
            opts);
        sub_ = std::move(subscription.value());

        i2w::PublisherOptions joyLocal;
        joyLocal.plane = i2w::EndpointPlane::Local;

        auto publisher = runtime().advertise<crawler_i2w_msgs::JoyMsgs>("/joy", joyLocal);

        publisher_ = std::move(publisher.value());

        return i2w::Ok();
    }
    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return i2w::Ok();
    }
    //   void OnDispose() override;
    i2w::Publisher<crawler_i2w_msgs::JoyMsgs> publisher_{};
    i2w::Subscription<crawler_i2w_msgs::JoyMsgs> sub_{};

};

int main()
{

    i2w::Config joySubConfig;
    joySubConfig.node_name = "joy_network_to_local";
    joySubConfig.ns = "";
    joySubConfig.transport.network_profile_file = "/home/tridrishti/tridrishti_ws/src/TriDrishti-TestNode/config/ecal-network-udp.yaml";


    JoyNode js(joySubConfig);

    js.Setup();

    while (running) 
    {
        js.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}