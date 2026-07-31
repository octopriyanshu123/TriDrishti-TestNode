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
        opts.plane = i2w::EndpointPlane::Local;
        opts.reliability = i2w::Reliability::BestEffort;
        opts.queue_depth = 32;
        opts.overflow_policy = i2w::OverflowPolicy::DropOldest;
        auto subscription = runtime().subscribe<crawler_i2w_msgs::JoyMsgs>(
            "/joy",
            [this](const i2w::Sample<crawler_i2w_msgs::JoyMsgs> &sample)
            {

                if(sample.value.button0){
                    linearScale++;
                    std::cout<<"linearScale -> "<<std::endl;
                }
                 if(sample.value.button3){
                    linearScale--;

                }
                 if(sample.value.button1){
                    angularScale++;
                }
                 if(sample.value.button4){
                    angularScale--;
                }
                cmd_vel_.linearVelocity = -normalize(sample.value.axis2, 10*linearScale );
                cmd_vel_.angularVelocity = -normalize(sample.value.axis0, 5*angularScale );
                cmd_vel_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
                (void)publisher_.publish(cmd_vel_, static_cast<std::int64_t>(cmd_vel_.timestamp));
                std::cout<<"linearScale -> "<<linearScale<<" angularScale -> "<<angularScale<<std::endl;

            },
            opts);
        sub_ = std::move(subscription.value());

        i2w::PublisherOptions cmdVelPubOpt;
        cmdVelPubOpt.plane = i2w::EndpointPlane::Local;

        auto publisher = runtime().advertise<crawler_i2w_msgs::cmd_vel>("/cmd_vel", cmdVelPubOpt);

        publisher_ = std::move(publisher.value());

        return i2w::Ok();
    }
    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return i2w::Ok();
    }
    //   void OnDispose() override;
    i2w::Publisher<crawler_i2w_msgs::cmd_vel> publisher_{};
    i2w::Subscription<crawler_i2w_msgs::JoyMsgs> sub_{};
};

int main()
{

    i2w::Config joySubConfig;
    joySubConfig.node_name = "Joy_Sub";
    joySubConfig.ns = "robot";
    joySubConfig.transport.network_profile_file = "/home/octo/TriDristi-ws/src/i2w/examples/config/ecal-network-udp.yaml";

    JoyNode js(joySubConfig);

    js.Setup();

    while (running)
    {
        js.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}