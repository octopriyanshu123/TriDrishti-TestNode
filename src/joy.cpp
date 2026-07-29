#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
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

std::atomic_bool running{true};
namespace
{

    bool IsDesiredAxis(std::uint8_t axis)
    {
        return axis == 0 || axis == 3;
    }

    bool IsDesiredButton(std::uint8_t button)
    {
        return button == 0 || button == 1 || button == 3 || button == 4;
    }

} // namespace

void Stop(int)
{
    running.store(false);
}

void JoyCallback(const i2w::Sample<crawler_i2w_msgs::JoyMsgs> &sample) noexcept
{
    std::cout << "HI" << std::endl;
}

class JoyNode final : public i2w::SystemBase
{
public:
    explicit JoyNode(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
                std::cout<<"JoyNode Constructor "<<std::endl;

    }

private:
    i2w::LifecycleResult OnSetup()
    {
        std::cout<<"JoyNode OnSetup "<<std::endl;

        fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0)
        {
            std::printf("failed to open joystick device: %s\n", device_path_.c_str());
            return i2w::Fail();
        }
        i2w::PublisherOptions opts;
        opts.plane = i2w::EndpointPlane::Local;

        auto publisher = runtime().advertise<crawler_i2w_msgs::JoyMsgs>("/joy", opts);

        publisher_ = std::move(publisher.value());
        return i2w::Ok();
    }
    i2w::LifecycleResult OnTick() noexcept
    {
        js_event event{};
        while (true)
        {
            const ssize_t bytes = read(fd_, &event, sizeof(event));
            if (bytes == sizeof(event))
            {
                if (!PublishEvent(&event))
                {
                    return i2w::Fail();
                }
                continue;
            }

            if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                return i2w::Ok();
            }

            if (bytes == 0)
            {
                return i2w::Ok();
            }


            std::printf("joystick read failed: %s\n", std::strerror(errno));
            return i2w::Fail();
        }
    }

    void OnDispose()
    {
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    int count = 0;
    bool PublishEvent(const void *raw_event) noexcept
    {
        const auto &event = *static_cast<const js_event *>(raw_event);
        const std::uint8_t type = event.type & ~JS_EVENT_INIT;

        if (type == JS_EVENT_AXIS && event.number < 8 && IsDesiredAxis(event.number))
        {
            const float value = static_cast<float>(event.value);
            float &axis = (event.number == 0) ? joy_.axis0 : joy_.axis2;
            if (axis == value)
            {
                return true;
            }
            axis = value;
        }
        else if (type == JS_EVENT_BUTTON && event.number < 12 &&
                 IsDesiredButton(event.number))
        {
            const bool pressed = event.value != 0;
            bool *button = nullptr;
            switch (event.number)
            {
            case 0:
                button = &joy_.button0;
                break;
            case 1:
                button = &joy_.button1;
                break;
            case 3:
                button = &joy_.button3;
                break;
            case 4:
                button = &joy_.button4;
                break;
            default:
                break;
            }
            if (button == nullptr || *button == pressed)
            {
                return true;
            }
            *button = pressed;
        }
        else
        {
            return true;
        }

        joy_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
        const auto sent = publisher_.publish(joy_, static_cast<std::int64_t>(joy_.timestamp));
                        std::cout<<"Published "<<count++<<std::endl;


    

        return true;
    }

    std::string device_path_{"/dev/input/js0"};
    int fd_{-1};
    crawler_i2w_msgs::JoyMsgs joy_{};
    i2w::Publisher<crawler_i2w_msgs::JoyMsgs> publisher_{};
};

int main()
{

    i2w::Config joySubConfig;
    joySubConfig.node_name = "Joy_pub";
    joySubConfig.ns = "robot";
    joySubConfig.transport.network_profile_file = "/home/octo/TriDristi-ws/src/joyToCmdVel/config/config.json";

    JoyNode js(joySubConfig);

    js.Setup();

    while (running)
    {
        js.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}