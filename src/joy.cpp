#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <csignal>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>


// -----------------------------------------------------------------------------
// Global running flag
// -----------------------------------------------------------------------------

std::atomic_bool running{true};


// -----------------------------------------------------------------------------
// Joystick filtering
// -----------------------------------------------------------------------------

namespace
{

bool IsDesiredAxis(std::uint8_t axis)
{
    // Linux joystick:
    //   axis 0 -> JoyMsgs::axis0
    //   axis 4 -> JoyMsgs::axis2
    return axis == 0 || axis == 4;
}

bool IsDesiredButton(std::uint8_t button)
{
    // Linux joystick buttons used by this node:
    //   button 0 -> JoyMsgs::button0
    //   button 1 -> JoyMsgs::button1
    //   button 3 -> JoyMsgs::button4
    //   button 4 -> JoyMsgs::button3
    return button == 0 ||
           button == 1 ||
           button == 3 ||
           button == 4;
}

} // namespace


// -----------------------------------------------------------------------------
// Signal handler
// -----------------------------------------------------------------------------

void Stop(int)
{
    running.store(false);
}


// -----------------------------------------------------------------------------
// JoyNode
// -----------------------------------------------------------------------------

class JoyNode final : public i2w::SystemBase
{
public:

    explicit JoyNode(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
        std::cout << "JoyNode Constructor" << std::endl;
    }

private:

    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------

    i2w::LifecycleResult OnSetup()
    {
        std::cout << "JoyNode OnSetup" << std::endl;

        // Open Linux joystick device.
        fd_ = open(
            device_path_.c_str(),
            O_RDONLY | O_NONBLOCK);

        if (fd_ < 0)
        {
            std::fprintf(
                stderr,
                "Failed to open joystick device '%s': %s\n",
                device_path_.c_str(),
                std::strerror(errno));

            return i2w::Fail();
        }

        std::cout << "Joystick opened: "
                  << device_path_
                  << std::endl;

        // ---------------------------------------------------------------------
        // Advertise Joy topic
        // ---------------------------------------------------------------------

        i2w::PublisherOptions opts;
        opts.plane = i2w::EndpointPlane::Network;

        auto publisher =
            runtime().advertise<crawler_i2w_msgs::JoyMsgs>(
                "/joy",
                opts);

        if (!publisher)
        {
            std::fprintf(
                stderr,
                "Failed to advertise /joy\n");

            close(fd_);
            fd_ = -1;

            return i2w::Fail();
        }

        publisher_ = std::move(publisher.value());

        std::cout << "Joy publisher created: /joy"
                  << std::endl;

        return i2w::Ok();
    }


    // -------------------------------------------------------------------------
    // Tick
    // -------------------------------------------------------------------------

    i2w::LifecycleResult OnTick() noexcept
    {
        js_event event{};

        // Drain all available joystick events.
        while (true)
        {
            const ssize_t bytes =
                read(
                    fd_,
                    &event,
                    sizeof(event));

            // -----------------------------------------------------------------
            // Valid joystick event
            // -----------------------------------------------------------------

            if (bytes == static_cast<ssize_t>(sizeof(event)))
            {
                if (!PublishEvent(&event))
                {
                    std::fprintf(
                        stderr,
                        "Failed to publish joystick event\n");

                    return i2w::Fail();
                }

                // We received real data from the joystick.
                // A future disconnect should therefore publish zero state.
                zero_published_ = false;

                continue;
            }

            // -----------------------------------------------------------------
            // No more events currently available
            // -----------------------------------------------------------------

            if (bytes < 0 &&
                (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                return i2w::Ok();
            }

            // -----------------------------------------------------------------
            // Device closed / disconnected
            // -----------------------------------------------------------------

            if (bytes == 0)
            {
                std::fprintf(
                    stderr,
                    "Joystick device closed or disconnected\n");

                if (!zero_published_)
                {
                    PublishZeroState();
                    zero_published_ = true;
                }

                return i2w::Ok();
            }

            // -----------------------------------------------------------------
            // Partial read
            // -----------------------------------------------------------------

            if (bytes > 0)
            {
                std::fprintf(
                    stderr,
                    "Incomplete joystick event read: %zd bytes\n",
                    bytes);

                return i2w::Fail();
            }

            // -----------------------------------------------------------------
            // Real read error
            // -----------------------------------------------------------------

            std::fprintf(
                stderr,
                "Joystick read failed: errno=%d (%s)\n",
                errno,
                std::strerror(errno));

            if (!zero_published_)
            {
                PublishZeroState();
                zero_published_ = true;
            }

            return i2w::Fail();
        }
    }


    // -------------------------------------------------------------------------
    // Dispose
    // -------------------------------------------------------------------------

    void OnDispose()
    {
        std::cout << "JoyNode OnDispose"
                  << std::endl;

        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }


    // -------------------------------------------------------------------------
    // Publish joystick event
    // -------------------------------------------------------------------------

    bool PublishEvent(const void *raw_event) noexcept
    {
        if (raw_event == nullptr)
        {
            return false;
        }

        const auto &event =
            *static_cast<const js_event *>(raw_event);

        // Remove JS_EVENT_INIT flag.
        //
        // JS_EVENT_INIT can be combined with:
        //   JS_EVENT_AXIS
        //   JS_EVENT_BUTTON
        //
        // We only care about the actual event type.
        const std::uint8_t type =
            event.type & ~JS_EVENT_INIT;


        // =====================================================================
        // AXIS EVENT
        // =====================================================================

        if (type == JS_EVENT_AXIS)
        {
            if (event.number >= 8 ||
                !IsDesiredAxis(event.number))
            {
                return true;
            }

            std::cout
                << "Axis "
                << static_cast<int>(event.number)
                << " = "
                << event.value
                << std::endl;


            const float value =
                static_cast<float>(event.value);


            float *axis = nullptr;


            if (event.number == 0)
            {
                axis = &joy_.axis0;
            }
            else if (event.number == 4)
            {
                axis = &joy_.axis2;
            }


            if (axis == nullptr)
            {
                return true;
            }


            // Don't publish if value has not changed.
            if (*axis == value)
            {
                return true;
            }


            *axis = value;
        }


        // =====================================================================
        // BUTTON EVENT
        // =====================================================================

        else if (type == JS_EVENT_BUTTON)
        {
            if (event.number >= 12 ||
                !IsDesiredButton(event.number))
            {
                return true;
            }

            std::cout
                << "Button "
                << static_cast<int>(event.number)
                << " = "
                << event.value
                << std::endl;


            const bool pressed =
                event.value != 0;


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
                    // Linux button 3 -> Joy button4
                    button = &joy_.button4;
                    break;

                case 4:
                    // Linux button 4 -> Joy button3
                    button = &joy_.button3;
                    break;

                default:
                    return true;
            }


            // Don't publish if state has not changed.
            if (*button == pressed)
            {
                return true;
            }


            *button = pressed;
        }


        // =====================================================================
        // Unknown event
        // =====================================================================

        else
        {
            return true;
        }


        // =====================================================================
        // Update timestamp
        // =====================================================================

        joy_.timestamp =
            static_cast<std::uint64_t>(
                runtime().clock().now().ns);


        // =====================================================================
        // Publish
        // =====================================================================

        const auto sent =
            publisher_.publish(
                joy_,
                static_cast<std::int64_t>(
                    joy_.timestamp));


        if (!sent)
        {
            std::fprintf(
                stderr,
                "Failed to publish /joy\n");

            return false;
        }


        ++publish_count_;


        std::cout
            << "Published joystick message #"
            << publish_count_
            << std::endl;


        return true;
    }


    // -------------------------------------------------------------------------
    // Publish zero state
    // -------------------------------------------------------------------------

    void PublishZeroState() noexcept
    {
        // Reset all axes/buttons.
        joy_ = crawler_i2w_msgs::JoyMsgs{};


        // Update timestamp.
        joy_.timestamp =
            static_cast<std::uint64_t>(
                runtime().clock().now().ns);


        std::cout
            << "Publishing zero joystick state"
            << std::endl;


        publisher_.publish(
            joy_,
            static_cast<std::int64_t>(
                joy_.timestamp));
    }


private:

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    std::string device_path_{"/dev/input/js0"};


    // -------------------------------------------------------------------------
    // Linux joystick FD
    // -------------------------------------------------------------------------

    int fd_{-1};


    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    bool zero_published_{false};


    crawler_i2w_msgs::JoyMsgs joy_{};


    // -------------------------------------------------------------------------
    // i2w publisher
    // -------------------------------------------------------------------------

    i2w::Publisher<crawler_i2w_msgs::JoyMsgs> publisher_{};


    // -------------------------------------------------------------------------
    // Debug / statistics
    // -------------------------------------------------------------------------

    std::uint64_t publish_count_{0};
};


// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main()
{
    // -------------------------------------------------------------------------
    // i2w configuration
    // -------------------------------------------------------------------------

    i2w::Config joy_pub_config;

    joy_pub_config.node_name = "Joy_pub";
    joy_pub_config.ns = "";


    // -------------------------------------------------------------------------
    // Network profile
    // -------------------------------------------------------------------------

    const auto network_profile_file_path =
        std::string(CONFIG_DIR) +
        "/ecal-network-udp.yaml";


    std::cout
        << "Network Profile File Path: "
        << network_profile_file_path
        << std::endl;


    joy_pub_config
        .transport
        .network_profile_file =
            network_profile_file_path;


    // -------------------------------------------------------------------------
    // Create node
    // -------------------------------------------------------------------------

    JoyNode joy_node(joy_pub_config);


    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------

    if (!joy_node.Setup().ok)
    {
        std::fprintf(
            stderr,
            "JoyNode setup failed\n");

        return 2;
    }


    std::cout
        << "JoyNode setup successful"
        << std::endl;


    // -------------------------------------------------------------------------
    // Signal handling
    // -------------------------------------------------------------------------

    std::signal(
        SIGINT,
        Stop);

    std::signal(
        SIGTERM,
        Stop);


    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------

    while (running.load())
    {
        const auto result =
            joy_node.Tick();


        if (!result.ok)
        {
            std::fprintf(
                stderr,
                "JoyNode tick failed\n");

            joy_node.Dispose();

            return 3;
        }


        // 1 ms loop.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }


    // -------------------------------------------------------------------------
    // Clean shutdown
    // -------------------------------------------------------------------------

    std::cout
        << "Stopping JoyNode..."
        << std::endl;


    joy_node.Dispose();


    std::cout
        << "JoyNode stopped"
        << std::endl;


    return 0;
}