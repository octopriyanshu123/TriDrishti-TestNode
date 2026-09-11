#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include "i2w/impl.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"

namespace {

constexpr const char* kLogPrefix =
    "[node=UiRobotConnectionCheckClient][kind=service][role=client]";
constexpr const char* kDefaultServiceName = "/robot_ui_sync/ui_robot_connection_check";
constexpr const char* kNetworkProfileEnv = "UI_ROBOT_CONNECTION_CHECK_ECAL_CONFIG";

volatile std::sig_atomic_t running = 1;

void Stop(int)
{
    running = 0;
}

std::string resolveNetworkProfilePath()
{
    if (const char* env_path = std::getenv(kNetworkProfileEnv);
        env_path != nullptr && *env_path != '\0')
    {
        return env_path;
    }

    const std::filesystem::path cwd_profile{"config/ecal-network-udp.yaml"};
    if (std::filesystem::exists(cwd_profile))
    {
        return cwd_profile.string();
    }

#ifdef TRIDRISHTI_TESTNODE_SOURCE_DIR
    const auto source_profile =
        std::filesystem::path{TRIDRISHTI_TESTNODE_SOURCE_DIR} / "config/ecal-network-udp.yaml";
    if (std::filesystem::exists(source_profile))
    {
        return source_profile.string();
    }
#endif

    return cwd_profile.string();
}

const char* planeName(i2w::EndpointPlane plane) noexcept
{
    return plane == i2w::EndpointPlane::Network ? "network" : "local";
}

} // namespace

class UiRobotConnectionCheckClient final : public i2w::SystemBase
{
public:
    explicit UiRobotConnectionCheckClient(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:

    i2w::LifecycleResult OnSetup() override
    {
        i2w::ServiceOptions options;

        options.plane = plane_;
        options.max_outstanding_calls = 1;
        options.call_timeout_ms = 500;

        auto client = runtime().create_client<crawler_i2w_services::UiRobotConnectionCheckRequest,crawler_i2w_services::UiRobotConnectionCheckReponse>(service_name_,options);

        if (!client)
        {
            std::printf(
                "%s[plane=%s][name=%s] create_client_failed error=%s\n",
                kLogPrefix,
                planeName(plane_),
                service_name_.c_str(),
                i2w::to_string(client.error())
            );

            return i2w::Fail();
        }

        client_ = std::move(client.value());
        next_call_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);

        std::printf(
            "%s[plane=%s][name=%s] client_ready\n",
            kLogPrefix,
            planeName(plane_),
            service_name_.c_str()
        );

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() override
    {
        const auto now = std::chrono::steady_clock::now();

        // If we're waiting on a response and it's taken too long, mark the UI as not live.
        if (waiting_for_response_ && now >= response_deadline_)
        {
            waiting_for_response_ = false;
            is_ui_live_ = false;
            std::cout << kLogPrefix
                      << "[plane=" << planeName(plane_)
                      << "][name=" << service_name_
                      << "] response_timeout ui_live=false"
                      << std::endl;
        }

        // Rate-limit outgoing calls to once per second.
        if (!waiting_for_response_ && now >= next_call_)
        {
            crawler_i2w_services::UiRobotConnectionCheckRequest request;

            request.ping = 1;
            request.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);

            const auto result = client_.call(request, runtime().clock().now().ns,
                [this](const i2w::Sample<crawler_i2w_services::UiRobotConnectionCheckReponse>& sample)
                {
                    is_ui_live_ = true;
                    waiting_for_response_ = false;

                    std::cout << kLogPrefix
                              << "[plane=" << planeName(plane_)
                              << "][name=" << service_name_
                              << "] response_received pong="
                              << static_cast<int>(sample.value.pong)
                              << " ui_live=true"
                              << std::endl;
                });

            if (!result)
            {
                is_ui_live_ = false;
                next_call_ = now + std::chrono::milliseconds(500);
                std::cout << kLogPrefix
                          << "[plane=" << planeName(plane_)
                          << "][name=" << service_name_
                          << "] request_submit_failed error="
                          << i2w::to_string(result.error())
                          << " ui_live=false"
                          << std::endl;
                return i2w::Ok();
            }

            waiting_for_response_ = true;
            response_deadline_ = now + std::chrono::milliseconds(500);
            next_call_ = now + std::chrono::milliseconds(500);

            std::cout << kLogPrefix
                      << "[plane=" << planeName(plane_)
                      << "][name=" << service_name_
                      << "] request_sent ping="
                      << static_cast<int>(request.ping)
                      << " timestamp="
                      << request.timestamp
                      << std::endl;
        }

        return i2w::Ok();
    }

private:

    std::string service_name_{kDefaultServiceName};

    i2w::EndpointPlane plane_{i2w::EndpointPlane::Network};

    i2w::Client<crawler_i2w_services::UiRobotConnectionCheckRequest,crawler_i2w_services::UiRobotConnectionCheckReponse> client_{};
    bool waiting_for_response_{false};
    bool is_ui_live_{false};
    std::chrono::steady_clock::time_point next_call_{};
    std::chrono::steady_clock::time_point response_deadline_{};
};

int main()
{
    i2w::Config config;

    config.node_name = "UiRobotConnectionCheckClient";
    config.ns = "";
    config.transport.network_profile_file = resolveNetworkProfilePath();

    std::cout << kLogPrefix
              << "[plane=network][name=" << kDefaultServiceName
              << "] using_network_profile="
              << config.transport.network_profile_file
              << std::endl;

    UiRobotConnectionCheckClient node(config);

    if (!node.Setup().ok)
    {
        std::cerr << "Setup failed\n";
        return 1;
    }

    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    while (running)
    {
        if (!node.Tick().ok)
        {
            std::cerr << "Tick failed\n";
            node.Dispose();
            return 2;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    node.Dispose();
    return 0;
}
