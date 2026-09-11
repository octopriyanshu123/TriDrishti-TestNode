#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <utility>

#include "i2w/impl.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"
#include <future>
class UiRobotConnectionCheckServer final : public i2w::SystemBase
{
public:
    explicit UiRobotConnectionCheckServer(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:
    i2w::LifecycleResult OnSetup() override
    {
        i2w::ServiceOptions options;

        options.plane = i2w::EndpointPlane::Network;
        options.max_outstanding_calls = 1;
        options.call_timeout_ms = 500;

        auto client = runtime().create_client<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse>(service_name_, options);

        if (!client)
        {
            std::printf(
                "create_client failed: %s\n",
                i2w::to_string(client.error()));

            return i2w::Fail();
        }

        client_ = std::move(client.value());

        // Server setup
        i2w::ServiceOptions server_options;
        server_options.plane = i2w::EndpointPlane::Local;
        server_options.max_outstanding_calls = 1;
        server_options.call_timeout_ms = 2000;

        // auto server = runtime().advertise_service<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse>(
        //     service_name_,
        //     [this](const crawler_i2w_services::UiRobotConnectionCheckRequest &request, const i2w::Header &header, crawler_i2w_services::UiRobotConnectionCheckReponse &response)
        //     {
        //         // std::cout << "[PING PING NTL] Request received Local : ping = "
        //         //           << static_cast<int>(request.ping)
        //         //           << std::endl;

        //         std::cout << "[PING PING NTL] Request received Local : ping = "
        //                   << static_cast<int>(request.ping)
        //                   << " timestamp = "
        //                   << request.timestamp
        //                   << std::endl;
        //         client_.call(request, runtime().clock().now().ns,
        //             [this](const i2w::Sample<crawler_i2w_services::UiRobotConnectionCheckReponse> &sample)
        //             {

        //                 // response.pong = sample.value.pong;
        //                 // response.timestamp = request.timestamp;

        //                 std::cout << "[PING PING NTL] Response received Network : pong = "
        //                           << static_cast<int>(sample.value.pong)
        //                           << std::endl;
        //             });

        //         // response.pong = 1;
        //         // response.timestamp = request.timestamp;
        //     },
        //     server_options);

        auto server = runtime().advertise_service<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse>(
            service_name_,
            [this](const auto &request, const auto &header, auto &response)
            {
                std::cout << "[PING] Local ping = " << (int)request.ping << "\n";

                auto promise = std::make_shared<std::promise<crawler_i2w_services::UiRobotConnectionCheckReponse>>();
                auto future = promise->get_future();

                client_.call(request, runtime().clock().now().ns,
                             [promise](const auto &sample)
                             {
                                 promise->set_value(sample.value);
                             });

                // Block up to the network call's timeout waiting for the pong
                if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready)
                {
                    auto net_response = future.get();
                    response.pong = net_response.pong;
                    response.timestamp = request.timestamp;
                }
                else
                {
                    std::cerr << "[PING] Network call timed out\n";
                    // decide on a fallback pong value here
                }
            },
            server_options);
        if (!server)
        {
            std::printf(
                "create_client failed: %s\n",
                i2w::to_string(server.error()));

            return i2w::Fail();
        }

        server_ = std::move(server.value());

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() override
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        return i2w::Ok();
    }

private:
    std::string service_name_{"/ui_robot_connection_check"};

    i2w::Server<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse> server_{};
    i2w::Client<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse> client_{};

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
    config.transport.network_profile_file = "/home/tridrishti/tridrishti_ws/src/TriDrishti-TestNode/config/ecal-network-udp.yaml";

    UiRobotConnectionCheckServer node(config);

    if (!node.Setup().ok)
    {
        std::cerr << "Setup failed\n";
        return 1;
    }

    while (true)
    {
        node.Tick();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}