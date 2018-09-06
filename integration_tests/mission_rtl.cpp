#include <iostream>
#include <functional>
#include <memory>
#include <future>
#include <atomic>
#include <cmath>
#include "integration_test_helper.h"
#include "path_checker.h"
#include "dronecode_sdk.h"
#include "plugins/telemetry/telemetry.h"
#include "plugins/action/action.h"
#include "plugins/mission/mission.h"

using namespace dronecode_sdk;
using namespace std::placeholders; // for `_1`

// TODO: add checks that verify the return altitude

void do_mission_with_rtl(float mission_altitude_m, float rtl_altitude_m);

TEST_F(SitlTest, MissionWithRTLHigh)
{
    do_mission_with_rtl(20, 30);
}

TEST_F(SitlTest, MissionWithRTLLow)
{
    do_mission_with_rtl(5, 10);
}

TEST_F(SitlTest, MissionWithRTLHigherAnyway)
{
    do_mission_with_rtl(10, 5);
}

void do_mission_with_rtl(float mission_altitude_m, float return_altitude_m)
{
    DronecodeSDK dc;

    {
        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();

        LogInfo() << "Waiting to discover system...";
        dc.register_on_discover([prom](uint64_t uuid) {
            LogInfo() << "Discovered system with UUID: " << uuid;
            prom->set_value();
        });

        ConnectionResult ret = dc.add_udp_connection();
        ASSERT_EQ(ret, ConnectionResult::SUCCESS);

        auto status = future_result.wait_for(std::chrono::seconds(2));
        ASSERT_EQ(status, std::future_status::ready);
        future_result.get();
        // FIXME: This hack is to prevent that the promise is set twice.
        dc.register_on_discover(nullptr);
    }

    System &system = dc.system();
    auto telemetry = std::make_shared<Telemetry>(system);
    auto mission = std::make_shared<Mission>(system);
    auto action = std::make_shared<Action>(system);

    PathChecker pc;

    telemetry->position_async([&pc](Telemetry::Position position) {
        pc.check_current_alitude(position.relative_altitude_m);
    });

    while (!telemetry->health_all_ok()) {
        LogInfo() << "Waiting for system to be ready";
        LogDebug() << "Health: " << telemetry->health();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LogInfo() << "Setting RTL return altitude to " << return_altitude_m;
    action->set_return_to_launch_return_altitude(return_altitude_m);

    LogInfo() << "System ready";
    LogInfo() << "Creating and uploading mission";

    std::vector<std::shared_ptr<MissionItem>> mission_items;

    auto first_item = std::make_shared<MissionItem>();
    first_item->set_position(47.39821545, 8.54562609);
    first_item->set_relative_altitude(5.0f);
    mission_items.push_back(first_item);

    auto second_item = std::make_shared<MissionItem>();
    second_item->set_position(47.39829534, 8.54488580);
    second_item->set_gimbal_pitch_and_yaw(-90.0, 0.0);
    mission_items.push_back(second_item);

    Mission::Result result = mission->upload_mission(mission_items);

    if (result != Mission::Result::SUCCESS) {
        std::cout << "Failed to upload mission: "
                  << Mission::result_str(result) << std::endl;
        return;
    }


    pc.set_min_altitude(0.0f);
    pc.set_next_reach_altitude(mission_altitude_m);
    pc.set_max_altitude(mission_altitude_m);

    LogInfo() << "Arming...";
    const Action::Result arm_result = action->arm();
    ASSERT_EQ(arm_result, Action::Result::SUCCESS);
    LogInfo() << "Armed.";

    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission->subscribe_progress([&mission](int current, int total) {
        LogInfo() << "Mission status update: " << current << " / " << total;
    });

    {
        LogInfo() << "Starting mission.";
        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();
        mission->start_mission_async([prom](Mission::Result result) {
            ASSERT_EQ(result, Mission::Result::SUCCESS);
            prom->set_value();
            LogInfo() << "Started mission.";
        });

        auto status = future_result.wait_for(std::chrono::seconds(2));
        ASSERT_EQ(status, std::future_status::ready);
        future_result.get();
    }

    while (!mission->mission_finished()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    {
        // If the mission is higher than the return altitude, we return
        // at the current mission altitude. If the return altitude is higher
        // e.g. to clear trees, we need to climb up.
        if (return_altitude_m > mission_altitude_m) {
            pc.set_max_altitude(return_altitude_m);
            pc.set_next_reach_altitude(return_altitude_m);
        } else {
            pc.set_max_altitude(mission_altitude_m);
            pc.set_next_reach_altitude(mission_altitude_m);
        }
        pc.set_min_altitude(0.0f);

        // We are done, and can do RTL to go home.
        LogInfo() << "Commanding RTL...";
        const Action::Result result = action->return_to_launch();
        EXPECT_EQ(result, Action::Result::SUCCESS);
        LogInfo() << "Commanded RTL.";
    }

    while (telemetry->armed()) {
        // Wait until we're done.
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    LogInfo() << "Disarmed, exiting.";
}
