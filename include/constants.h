#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>
#include "comets/vendor.h"
#include "comets/paths.h"
#include "comets/types.h"

namespace constants
{
    using namespace okapi;
    inline constexpr bool USE_TANK = true;

    inline constexpr okapi::PathfinderLimits PATH_LIMITS = {
        1.0 * 0.66, // Maximum linear velocity of the Chassis in m/s
        2.0 * 0.66, // Maximum linear acceleration of the Chassis in m/s/s
        10.0 * 0.66 // Maximum linear jerk of the Chassis in m/s/s/s
    };

    inline constexpr comets::path_plan PATHS[] = {
        {.name = "right_turn", .points = {{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 90_deg}}},
        {.name = "straight", .points = {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}},
        {.name = "strafe_right", .points = {{0_ft, 0_ft, 0_deg}, {0_ft, 2_ft, 0_deg}}},
    };

    namespace drivebase
    {
        inline constexpr std::array<int8_t, 3> LEFT_PORTS = {
            2,
            -4,
            -6,
        };
        inline constexpr std::array<int8_t, 3> RIGHT_PORTS = {
            -1,
            3,
            5,
        };

        inline constexpr auto CHASSIS_GEARSET = okapi::AbstractMotor::gearset::red;
        inline constexpr auto CHASSIS_DIMS = {4_in, 12.5_in};
        inline constexpr auto CHASSIS_TPR = okapi::imev5GreenTPR;
    }

    namespace catapult
    {
        inline constexpr int8_t PORT = 9;
        inline constexpr bool REVERSED = false;
        inline constexpr double TOLERANCE = 7;
        inline constexpr auto POS_PIDF = comets::PIDF_Value{
            .P = 0.01,
            .I = 0.0,
            .D = 0.0,
            .F = 0.05};

        inline constexpr auto VEL_PIDF = comets::PIDF_Value{
            .P = 0.03,
            .I = 0.0,
            .D = 0.02,
            .F = 0.10};

        inline constexpr auto STORED_POSITION = 0.0;
        inline constexpr auto EXTENDED_POSITION = 350.0;
        inline constexpr auto MOTOR_GEARSET = okapi::AbstractMotor::gearset::green;

    } // namespace catapult
    namespace intake
    {
        inline constexpr int8_t LEFT_PORT = -19;
        inline constexpr int8_t RIGHT_PORT = 20;
        inline constexpr auto MOTOR_GEARSET = okapi::AbstractMotor::gearset::green;
    }

    // Max velocity of auton, in RPM
    inline constexpr double TURN_VEL_MULT = 0.3;

    inline constexpr double TELEOP_POLL_TIME = 10.0; // ms
}
#endif