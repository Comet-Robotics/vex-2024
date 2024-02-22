#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>
#include "comets/vendor.h"
#include "comets/paths.h"
#include "comets/types.h"

namespace constants
{
    using namespace okapi;
    inline constexpr bool USE_TANK = false;

    inline constexpr okapi::PathfinderLimits PATH_LIMITS = {
        2.0, // Maximum linear velocity of the Chassis in m/s
        4.0, // Maximum linear acceleration of the Chassis in m/s/s
        10.0 // Maximum linear jerk of the Chassis in m/s/s/s
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

        inline constexpr auto CHASSIS_DIMS = {4_in, 12.5_in};
        inline constexpr auto CHASSIS_INTERNAL_GEARSET = okapi::AbstractMotor::gearset::blue;
        inline constexpr auto CHASSIS_TPR = double(okapi::imev5BlueTPR) * 84.0 / 36.0;
    }

    namespace catapult
    {
        inline constexpr int8_t SWITCH_PORT = 1;
        inline constexpr int8_t PORT = 18;
        inline constexpr bool REVERSED = true;
        inline constexpr double TOLERANCE = 7;

        inline constexpr auto STORED_POSITION = 0.0;
        inline constexpr auto EXTENDED_POSITION = 365.0;
        inline constexpr auto MOTOR_GEARSET = okapi::AbstractMotor::gearset::red;
        inline constexpr auto MOTOR_GEARRATIO = 1.0;
    } // namespace catapult

    namespace intake
    {
        inline constexpr int8_t PORT = -10; // make negative for reverse
        inline constexpr auto MOTOR_GEARSET = okapi::AbstractMotor::gearset::green;
    }

    namespace wings
    {
        inline constexpr int8_t LEFT_PORT = -4;
        inline constexpr int8_t RIGHT_PORT = 5;
        inline constexpr auto MOTOR_GEARSET = okapi::AbstractMotor::gearset::red;
    }

    // Max velocity of auton, in RPM
    inline constexpr double TURN_VEL_MULT = 0.7;

    inline constexpr double TELEOP_POLL_TIME = 10.0; // ms
}
#endif