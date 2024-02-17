#ifndef __SUBSYSTEMS_DRIVEBASE_H__
#define __SUBSYSTEMS_DRIVEBASE_H__

#include "comets/vendor.h"
#include "constants.h"
#include <memory>

class Drivebase
{
public:
    Drivebase();

    void generatePath(std::initializer_list<okapi::PathfinderPoint> iwaypoints, const std::string &ipathId);
    void arcade(double iforwardSpeed, double iyaw, double ithreshold = 0);
    void tank(double left, double right, double threshold = 0);

    inline auto get_state() noexcept
    {
        return chassis->getState();
    }

    inline auto &get_profile_controller() noexcept
    {
        return profile_controller;
    }
    inline auto &get_chassis() noexcept
    {
        return chassis;
    }

    inline void setTarget(const std::string &target, bool backwards = false)
    {
        profile_controller->setTarget(target, backwards);
    }

    inline void waitUntilSettled()
    {
        profile_controller->waitUntilSettled();
    }

    [[nodiscard]] inline bool isSettled()
    {
        return profile_controller->isSettled();
    }

    inline void turnAngle(okapi::QAngle angle)
    {
        const double oldMaxVel = chassis->getMaxVelocity();
        chassis->setMaxVelocity(oldMaxVel * constants::TURN_VEL_MULT);
        chassis->turnAngle(angle);
        chassis->setMaxVelocity(oldMaxVel);
    }

    inline bool turnAngleAsync(okapi::QAngle angle)
    {
        const double oldMaxVel = chassis->getMaxVelocity();
        chassis->setMaxVelocity(oldMaxVel * constants::TURN_VEL_MULT);
        chassis->turnAngleAsync(angle);
        if (chassis->isSettled())
        {
            chassis->setMaxVelocity(oldMaxVel);
            return true;
        }

        return false;
    }

    inline void moveDistance(okapi::QLength length)
    {
        chassis->moveDistance(length);
    }

    inline void driveToPoint(const okapi::Point &ipoint, bool ibackwards = false, const okapi::QLength &ioffset = okapi::QLength(0.0))
    {
        chassis->driveToPoint(ipoint, ibackwards, ioffset);
    }

private:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    std::shared_ptr<okapi::AsyncMotionProfileController> profile_controller;
};

#endif
