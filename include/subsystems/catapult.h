#ifndef __SUBSYSTEMS_CATAPULT_H__
#define __SUBSYSTEMS_CATAPULT_H__

#include "comets/vendor.h"
#include <memory>

/**
 * This code just implements an two position arm, but the real mechanism has
 * rubber bands pulling the arm to the zero position.
 */
class Catapult
{
public:
    Catapult();

    bool is_motor_idle() noexcept;
    void wind_back(bool auton = false);
    void wind_back_partly(bool auton = false);
    void fire();
    void fire_and_wind();
    void fire_and_wind_partly();
    void stop();

    void zero_position();
    double get_position();

    void periodic(bool auton = false);

    inline okapi::AbstractMotor &get_leftMotor() noexcept
    {
        return m_leftMotor;
    }

    inline okapi::AbstractMotor &get_rightMotor() noexcept
    {
        return m_rightMotor;
    }

private:
    okapi::Motor m_leftMotor;
    okapi::Motor m_rightMotor;
    std::pair<double, int16_t> targetPositionVelocity;
    bool movingToPosition = false;
    bool fireAndWind = false;
    bool fireAndWindPartly = false;

    void set_position(double position);
};

#endif