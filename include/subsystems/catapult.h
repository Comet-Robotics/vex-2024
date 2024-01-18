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
    void wind_back();
    void fire();
    void stop();

    void zero_position();
    double get_position();

    void periodic();

    inline okapi::AbstractMotor &get_motor() noexcept
    {
        return m_motor;
    }

private:
    okapi::Motor m_motor;
    std::pair<double, int16_t> targetPositionVelocity;
    bool movingToPosition;

    void set_position(double position);
};

#endif
