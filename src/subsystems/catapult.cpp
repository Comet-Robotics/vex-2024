#include "subsystems/catapult.h"

#include <cassert>
#include "constants.h"
#include "comets/math.h"
#include "comets/logger.h"

#include "pros/api_legacy.h"
#include "pros/adi.h"

static inline constexpr double IDLE_VELOCITY_ERROR_RANGE = 10.0;
static inline constexpr double IDLE_POSITION_ERROR_RANGE = 10.0;

namespace catapult = constants::catapult;

static bool read_limit_switch() noexcept;

Catapult::Catapult() : m_motor(catapult::PORT), targetPositionVelocity({0, 0}), moving_to_ready(false)
{
    pros::c::adi_pin_mode(catapult::SWITCH_PORT, INPUT);
    m_motor.setReversed(catapult::REVERSED);
    m_motor.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    m_motor.setGearing(catapult::MOTOR_GEARSET);
}

void Catapult::fire()
{
    if (!read_limit_switch())
        return;
    m_motor.moveVelocity(80);
    moving_to_ready = true;
}

void Catapult::stop()
{
    m_motor.moveVelocity(0);
}

void Catapult::set_position(double position)
{
    m_motor.moveAbsolute(position, 400);
}

void Catapult::periodic()
{
    const bool ready = read_limit_switch();
    if (moving_to_ready && ready)
    {
        if (m_timer.getDtFromMark().convert(okapi::millisecond) > 100)
        {
            m_motor.moveVelocity(0);
            moving_to_ready = false;
        }
    }
    else if (moving_to_ready && !ready)
    {
        if (m_timer.getDtFromMark().convert(okapi::millisecond) > 100)
        {
            m_motor.moveVelocity(100);
            moving_to_ready = true;
        }
        // movement in progress
    }
    else if (!moving_to_ready && ready)
    {
        m_timer.placeMark();
        m_motor.moveVelocity(0);
    }
    else if (!moving_to_ready && !ready)
    {
        m_timer.placeMark();
        m_motor.moveVelocity(100);
    }
}

static bool read_limit_switch() noexcept
{
    // explicity check if equal to one so any errors get resolved as false
    return pros::c::adi_digital_read(catapult::SWITCH_PORT) == 1;
}
