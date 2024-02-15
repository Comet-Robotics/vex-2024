#include "subsystems/catapult.h"

#include <cassert>
#include "constants.h"
#include "comets/math.h"
#include "comets/logger.h"

static inline constexpr double IDLE_VELOCITY_ERROR_RANGE = 10.0;
static inline constexpr double IDLE_POSITION_ERROR_RANGE = 10.0;

namespace arm = constants::catapult;

static double get_next_nearest_position(double curr, double target);
static double remap_360_to_ps180(double curr);

Catapult::Catapult() : m_leftMotor(arm::LEFT_PORT), m_rightMotor(arm::RIGHT_PORT), targetPositionVelocity({0, 0}), movingToPosition(false)
{
    // m_motor.setPosPID(arm::POS_PIDF.F, arm::POS_PIDF.P, arm::POS_PIDF.I, arm::POS_PIDF.D);
    // m_motor.setVelPID(arm::VEL_PIDF.F, arm::VEL_PIDF.P, arm::VEL_PIDF.I, arm::VEL_PIDF.D);
    m_leftMotor.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    m_leftMotor.setGearing(arm::MOTOR_GEARSET);
    m_rightMotor.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    m_rightMotor.setGearing(arm::MOTOR_GEARSET);

    zero_position();
}

void Catapult::zero_position()
{
    m_leftMotor.tarePosition();
    m_rightMotor.tarePosition();
}

double Catapult::get_position()
{
    return m_leftMotor.getPosition(); // / static_cast<double>(constants::catapult::MOTOR_GEARSET);
}

void Catapult::wind_back(bool auton)
{
    if (fireAndWind)
        return;
    const auto curr_pos = get_position();
    if (comets::in_range((fmod(curr_pos, 360)), -arm::TOLERANCE, arm::TOLERANCE))
    {
        COMET_LOG("catapult at zero");
        return;
    }

    double nearestPosition = get_next_nearest_position(curr_pos, 0);
    COMET_LOG("pos curr %f ; near %f", curr_pos, nearestPosition);
    int offset = auton ? 5 : 2;
    targetPositionVelocity = {nearestPosition + offset, 50};
    movingToPosition = true;
    // std::printf("done winding.\n");
}

void Catapult::fire()
{
    if (fireAndWind)
        return;
    m_leftMotor.moveVelocity(80);
    m_rightMotor.moveVelocity(80);
    movingToPosition = true;
    targetPositionVelocity = {get_next_nearest_position(get_position(), 80), 80};
}

void Catapult::fire_and_wind()
{
    fire();
    fireAndWind = true;
}

void Catapult::stop()
{
    m_leftMotor.moveVelocity(0);
    m_rightMotor.moveVelocity(0);
}

bool Catapult::is_motor_idle() noexcept
{
    const double v_error = m_leftMotor.getVelocityError();
    const double p_error = m_leftMotor.getPositionError();
    static constexpr auto in_range = [](double target, double range)
    {
        return comets::in_range(target, -range / 2, range / 2);
    };
    return in_range(v_error, IDLE_VELOCITY_ERROR_RANGE) &&
           in_range(p_error, IDLE_POSITION_ERROR_RANGE);
}

void Catapult::set_position(double position)
{
    m_leftMotor.moveAbsolute(position, 400);
    m_rightMotor.moveAbsolute(position, 400);
}

void Catapult::periodic(bool auton)
{
    if (movingToPosition)
    {
        if (targetPositionVelocity.first > get_position())
        {
            m_leftMotor.moveVelocity(targetPositionVelocity.second);
            m_rightMotor.moveVelocity(targetPositionVelocity.second);
        }
        else
        {
            stop();
            movingToPosition = false;
            COMET_LOG("done moving to position %f", targetPositionVelocity.first);

            if (fireAndWind)
            {
                fireAndWind = false;
                wind_back(auton);
            }
        }
    }
}

static double get_next_nearest_position(double curr, double target)
{
    const double remainder = fmod((360.0 + target) - fmod(curr, 360.0), 360.0);
    const double new_target = curr + remainder;
    if (new_target < curr)
        COMET_LOG("new_target (%f) < curr (%f); assertion error:", new_target, curr);
    assert(new_target >= curr);
    return new_target;
}

static double remap_360_to_ps180(double curr)
{
    return curr - 180.0;
}