#include "subsystems/wings.h"
#include "constants.h"
#include "comets/math.h"

Wings::Wings() : m_left(constants::wings::LEFT_PORT),
                 m_right(constants::wings::RIGHT_PORT),
                 m_left_task(pros::Task::create(std::bind(Wings::task_handler, std::ref(m_left)))),
                 m_right_task(pros::Task::create(std::bind(Wings::task_handler, std::ref(m_right))))
{
    m_left.setGearing(constants::wings::MOTOR_GEARSET);
    m_left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
    m_right.setGearing(constants::wings::MOTOR_GEARSET);
    m_right.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
}

void Wings::toggle_left()
{
    m_left_task.notify();
}

void Wings::toggle_right()
{
    m_right_task.notify();
}

void Wings::toggle_motor(okapi::Motor &motor)
{
    const auto position = motor.getPosition();
    const auto velocity = motor.getActualVelocity();
    const bool is_stored = comets::in_range(position, -0.1, +0.1);

    if (is_stored)
    {
        motor.moveAbsolute(0.23, 100);
    }
    else
    {
        motor.moveAbsolute(-0.05, 100);
    }
}

void Wings::task_handler(okapi::Motor &motor)
{
    while (pros::Task::notify_take(true, TIMEOUT_MAX))
    {
        Wings::toggle_motor(motor);
    }
}
