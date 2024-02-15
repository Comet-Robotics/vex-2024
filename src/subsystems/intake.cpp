#include "subsystems/intake.h"
#include "comets/math.h"

using namespace constants::intake;

static constexpr auto SPEED = SPEED_MULTIPLIER * static_cast<int>(MOTOR_GEARSET);

Intake::Intake() : m_motors({LEFT_PORT, RIGHT_PORT})
{
    static_assert(comets::signum(LEFT_PORT) != comets::signum(RIGHT_PORT),
                  "Directions of motors must be opposite");
    m_motors.setGearing(MOTOR_GEARSET);
}

void Intake::forward() noexcept
{
    m_motors.moveVelocity(SPEED);
}

void Intake::reverse() noexcept
{
    m_motors.moveVelocity(-SPEED);
}

void Intake::stop() noexcept
{
    m_motors.moveVelocity(0);
}

bool Intake::is_running() const noexcept
{
    return m_motors.getTargetVelocity() == 0;
}
