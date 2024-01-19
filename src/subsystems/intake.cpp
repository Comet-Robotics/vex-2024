#include "subsystems/intake.h"
#include "comets/math.h"

using namespace constants::intake;

static constexpr auto SPEED = static_cast<int>(MOTOR_GEARSET);

Intake::Intake() : m_motors({PORT})
{
    // static_assert(comets::signum(LEFT_PORT) != comets::signum(RIGHT_PORT),
    //               "Directions of motors must be opposite");
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
