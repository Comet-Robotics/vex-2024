#include "subsystems.h"
#include "comets/vendor.h"
#include "tasks/auton.h"

#include "comets/logger.h"

using namespace okapi;

static constexpr auto FEEDING_HOLD_DURATION = 300_ms;
static constexpr auto FIRING_HOLD_DURATION = 300_ms;

enum class AutonState
{
    GOTO_FEEDING = -1,
    CURR_FEEDING = 0, // This should be the initial condition
    GOTO_FIRING = 1,
    CURR_FIRING = 2,
};

void autonomous_initialize()
{
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    auto state = AutonState::CURR_FEEDING;
    bool firstChange = false;

    Timer timer;
    intake->forward();
    timer.placeMark();

    const auto changeStateAfter = [&](AutonState v, QTime duration)
    {
        if (timer.getDtFromMark() >= duration)
        {
            timer.placeMark();
            state = v;
            return true;
        }
        return false;
    };

    while (true)
    {
        catapult->periodic();
        COMET_LOG("%d", int(state));
        COMET_LOG("%0.2f ms since mark", timer.getDtFromMark().convert(okapi::millisecond));
        switch (state)
        {
        case AutonState::CURR_FEEDING:
        {
            changeStateAfter(AutonState::GOTO_FIRING, FEEDING_HOLD_DURATION);
            break;
        }
        case AutonState::CURR_FIRING:
        {
            catapult->fire_and_wind();
            changeStateAfter(AutonState::GOTO_FEEDING, FIRING_HOLD_DURATION);
            break;
        }
        case AutonState::GOTO_FEEDING:
        {
            catapult->wind_back();
            changeStateAfter(AutonState::CURR_FEEDING, 500_ms);
            break;
        }
        case AutonState::GOTO_FIRING:
        {
            changeStateAfter(AutonState::CURR_FIRING, 500_ms);
            break;
        }
        }
    }
}
