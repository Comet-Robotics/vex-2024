#include "subsystems.h"
#include "comets/vendor.h"
#include "tasks/auton.h"

#include "comets/logger.h"

using namespace okapi;
using namespace okapi::literals;

inline constexpr auto MAIN_LOOP_TICK_TIME = 10_ms;
inline constexpr auto FEEDING_HOLD_DURATION = 1000_ms;
inline constexpr auto FIRING_HOLD_DURATION = 300_ms;

enum class AutonState
{
    GOTO_FEEDING = -1,
    CURR_FEEDING = 0, // This should be the initial condition
    GOTO_FIRING = 1,
    CURR_FIRING = 2,
};

void autonomous_initialize()
{
    // These need validation
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 2_ft, 90_deg}}, "goto_fire");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {2_ft, 5_ft, 90_deg}}, "goto_feed");
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
    bool first_tick_in_state = true;

    catapult->zero_position();

    Timer timer;
    intake->forward();
    timer.placeMark();

    const auto onFirstTick = [&](auto callable)
    {
        if (first_tick_in_state)
        {
            callable();
            first_tick_in_state = false;
        }
    };

    const auto changeState = [&](AutonState v)
    {
        timer.placeMark();
        state = v;
        first_tick_in_state = true;
    };

    const auto changeStateAfter = [&](AutonState v, QTime duration)
    {
        if (timer.getDtFromMark() >= duration)
        {
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
            if (timer.getDtFromMark() >= FEEDING_HOLD_DURATION)
            {
                changeState(AutonState::GOTO_FIRING);
            }
            break;
        }
        case AutonState::CURR_FIRING:
        {
            onFirstTick([&]
                        { catapult->fire_and_wind(); });
            if (timer.getDtFromMark() >= FIRING_HOLD_DURATION)
            {
                changeState(AutonState::GOTO_FEEDING);
            }
            break;
        }
        case AutonState::GOTO_FEEDING:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_feed"); });
            if (drivebase->isSettled())
            {
                changeState(AutonState::CURR_FEEDING);
            }
            break;
        }
        case AutonState::GOTO_FIRING:
        {
            onFirstTick([&]

                        { drivebase->setTarget("goto_fire", true); });
            if (drivebase->isSettled())
            {
                changeState(AutonState::CURR_FIRING);
            }
            break;
        }
        }

        const auto delay = (MAIN_LOOP_TICK_TIME).convert(okapi::millisecond);
        if (delay > 0)
        {
            pros::delay(static_cast<uint32_t>(delay));
        }
    }
}
