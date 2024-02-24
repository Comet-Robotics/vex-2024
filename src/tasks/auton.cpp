#include "subsystems.h"
#include "comets/vendor.h"
#include "tasks/auton.h"

#include "comets/logger.h"

using namespace okapi;
using namespace okapi::literals;

inline constexpr auto MAIN_LOOP_TICK_TIME = 10_ms;
inline constexpr auto FEEDING_HOLD_DURATION = 200_ms;
inline constexpr auto FIRING_HOLD_DURATION = 200_ms;
inline constexpr auto OUTTAKE_DURATION = 300_ms;
inline constexpr auto FIRST_SHOT_TIME = 500_ms;

inline constexpr auto NUM_CYCLES = 5;

inline constexpr auto SKILLS = true;

enum class SkillsState
{
    STARTTO_FEEDING_MOVE1,
    STARTTO_FEEDING_TURN1,
    SHOOT,
    GOTO_FEEDING,
    CURR_FEEDING,
    GOTO_FIRING,
    CURR_FIRING,
};

enum class RegularState
{
    STARTTO_FEEDING_MOVE1,
    SHOOT,
    STARTTO_FEEDING_TURN1,
    GOTO_FEEDING_MOVE1,
    CURR_FEEDING,
    GOTO_FIRING_MOVE1,
    GOTO_FIRING_TURN1,
    CURR_FIRING,
    GOTO_FEEDING_TURN1,
    IDLE,
};

/*
enum class RegularState
{
    STARTTO_ALLIANCE_MOVE1,
    STARTTO_ALLIANCE_TURN1,
    STARTTO_ALLIANCE_MOVE2,
    FEEDING_ALLIANCE,
    STARTTO_PUSHING_MOVE1,
    STARTTO_PUSHING_TURN1,
    STARTTO_PUSHING_MOVE2,
    STARTTO_PUSHING_TURN2,
    OUTTAKE_ALLIANCE,
    ALLIANCE_SCORE_FORWARD,
    STARTTO_PUSHING_PUSH,
    PUSHTO_FEEDING_MOVE1,
    PUSHTO_FEEDING_TURN1,
    PUSHTO_FEEDING_MOVE2,
    PUSHTO_FEEDING_TURN2,
    PUSHTO_FEEDING_MOVE3,
    CURR_FEEDING,
    GOTO_FIRING,
    CURR_FIRING,
    GOTO_FEEDING,
    GOTO_BAR,
    IDLE,
};
*/

// constexpr std::string_view stateToString(RegularState state);

void autonomous_initialize()
{
    // These need validation
    /*
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {5_ft, 0_ft, 0_deg}}, "startto_feed");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 2_ft, 120_deg}}, "goto_fire");
    drivebase->generatePath({{2_ft, 0_ft, 0_deg}, {0_ft, 4.5_ft, 120_deg}}, "goto_feed");
    */

    // skills
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 0_ft, 0_deg}}, "startto_feed");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 0_ft, 0_deg}}, "goto_fire");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4.5_ft, 0_ft, 0_deg}}, "goto_feed");

    // regular (FULL)
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 2_ft, 120_deg}}, "goto_fire_regular");
    drivebase->generatePath({{2_ft, 0_ft, 0_deg}, {0_ft, 4.5_ft, 120_deg}}, "goto_feed_regular");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {18_in, 0_ft, 0_deg}}, "startto_alliance_move1"); // reversed
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {24_in, 0_ft, 0_deg}}, "startto_alliance_move2");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {32_in, 0_ft, 0_deg}}, "startto_pushing_move1"); // reversed
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {20_in, 0_ft, 0_deg}}, "startto_pushing_move2"); // reversed
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "alliance_score_forward");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {38_in, 0_ft, 0_deg}}, "startto_pushing_push"); // reversed
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {16_in, 0_ft, 0_deg}}, "pushto_feeding_move1");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {42_in, 0_ft, 0_deg}}, "pushto_feeding_move2");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {16_in, 0_ft, 0_deg}}, "pushto_feeding_move3");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {24_in, 24_in, 45_deg}}, "goto_bar");

    // regular (HOUSTON)
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "startto_feeding_move1");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 0_ft, 0_deg}}, "goto_feeding_move1");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 0_ft, 0_deg}}, "goto_firing_move1");
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
    if (SKILLS)
    {
        autonomousSkills();
    }
    else
    {
        autonomousRegular();
    }
}

void autonomousRegular()
{
    auto state = RegularState::STARTTO_FEEDING_MOVE1;
    bool first_tick_in_state = true;

    catapult->zero_position();

    Timer timer;
    timer.placeMark();

    intake->forward();

    int currentCycles = 0;

    const auto onFirstTick = [&](auto callable)
    {
        if (first_tick_in_state)
        {
            callable();
            first_tick_in_state = false;
        }
    };

    const auto changeState = [&](RegularState v)
    {
        timer.placeMark();
        state = v;
        first_tick_in_state = true;
    };

    const auto changeStateAfter = [&](RegularState v, QTime duration)
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
        // COMET_LOG("%0.2f ms since mark", timer.getDtFromMark().convert(okapi::millisecond));
        // COMET_LOG("State: %s", stateToString(state).data());

        switch (state)
        {
        case RegularState::STARTTO_FEEDING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_feeding_move1", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::SHOOT);
            }
            break;
        }

        case RegularState::SHOOT:
        {
            onFirstTick([&]
                        { catapult->fire(); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_FEEDING_TURN1);
            }
            break;
        }

        case RegularState::STARTTO_FEEDING_TURN1:
        {
            onFirstTick([&]
                        { drivebase->turnAngle(-45_deg); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::GOTO_FEEDING_MOVE1);
            }
            break;
        }

        case RegularState::GOTO_FEEDING_MOVE1:
        {
            onFirstTick([&]
                        { 
                            drivebase->setTarget("goto_feeding_move1");
                            catapult->wind_back(); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::CURR_FEEDING);
            }
            break;
        }

        case RegularState::CURR_FEEDING:
        {
            if (timer.getDtFromMark() >= FEEDING_HOLD_DURATION)
            {
                changeState(RegularState::GOTO_FIRING_MOVE1);
            }
            break;
        }

        case RegularState::GOTO_FIRING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_firing_move1", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::GOTO_FIRING_TURN1);
            }
            break;
        }

        case RegularState::GOTO_FIRING_TURN1:
        {
            onFirstTick([&]
                        { drivebase->turnAngle(60_deg); });

            if (drivebase->isSettled())
            {
                changeState(RegularState::CURR_FIRING);
            }
            break;
        }

        case RegularState::CURR_FIRING:
        {
            onFirstTick([&]
                        {
                            catapult->fire();
                            currentCycles++; });
            if (timer.getDtFromMark() >= FIRING_HOLD_DURATION)
            {
                if (currentCycles >= NUM_CYCLES)
                {
                    catapult->wind_back();
                    changeState(RegularState::IDLE);
                }
                else
                {
                    changeState(RegularState::GOTO_FEEDING_TURN1);
                }
            }
            break;
        }

        case RegularState::GOTO_FEEDING_TURN1:
        {
            onFirstTick([&]
                        { drivebase->turnAngle(-80_deg); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::GOTO_FEEDING_MOVE1);
            }
            break;
        }

        case RegularState::IDLE:
        {
            break;
        }
        };

        /*
        switch (state)
        {
        case RegularState::STARTTO_ALLIANCE_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_alliance_move1"); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_ALLIANCE_TURN1);
            }
            break;
        }
        case RegularState::STARTTO_ALLIANCE_TURN1:
        {
            onFirstTick([&]
                        { catapult->fire_and_wind_partly(); });
            drivebase->turnAngle(-45_deg);
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_MOVE2);
            }
            break;
        }
        case RegularState::STARTTO_ALLIANCE_MOVE2:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_alliance_move2");
                        intake->forward(); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::FEEDING_ALLIANCE);
            }
            break;
        }
        case RegularState::FEEDING_ALLIANCE:
        {
            if (timer.getDtFromMark() > FEEDING_HOLD_DURATION)
            {
                changeState(RegularState::STARTTO_PUSHING_MOVE1);
            }
            break;
        }
        case RegularState::STARTTO_PUSHING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_pushing_move1", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_TURN1);
            }
            break;
        }

        case RegularState::STARTTO_PUSHING_TURN1:
        {
            drivebase->turnAngle(-45_deg);
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_MOVE2);
            }
            break;
        }

        case RegularState::STARTTO_PUSHING_MOVE2:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_pushing_move2", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_TURN2);
            }
            break;
        }

        case RegularState::STARTTO_PUSHING_TURN2:
        {
            drivebase->turnAngle(90_deg);
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_PUSH);
            }
            break;
        }

        case RegularState::OUTTAKE_ALLIANCE:
        {
            onFirstTick([&]
                        { intake->reverse(); });
            if (timer.getDtFromMark() > OUTTAKE_DURATION)
            {
                changeState(RegularState::ALLIANCE_SCORE_FORWARD);
            }
            break;
        }

        case RegularState::ALLIANCE_SCORE_FORWARD:
        {
            onFirstTick([&]
                        { drivebase->setTarget("alliance_score_forward", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_PUSH);
            }
            break;
        }

        case RegularState::STARTTO_PUSHING_PUSH:
        {
            onFirstTick([&]
                        {
                            drivebase->setTarget("startto_pushing_push", true);
                            catapult->fire_and_wind(); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::PUSHTO_FEEDING_MOVE1);
            }
            break;
        }

        case RegularState::PUSHTO_FEEDING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("pushto_feeding_move1"); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::PUSHTO_FEEDING_TURN1);
            }
            break;
        }

        case RegularState::PUSHTO_FEEDING_TURN1:
        {
            drivebase->turnAngle(-60_deg);
            if (drivebase->isSettled())
            {
                changeState(RegularState::PUSHTO_FEEDING_MOVE2);
            }
            break;
        }

        case RegularState::PUSHTO_FEEDING_MOVE2:
        {
            onFirstTick([&]
                        { drivebase->setTarget("pushto_feeding_move2"); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::PUSHTO_FEEDING_TURN2);
            }
            break;
        }

        case RegularState::PUSHTO_FEEDING_TURN2:
        {
            drivebase->turnAngle(15_deg);
            if (drivebase->isSettled())
            {
                changeState(RegularState::PUSHTO_FEEDING_MOVE3);
            }
            break;
        }

        case RegularState::PUSHTO_FEEDING_MOVE3:
        {
            onFirstTick([&]
                        {
                            drivebase->setTarget("pushto_feeding_move3");
                            intake->forward(); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::CURR_FEEDING);
            }
            break;
        }

        case RegularState::CURR_FEEDING:
        {
            if (timer.getDtFromMark() >= FEEDING_HOLD_DURATION)
            {
                changeState(RegularState::GOTO_FIRING);
            }
            break;
        }

        case RegularState::GOTO_FIRING:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_fire_regular", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::CURR_FIRING);
            }
            break;
        }

        case RegularState::CURR_FIRING:
        {
            onFirstTick([&]
                        {
                            catapult->fire_and_wind();
                            currentCycles++; });
            if (timer.getDtFromMark() >= FIRING_HOLD_DURATION)
            {
                if (currentCycles >= NUM_CYCLES)
                {
                    changeState(RegularState::GOTO_BAR);
                }
                else
                {
                    changeState(RegularState::GOTO_FEEDING);
                }
            }
            break;
        }

        case RegularState::GOTO_FEEDING:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_feed_regular"); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::CURR_FEEDING);
            }
            break;
        }

        case RegularState::GOTO_BAR:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_bar", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::IDLE);
            }
            break;
        }

        case RegularState::IDLE:
        {
            intake->stop();
            break;
        }
        }
    }
    */
    }
}

void autonomousSkills()
{
    auto state = SkillsState::STARTTO_FEEDING_MOVE1;
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

    const auto changeState = [&](SkillsState v)
    {
        timer.placeMark();
        state = v;
        first_tick_in_state = true;
    };

    const auto changeStateAfter = [&](SkillsState v, QTime duration)
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
        COMET_LOG("%0.2f ms since mark", timer.getDtFromMark().convert(okapi::millisecond));
        switch (state)
        {
        case SkillsState::STARTTO_FEEDING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_feed", true); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::STARTTO_FEEDING_TURN1);
            }
            break;
        }
        case SkillsState::STARTTO_FEEDING_TURN1:
        {
            onFirstTick([&]
                        { drivebase->turnAngle(-45_deg); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::SHOOT);
            }
            break;
        }
        case SkillsState::SHOOT:
        {
            onFirstTick([&]
                        { catapult->fire_and_wind(); });

            if (timer.getDtFromMark() >= FIRST_SHOT_TIME)
            {
                changeState(SkillsState::GOTO_FEEDING);
            }
            break;
        }

        case SkillsState::CURR_FEEDING:
        {
            if (timer.getDtFromMark() >= FEEDING_HOLD_DURATION)
            {
                changeState(SkillsState::GOTO_FIRING);
            }
            break;
        }
        case SkillsState::CURR_FIRING:
        {
            onFirstTick([&]
                        { 
                            catapult->fire_and_wind(); 
                            intake->reverse(); });

            if (timer.getDtFromMark() >= FIRING_HOLD_DURATION)
            {
                changeState(SkillsState::GOTO_FEEDING);
            }
            break;
        }
        case SkillsState::GOTO_FEEDING:
        {
            onFirstTick([&]
                        { 
                            drivebase->setTarget("goto_feed");
                            intake->forward(); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::CURR_FEEDING);
            }
            break;
        }
        case SkillsState::GOTO_FIRING:
        {
            onFirstTick([&]

                        { drivebase->setTarget("goto_fire", true); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::CURR_FIRING);
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

/*
    constexpr std::string_view stateToString(RegularState state)
    {
        switch (state)
        {
        case RegularState::STARTTO_ALLIANCE_MOVE1:
            return "STARTTO_ALLIANCE_MOVE1";
        case RegularState::STARTTO_ALLIANCE_TURN1:
            return "STARTTO_ALLIANCE_TURN1";
        case RegularState::STARTTO_ALLIANCE_MOVE2:
            return "STARTTO_ALLIANCE_MOVE2";
        case RegularState::FEEDING_ALLIANCE:
            return "FEEDING_ALLIANCE";
        case RegularState::STARTTO_PUSHING_MOVE1:
            return "STARTTO_PUSHING_MOVE1";
        case RegularState::STARTTO_PUSHING_TURN1:
            return "STARTTO_PUSHING_TURN1";
        case RegularState::STARTTO_PUSHING_MOVE2:
            return "STARTTO_PUSHING_MOVE2";
        case RegularState::STARTTO_PUSHING_TURN2:
            return "STARTTO_PUSHING_TURN2";
        case RegularState::OUTTAKE_ALLIANCE:
            return "OUTTAKE_ALLIANCE";
        case RegularState::ALLIANCE_SCORE_FORWARD:
            return "ALLIANCE_SCORE_FORWARD";
        case RegularState::STARTTO_PUSHING_PUSH:
            return "STARTTO_PUSHING_PUSH";
        case RegularState::PUSHTO_FEEDING_MOVE1:
            return "PUSHTO_FEEDING_MOVE1";
        case RegularState::PUSHTO_FEEDING_TURN1:
            return "PUSHTO_FEEDING_TURN1";
        case RegularState::PUSHTO_FEEDING_MOVE2:
            return "PUSHTO_FEEDING_MOVE2";
        case RegularState::PUSHTO_FEEDING_TURN2:
            return "PUSHTO_FEEDING_TURN2";
        case RegularState::PUSHTO_FEEDING_MOVE3:
            return "PUSHTO_FEEDING_MOVE3";
        case RegularState::CURR_FEEDING:
            return "CURR_FEEDING";
        case RegularState::GOTO_FIRING:
            return "GOTO_FIRING";
        case RegularState::CURR_FIRING:
            return "CURR_FIRING";
        case RegularState::GOTO_FEEDING:
            return "GOTO_FEEDING";
        case RegularState::GOTO_BAR:
            return "GOTO_BAR";
        case RegularState::IDLE:
            return "IDLE";
        }
        return "Unknown State";
    }
    */