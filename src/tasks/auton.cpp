#include "subsystems.h"
#include "comets/vendor.h"
#include "tasks/auton.h"

#include "comets/logger.h"

using namespace okapi;
using namespace okapi::literals;

inline constexpr auto MAIN_LOOP_TICK_TIME = 10_ms;
inline constexpr auto FEEDING_HOLD_DURATION = 200_ms;
inline constexpr auto FIRING_HOLD_DURATION = 200_ms;
inline constexpr auto SPIT_OUT_TIME = 300_ms;
inline constexpr auto FIRST_SKILLS_SHOT_TIME = 1000_ms;

inline constexpr auto REGULAR_WAIT_TIME = 15000_ms;

inline constexpr auto SKILLS_CYCLES = 2;

inline constexpr auto SKILLS = false;

enum class AutonMode
{
    REGULAR,
    TEST,
    SKILLS,
};

inline constexpr AutonMode MODE = AutonMode::SKILLS;

enum class SkillsState
{
    STARTTO_FEEDING_MOVE1,
    SHOOT,
    STARTTO_FEEDING_TURN1,
    STARTTO_FEEDING_MOVE2,
    GOTO_FEEDING,
    CURR_FEEDING,
    GOTO_FIRING,
    CURR_FIRING,
    GOTO_SIDE_MOVE1,
    GOTO_SIDE_TURN1,
    GOTO_SIDE_MOVE2,
    GOTO_SIDE_TURN2,
    GOTO_SIDE_MOVE3,
    GOTO_SIDE_TURN3,
    GOTO_SIDE_MOVE4,
    GOTO_SIDE_TURN4,
    GOTO_SIDE_MOVE5,
    GOTO_FRONT_MOVE1,
    GOTO_FRONT_TURN1,
    GOTO_FRONT_MOVE2,
    GOTO_FRONT_TURN2,
    GOTO_FRONT_MOVE3,
    GOTO_FRONT_TURN3,
    GOTO_FRONT_MOVE4,
    GOTO_FRONT_MOVE5,
    IDLE,
};

enum class RegularState
{
    MOVE_1,
    TURN_1,
    MOVE_2,
    TURN_2,
    MOVE_3,
    FORWARD,
    BACK,
    IDLE,
};

/*
enum class RegularState
{
    STARTTO_PUSHING_MOVE1,
    STARTTO_PUSHING_TURN1,
    STARTTO_PUSHING_MOVE2,
    STARTTO_PUSHING_TURN2,
    STARTTO_PUSHING_MOVE3,
    PUSHING_FORWARD,
    PUSHING_BACK,
    ALLIANCE_TRIBALL_MOVE1,
    ALLIANCE_TRIBALL_TURN1,
    ALLIANCE_TRIBALL_MOVE2,
    ALLIANCE_TRIBALL_TURN2,
    ALLIANCE_TRIBALL_MOVE3,
    WAIT_FOR_INTAKE,
    SCORE_ALLIANCE_MOVE1,
    SCORE_ALLIANCE_TURN1,
    SCORE_ALLIANCE_MOVE2,
    SCORE_ALLIANCE_TURN2,
    SPIT_OUT_TRIBALL,
    SCORE_ALLIANCE_MOVE3,
    SCORE_ALLIANCE_BACK,
    SCORE_ALLIANCE_FORWARD,
    GOTO_WAITING,
    TURN_AROUND,
    WAIT,
    SCORE_TRIBALLS,
    SCORE_TRIBALLS_FORWARD,
    SCORE_TRIBALLS_BACK,
    PARK,
    IDLE,
};
*/

constexpr std::string_view stateToString(RegularState state);

void autonomous_initialize()
{
    // These need validation
    /*
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {5_ft, 0_ft, 0_deg}}, "startto_feed");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 2_ft, 120_deg}}, "goto_fire");
    drivebase->generatePath({{2_ft, 0_ft, 0_deg}, {0_ft, 4.5_ft, 120_deg}}, "goto_feed");
    */

    const auto new_path = [&](okapi::PathfinderPoint target, const char *name)
    {
        target.x *= 100.0 / 44.0;
        target.y *= 100.0 / 44.0;
        drivebase->generatePath({{0_ft, 0_ft, 0_deg}, target}, name);
    };

    // skills
    new_path({24_in, 0_ft, 0_deg}, "startto_feeding_move1");
    new_path({24_in, 0_ft, 0_deg}, "startto_feeding_move2");
    new_path({12_in, 0_ft, 0_deg}, "goto_side_move1");
    new_path({24_in, 0_ft, 0_deg}, "goto_side_move2");
    new_path({86_in, 0_ft, 0_deg}, "goto_side_move3");
    new_path({24_in, 0_ft, 0_deg}, "goto_side_move4");
    new_path({16_in, 0_ft, 0_deg}, "goto_side_move5");
    new_path({12_in, 0_ft, 0_deg}, "goto_front_move1");
    new_path({48_in, 0_ft, 0_deg}, "goto_front_move2");
    new_path({30_in, 0_ft, 0_deg}, "goto_front_move3");
    new_path({36_in, 0_ft, 0_deg}, "goto_front_move4");
    new_path({12_in, 0_ft, 0_deg}, "goto_front_move5");

    new_path({36_in, 0_ft, 0_deg}, "goto_fire");
    new_path({42_in, 0_ft, 0_deg}, "goto_feed");

    // regular
    new_path({56_in, 0_ft, 0_deg}, "move_1");
    new_path({24_in, 0_ft, 0_deg}, "move_2");
    new_path({16_in, 0_ft, 0_deg}, "move_3");
    new_path({16_in, 0_ft, 0_deg}, "forward");
    new_path({16_in, 0_ft, 0_deg}, "back");

    new_path({60_in, 0_ft, 0_deg}, "startto_pushing_move1");
    new_path({30_in, 0_ft, 0_deg}, "startto_pushing_move2");
    new_path({8_in, 0_ft, 0_deg}, "startto_pushing_move3");
    new_path({12_in, 0_ft, 0_deg}, "pushing_forward");
    new_path({12_in, 0_ft, 0_deg}, "pushing_back");
    new_path({10_in, 0_ft, 0_deg}, "alliance_triball_move1");
    new_path({14_in, 0_ft, 0_deg}, "alliance_triball_move2");
    new_path({8_in, 0_ft, 0_deg}, "alliance_triball_move3");
    new_path({8_in, 0_ft, 0_deg}, "score_alliance_move1");
    new_path({14_in, 0_ft, 0_deg}, "score_alliance_move2");
    new_path({10_in, 0_ft, 0_deg}, "score_alliance_move3");
    new_path({12_in, 0_ft, 0_deg}, "score_alliance_back");
    new_path({12_in, 0_ft, 0_deg}, "score_alliance_forward");
    new_path({0_in, 0_ft, 90_deg}, "goto_waiting"); // spline
    new_path({16_in, 0_ft, 0_deg}, "score_triballs");
    new_path({12_in, 0_ft, 0_deg}, "score_triballs_forward");
    new_path({12_in, 0_ft, 0_deg}, "score_triballs_back");
    new_path({30_in, 60_in, 90_deg}, "park");

    // test
    new_path({136.36_in, 0_ft, 0_deg}, "forward_test");
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
    if (MODE == AutonMode::TEST)
    {
        autonomousTest();
    }
    else if (MODE == AutonMode::SKILLS)
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
    auto state = RegularState::MOVE_1;
    bool first_tick_in_state = true;

    catapult->zero_position();

    Timer timer;
    timer.placeMark();

    int currentCycles = 0;
    int forwardAndBack = 0;

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

    const auto followPath = [&](std::string path, RegularState next)
    {
        onFirstTick([&]
                    { drivebase->setTarget(path); });
        if (drivebase->isSettled())
        {
            changeState(next);
        }
    };

    const auto followPathReversed = [&](std::string path, RegularState next)
    {
        onFirstTick([&]
                    { drivebase->setTarget(path, true); });
        if (drivebase->isSettled())
        {
            changeState(next);
        }
    };

    const auto turn = [&](QAngle angle, RegularState next)
    {
        onFirstTick([&]
                    { drivebase->turnAngle(angle); });
        if (drivebase->isSettled())
        {
            changeState(next);
        }
    };

    while (true)
    {
        catapult->periodic(true);
        // COMET_LOG("%0.2f ms since mark", timer.getDtFromMark().convert(okapi::millisecond));
        COMET_LOG("State: %s", stateToString(state).data());

        switch (state)
        {
        case RegularState::MOVE_1:
        {
            followPathReversed("move_1", RegularState::TURN_1);
            break;
        }
        case RegularState::TURN_1:
        {
            turn(45_deg, RegularState::MOVE_2);
            break;
        }
        case RegularState::MOVE_2:
        {
            onFirstTick([&]
                        { drivebase->setTarget("move_2", true); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::TURN_2);
            }
            break;
        }
        case RegularState::TURN_2:
        {
            turn(45_deg, RegularState::MOVE_3);
            break;
        }
        case RegularState::MOVE_3:
        {
            followPathReversed("move_3", RegularState::FORWARD);
            break;
        }
        case RegularState::FORWARD:
        {
            followPath("forward", RegularState::BACK);
            break;
        }
        case RegularState::BACK:
        {
            onFirstTick([&]
                        { drivebase->setTarget("back", true);
                        forwardAndBack++; });
            if (drivebase->isSettled())
            {
                if (forwardAndBack >= 2)
                {
                    changeState(RegularState::IDLE);
                }
                else
                {
                    changeState(RegularState::FORWARD);
                }
            }
        }
        };

        /*
                switch (state)
                {
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
                    // wings->toggle_right();
                    turn(-45_deg, RegularState::STARTTO_PUSHING_MOVE2);
                    break;
                }
                case RegularState::STARTTO_PUSHING_MOVE2:
                {
                    onFirstTick([&]
                                {
                                    drivebase->setTarget("startto_pushing_move2", true);
                                    wings->toggle_right(); });
                    if (drivebase->isSettled())
                    {
                        changeState(RegularState::STARTTO_PUSHING_TURN2);
                    }
                    break;
                }
                case RegularState::STARTTO_PUSHING_TURN2:
                {
                    // wings->toggle_right();
                    turn(-45_deg, RegularState::STARTTO_PUSHING_MOVE3);
                    break;
                }
                case RegularState::STARTTO_PUSHING_MOVE3:
                {
                    followPathReversed("startto_pushing_move3", RegularState::PUSHING_FORWARD);
                    break;
                }
                case RegularState::PUSHING_FORWARD:
                {
                    drivebase->arcade(1, 0);
                    if (timer.getDtFromMark() > 500_ms)
                    {
                        changeState(RegularState::PUSHING_BACK);
                    }
                    break;
                }
                case RegularState::PUSHING_BACK:
                {
                    drivebase->arcade(-1, 0);
                    if (timer.getDtFromMark() > 1000_ms)
                    {
                        changeState(RegularState::PUSHING_BACK);
                    }
                    break;
                    break;
                }
                case RegularState::ALLIANCE_TRIBALL_MOVE1:
                {
                    onFirstTick([&]
                                {
                                    wings->toggle_right();
                                    // catapult->fire_and_wind_partly();
                                    drivebase->setTarget("alliance_triball_move1"); });
                    if (drivebase->isSettled())
                    {
                        changeState(RegularState::ALLIANCE_TRIBALL_TURN1);
                    }
                    break;
                }
                case RegularState::ALLIANCE_TRIBALL_TURN1:
                {
                    turn(45_deg, RegularState::ALLIANCE_TRIBALL_MOVE2);
                    break;
                }
                case RegularState::ALLIANCE_TRIBALL_MOVE2:
                {
                    followPath("alliance_triball_move2", RegularState::ALLIANCE_TRIBALL_TURN2);
                    break;
                }
                case RegularState::ALLIANCE_TRIBALL_TURN2:
                {
                    turn(-90_deg, RegularState::ALLIANCE_TRIBALL_MOVE3);
                    break;
                }
                case RegularState::ALLIANCE_TRIBALL_MOVE3:
                {
                    intake->forward();
                    followPath("alliance_triball_move3", RegularState::WAIT_FOR_INTAKE);
                    break;
                }
                case RegularState::WAIT_FOR_INTAKE:
                {
                    if (timer.getDtFromMark() > FEEDING_HOLD_DURATION)
                    {
                        changeState(RegularState::SCORE_ALLIANCE_MOVE1);
                    }
                    break;
                }
                case RegularState::SCORE_ALLIANCE_MOVE1:
                {
                    intake->stop();
                    followPathReversed("score_alliance_move1", RegularState::SCORE_ALLIANCE_TURN1);
                    break;
                }
                case RegularState::SCORE_ALLIANCE_TURN1:
                {
                    turn(-90_deg, RegularState::SCORE_ALLIANCE_MOVE2);
                    break;
                }
                case RegularState::SCORE_ALLIANCE_MOVE2:
                {
                    followPath("score_alliance_move2", RegularState::SCORE_ALLIANCE_TURN2);
                    break;
                }
                case RegularState::SCORE_ALLIANCE_TURN2:
                {
                    turn(-45_deg, RegularState::SPIT_OUT_TRIBALL);
                    break;
                }
                case RegularState::SPIT_OUT_TRIBALL:
                {
                    onFirstTick([&]
                                { intake->reverse(); });
                    if (timer.getDtFromMark() > SPIT_OUT_TIME)
                    {
                        changeState(RegularState::SCORE_ALLIANCE_MOVE3);
                    }
                    break;
                }
                case RegularState::SCORE_ALLIANCE_MOVE3:
                {
                    followPath("score_alliance_move3", RegularState::SCORE_ALLIANCE_BACK);
                    break;
                }
                case RegularState::SCORE_ALLIANCE_BACK:
                {
                    followPathReversed("score_alliance_back", RegularState::SCORE_ALLIANCE_FORWARD);
                    break;
                }
                case RegularState::SCORE_ALLIANCE_FORWARD:
                {
                    followPath("score_alliance_forward", RegularState::GOTO_WAITING);
                    break;
                }
                case RegularState::GOTO_WAITING:
                {
                    followPathReversed("goto_waiting", RegularState::TURN_AROUND);
                    break;
                }
                case RegularState::TURN_AROUND:
                {
                    turn(180_deg, RegularState::WAIT);
                    break;
                }
                case RegularState::WAIT:
                {
                    onFirstTick([&]
                                {
                                    catapult->wind_back();
                                    wings->toggle_right(); });
                    if (timer.getDtFromMark() > REGULAR_WAIT_TIME)
                    {
                        changeState(RegularState::SCORE_TRIBALLS);
                    }
                    break;
                }
                case RegularState::SCORE_TRIBALLS:
                {
                    followPathReversed("score_triballs", RegularState::SCORE_TRIBALLS_FORWARD);
                    break;
                }
                case RegularState::SCORE_TRIBALLS_FORWARD:
                {
                    followPath("score_triballs_forward", RegularState::SCORE_TRIBALLS_BACK);
                    break;
                }
                case RegularState::SCORE_TRIBALLS_BACK:
                {
                    followPathReversed("score_triballs_back", RegularState::PARK);
                    break;
                }
                case RegularState::PARK:
                {
                    onFirstTick([&]
                                {
                                wings->toggle_right();
                                drivebase->setTarget("park"); });
                    if (drivebase->isSettled())
                    {
                        changeState(RegularState::IDLE);
                    }
                    break;
                }
                case RegularState::IDLE:
                {
                    break;
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

    int currentCycles = 0;

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

    const auto followPath = [&](std::string path, SkillsState next)
    {
        onFirstTick([&]
                    { drivebase->setTarget(path); });
        if (drivebase->isSettled())
        {
            changeState(next);
        }
    };

    const auto followPathReversed = [&](std::string path, SkillsState next)
    {
        onFirstTick([&]
                    { drivebase->setTarget(path, true); });
        if (drivebase->isSettled())
        {
            changeState(next);
        }
    };

    const auto turn = [&](QAngle angle, SkillsState next)
    {
        onFirstTick([&]
                    { drivebase->turnAngle(angle); });
        if (drivebase->isSettled())
        {
            changeState(next);
        }
    };

    while (timer.getDtFromMark() < 500_ms)
    {
        drivebase->arcade(1, 0);
    }
    timer.placeMark();
    while (timer.getDtFromMark() < 1000_ms)
    {
        drivebase->arcade(-1, 0);
    }
    timer.placeMark();

    while (true)
    {
        catapult->periodic(true);
        COMET_LOG("%0.2f ms since mark", timer.getDtFromMark().convert(okapi::millisecond));

        switch (state)
        {
        case SkillsState::STARTTO_FEEDING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_feeding_move1"); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::STARTTO_FEEDING_TURN1);
            }
            break;
        }
        case SkillsState::STARTTO_FEEDING_TURN1:
        {
            turn(-165_deg, SkillsState::SHOOT);
            break;
        }
        case SkillsState::SHOOT:
        {
            onFirstTick([&]
                        { catapult->fire_and_wind(); });

            if (timer.getDtFromMark() >= FIRST_SKILLS_SHOT_TIME)
            {
                intake->forward();
                changeState(SkillsState::STARTTO_FEEDING_MOVE2);
            }
            break;
        }
        case SkillsState::STARTTO_FEEDING_MOVE2:
        {
            followPath("startto_feeding_move2", SkillsState::CURR_FEEDING);
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
                            intake->reverse();
                            currentCycles++; });

            if (timer.getDtFromMark() >= FIRING_HOLD_DURATION)
            {
                if (currentCycles > SKILLS_CYCLES)
                {
                    changeState(SkillsState::GOTO_SIDE_MOVE1);
                }
                else
                {
                    changeState(SkillsState::GOTO_FEEDING);
                }
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
        case SkillsState::GOTO_SIDE_MOVE1:
        {
            followPath("goto_side_move1", SkillsState::GOTO_SIDE_TURN1);
            break;
        }
        case SkillsState::GOTO_SIDE_TURN1:
        {
            intake->stop();
            turn(45_deg, SkillsState::GOTO_SIDE_MOVE2);
            break;
        }
        case SkillsState::GOTO_SIDE_MOVE2:
        {
            followPath("goto_side_move2", SkillsState::GOTO_SIDE_TURN2);
            break;
        }
        case SkillsState::GOTO_SIDE_TURN2:
        {
            turn(-100_deg, SkillsState::GOTO_SIDE_MOVE3);
            break;
        }
        case SkillsState::GOTO_SIDE_MOVE3:
        {
            followPathReversed("goto_side_move3", SkillsState::GOTO_SIDE_TURN3);
            break;
        }
        case SkillsState::GOTO_SIDE_TURN3:
        {
            turn(45_deg, SkillsState::GOTO_SIDE_MOVE4);
            break;
        }
        case SkillsState::GOTO_SIDE_MOVE4:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_side_move4", true); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::GOTO_SIDE_TURN4);
            }
            break;
        }
        case SkillsState::GOTO_SIDE_TURN4:
        {
            turn(45_deg, SkillsState::GOTO_SIDE_MOVE5);
            break;
        }
        case SkillsState::GOTO_SIDE_MOVE5:
        {
            followPathReversed("goto_side_move5", SkillsState::GOTO_FRONT_MOVE1);
            break;
        }
        case SkillsState::GOTO_FRONT_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_front_move1"); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::GOTO_FRONT_TURN1);
            }
            break;
        }
        case SkillsState::GOTO_FRONT_TURN1:
        {
            turn(90_deg, SkillsState::GOTO_FRONT_MOVE2);
            break;
        }
        case SkillsState::GOTO_FRONT_MOVE2:
        {
            followPathReversed("goto_front_move2", SkillsState::GOTO_FRONT_TURN2);
            break;
        }
        case SkillsState::GOTO_FRONT_TURN2:
        {
            turn(-90_deg, SkillsState::GOTO_FRONT_MOVE3);
            break;
        }
        case SkillsState::GOTO_FRONT_MOVE3:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_front_move3", true);
                        wings->toggle_left();
                        wings->toggle_right(); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::GOTO_FRONT_TURN3);
            }
            break;
        }
        case SkillsState::GOTO_FRONT_TURN3:
        {
            turn(-90_deg, SkillsState::GOTO_FRONT_MOVE4);
            break;
        }
        case SkillsState::GOTO_FRONT_MOVE4:
        {
            followPathReversed("goto_front_move4", SkillsState::IDLE);
            break;
        }
        case SkillsState::GOTO_FRONT_MOVE5:
        {
            onFirstTick([&]
                        { drivebase->setTarget("goto_front_move5"); });
            if (drivebase->isSettled())
            {
                wings->toggle_left();
                wings->toggle_right();
                changeState(SkillsState::IDLE);
            }
        }
        case SkillsState::IDLE:
        {
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

void autonomousTest()
{
    drivebase->setTarget("forward_test");
    drivebase->waitUntilSettled();
}

constexpr std::string_view stateToString(RegularState state)
{
    switch (state)
    {
    case RegularState::STARTTO_PUSHING_MOVE1:
        return "STARTTO_PUSHING_MOVE1";
    case RegularState::STARTTO_PUSHING_TURN1:
        return "STARTTO_PUSHING_TURN1";
    case RegularState::STARTTO_PUSHING_MOVE2:
        return "STARTTO_PUSHING_MOVE2";
    case RegularState::STARTTO_PUSHING_TURN2:
        return "STARTTO_PUSHING_TURN2";
    case RegularState::STARTTO_PUSHING_MOVE3:
        return "STARTTO_PUSHING_MOVE3";
    case RegularState::PUSHING_FORWARD:
        return "PUSHING_FORWARD";
    case RegularState::PUSHING_BACK:
        return "PUSHING_BACK";
    case RegularState::ALLIANCE_TRIBALL_MOVE1:
        return "ALLIANCE_TRIBALL_MOVE1";
    case RegularState::ALLIANCE_TRIBALL_TURN1:
        return "ALLIANCE_TRIBALL_TURN1";
    case RegularState::ALLIANCE_TRIBALL_MOVE2:
        return "ALLIANCE_TRIBALL_MOVE2";
    case RegularState::ALLIANCE_TRIBALL_TURN2:
        return "ALLIANCE_TRIBALL_TURN2";
    case RegularState::ALLIANCE_TRIBALL_MOVE3:
        return "ALLIANCE_TRIBALL_MOVE3";
    case RegularState::WAIT_FOR_INTAKE:
        return "WAIT_FOR_INTAKE";
    case RegularState::SCORE_ALLIANCE_MOVE1:
        return "SCORE_ALLIANCE_MOVE1";
    case RegularState::SCORE_ALLIANCE_TURN1:
        return "SCORE_ALLIANCE_TURN1";
    case RegularState::SCORE_ALLIANCE_MOVE2:
        return "SCORE_ALLIANCE_MOVE2";
    case RegularState::SCORE_ALLIANCE_TURN2:
        return "SCORE_ALLIANCE_TURN2";
    case RegularState::SPIT_OUT_TRIBALL:
        return "SPIT_OUT_TRIBALL";
    case RegularState::SCORE_ALLIANCE_MOVE3:
        return "SCORE_ALLIANCE_MOVE3";
    case RegularState::SCORE_ALLIANCE_BACK:
        return "SCORE_ALLIANCE_BACK";
    case RegularState::SCORE_ALLIANCE_FORWARD:
        return "SCORE_ALLIANCE_FORWARD";
    case RegularState::GOTO_WAITING:
        return "GOTO_WAITING";
    case RegularState::TURN_AROUND:
        return "TURN_AROUND";
    case RegularState::WAIT:
        return "WAIT";
    case RegularState::SCORE_TRIBALLS:
        return "SCORE_TRIBALLS";
    case RegularState::SCORE_TRIBALLS_FORWARD:
        return "SCORE_TRIBALLS_FORWARD";
    case RegularState::SCORE_TRIBALLS_BACK:
        return "SCORE_TRIBALLS_BACK";
    case RegularState::PARK:
        return "PARK";
    case RegularState::IDLE:
        return "IDLE";
    }
    return "Unknown State";
}