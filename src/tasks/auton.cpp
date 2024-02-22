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

inline constexpr auto REGULAR_WAIT_TIME = 15000_ms;

inline constexpr auto SKILLS = false;

enum class SkillsState
{
    STARTTO_FEEDING,
    GOTO_FEEDING,
    CURR_FEEDING,
    GOTO_FIRING,
    CURR_FIRING,
};

enum class RegularState
{
    STARTTO_PUSHING_MOVE1,
    STARTTO_PUSHING_TURN1,
    STARTTO_PUSHING_MOVE2,
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

constexpr std::string_view stateToString(RegularState state);

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
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {4_ft, 0_ft, 0_deg}}, "goto_feed");

    // regular
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {60_in, 0_ft, 0_deg}}, "startto_pushing_move1");
    drivebase->generatePath({{0_ft, 3_ft, 45_deg}, {12_in, 0_ft, 0_deg}}, "startto_pushing_move2");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "pushing_forward");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "pushing_back");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {10_in, 0_ft, 0_deg}}, "alliance_triball_move1");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {14_in, 0_ft, 0_deg}}, "alliance_triball_move2");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {8_in, 0_ft, 0_deg}}, "alliance_triball_move3");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {8_in, 0_ft, 0_deg}}, "score_alliance_move1");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {14_in, 0_ft, 0_deg}}, "score_alliance_move2");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {10_in, 0_ft, 0_deg}}, "score_alliance_move3");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "score_alliance_back");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "score_alliance_forward");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {0_in, 0_ft, 0_deg}}, "goto_waiting"); // spline
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {16_in, 0_ft, 0_deg}}, "score_triballs");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "score_triballs_forward");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {12_in, 0_ft, 0_deg}}, "score_triballs_back");
    drivebase->generatePath({{0_ft, 0_ft, 0_deg}, {30_in, 60_in, 90_deg}}, "park");
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
    auto state = RegularState::STARTTO_PUSHING_MOVE1;
    bool first_tick_in_state = true;

    catapult->zero_position();

    Timer timer;
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
        case RegularState::STARTTO_PUSHING_MOVE1:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_pushing_move1", true);
                        catapult->fire_and_wind_partly(); });
            if (drivebase->isSettled())
            {
                changeState(RegularState::STARTTO_PUSHING_TURN1);
            }
            break;
        }
        case RegularState::STARTTO_PUSHING_TURN1:
        {
            turn(-45_deg, RegularState::STARTTO_PUSHING_MOVE2);
            break;
        }
        case RegularState::STARTTO_PUSHING_MOVE2:
        {
            // extend wings
            followPathReversed("startto_pushing_move2", RegularState::PUSHING_FORWARD);
            break;
        }
        case RegularState::PUSHING_FORWARD:
        {
            followPath("pushing_forward", RegularState::PUSHING_BACK);
            break;
        }
        case RegularState::PUSHING_BACK:
        {
            followPathReversed("pushing_back", RegularState::ALLIANCE_TRIBALL_MOVE1);
            break;
        }
        case RegularState::ALLIANCE_TRIBALL_MOVE1:
        {
            followPath("alliance_triball_move1", RegularState::ALLIANCE_TRIBALL_TURN1);
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
                        { catapult->wind_back(); });
            if (timer.getDtFromMark() > REGULAR_WAIT_TIME)
            {
                changeState(RegularState::SCORE_TRIBALLS);
            }
            break;
        }
        case RegularState::SCORE_TRIBALLS:
        {
            // extend wings
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
            followPath("park", RegularState::IDLE);
            break;
        }
        case RegularState::IDLE:
        {
            break;
        }
        }
    }
}

void autonomousSkills()
{
    auto state = SkillsState::STARTTO_FEEDING;
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
        case SkillsState::STARTTO_FEEDING:
        {
            onFirstTick([&]
                        { drivebase->setTarget("startto_feed"); });
            if (drivebase->isSettled())
            {
                changeState(SkillsState::CURR_FEEDING);
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
    case RegularState::PUSHING_FORWARD:
        return "PUSHING_FORWARD";
    case RegularState::PUSHING_BACK:
        return "PUSHING_BACK";
    case RegularState::ALLIANCE_TRIBALL_TURN1:
        return "ALLIANCE_TRIBALL_TURN1";
    case RegularState::ALLIANCE_TRIBALL_MOVE1:
        return "ALLIANCE_TRIBALL_MOVE1";
    case RegularState::ALLIANCE_TRIBALL_TURN2:
        return "ALLIANCE_TRIBALL_TURN2";
    case RegularState::ALLIANCE_TRIBALL_MOVE2:
        return "ALLIANCE_TRIBALL_MOVE2";
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