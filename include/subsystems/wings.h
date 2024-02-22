#pragma once

#include "comets/vendor.h"

class Wings final
{
public:
    Wings();

    void toggle_left();
    void toggle_right();

    double position_left() const
    {
        return m_left.getPosition();
    }
    double position_right() const
    {
        return m_right.getPosition();
    }

private:
    mutable okapi::Motor m_left, m_right;
    mutable pros::Task m_left_task, m_right_task;

    static void
    toggle_motor(okapi::Motor &motor);
    static void
    task_handler(okapi::Motor &motor);
};
