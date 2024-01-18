#ifndef __COMETS_PATHS__
#define __COMETS_PATHS__

#include <string_view>
#include "comets/vendor.h"

namespace comets
{
    struct path_plan
    {
        std::string_view name;
        std::initializer_list<okapi::PathfinderPoint> points;
    };
} // namespace comets

#endif
