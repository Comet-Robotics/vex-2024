#ifndef __COMETS_CONTROLS_H__
#define __COMETS_CONTROLS_H__

#include <utility>
#include "comets/vendor.h"

namespace comets
{
    class EdgeDetector
    {
    public:
        constexpr EdgeDetector() = default;

        constexpr void monitor(bool input) {
            // this->previousInput = std::exchange(this->currentInput, input);
            this->previousInput = this->currentInput;
            this->currentInput = input;
        }

        inline constexpr bool isPushed()
        {
            return !previousInput && currentInput;
        }
        inline constexpr bool isReleased()
        {
            return previousInput && !currentInput;
        }

    private:
        bool previousInput = false;
        bool currentInput = false;
    };
}

#endif
