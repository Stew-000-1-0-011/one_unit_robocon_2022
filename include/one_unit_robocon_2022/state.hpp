#pragma once

namespace OneUnitRobocon2022
{
    namespace
    {
        enum class State: std::uint8_t
        {
            disable,
            reset,
            manual,
            automatic
        };
    }
}