#pragma once

#include <cstdint>
#include <unordered_map>

namespace CRSLib
{
    namespace
    {
        // shirasu_fwのver.1.2のMotorCtrl.hppを見ながら...なんかWikiと並び違くない...？
        enum class Mode : std::uint8_t
        {
            disable = 0,
            default,
            homing,
            reserved,
            current,
            velocity,
            position
        };

        /// TODO: ほんとにこれでいいのか要確認！！
        enum class Diag : std::uint8_t
        {
            usb,
            current,
            velocity,
            position
        };

        struct Shirasu final
        {
            std::unordered_map<>
        };
    }
}