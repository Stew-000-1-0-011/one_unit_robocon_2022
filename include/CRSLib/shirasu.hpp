#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "CRSLib/can_publisher.hpp"

namespace CRSLib
{
    namespace
    {
        // shirasu_fwのver.1.2のMotorCtrl.hppを見ながら...なんかWikiと並び違くない...？
        enum class Mode : std::uint8_t
        {
            disable = 0,
            default_,
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

        class Shirasu final
        {
            const CanPublisher can_pub;
            std::uint32_t base_id;

        public:
            Shirasu(ros::NodeHandle& nh, const uint32_t base_id) noexcept:
                can_pub{nh},
                base_id{base_id}
            {}

            Shirasu(const Shirasu& obj) noexcept:
                can_pub{obj.can_pub},
                base_id{obj.base_id}
            {}

            Shirasu(Shirasu&& obj) noexcept:
                can_pub{obj.can_pub},
                base_id{obj.base_id}
            {}

            Shirasu& operator=(const Shirasu& obj) noexcept
            {
                if(&obj == this) return *this;
                base_id = obj.base_id;
            }

            Shirasu& operator=(Shirasu&& obj) noexcept
            {
                if(&obj == this) return *this;
                base_id = obj.base_id;
            }

            void send_cmd(const Mode mode) const noexcept
            {
                can_pub.can_publish<Mode>(base_id, mode);
            }

            void send_target(const float target) const noexcept
            {
                can_pub.can_publish<float>(base_id + 1, target);
            }

            void send_diag(const Diag diag) noexcept
            {
                can_pub.can_publish<Diag>(base_id + 2, diag);
            }
        };
    }
}