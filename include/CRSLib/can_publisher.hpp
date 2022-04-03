#pragma once

#include <cstdint>

#include <ros/ros.h>
#include <can_plugins/Frame.h>

#include <StewLib/judge_lower_cost_than_ref.hpp>
#include <StewLib/serialize.hpp>

namespace CRSLib
{
    namespace
    {
        class CanPublisher final
        {
            ros::Publisher pub;
            template<class RawData>
            static constexpr auto default_can_data_convertor = []()

        public:
            CanPublisher(ros::NodeHandle& nh) noexcept:
                pub{nh.advertise<can_plugins::Frame>("can_tx", 1000)}  // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので潔く直値で初期化)
            {}

            template<class RawData, auto >
            void can_publish(const std::uint32_t can_id, const StewLib:: StewLib::Serialize<RawData, 8> serialized_data) noexcept
            {
                using SerializeType = StewLib::Serialize<RawData, 8>;

                can_plugins::Frame frame_msg;
                frame_msg.id = can_id;
                frame_msg.is_rtr = false;
                frame_msg.is_extended = false;
                frame_msg.is_error = false;
                frame_msg.dlc = 8;
                auto data_p = &frame_msg.data[0];

                for(std::size_t i = 0; i < SerializeType::chunks_size - 1; ++i)
                {
                    std::memcpy(data_p, serialized_data[i], 8);

                    pub.publish(frame_msg);
                }

                frame_msg.dlc = SerializeType::last_size;
                std::memcpy(data_p, serialized_data[SerializeType::chunks_size - 1], SerializeType::last_size);
                pub.publish(frame_msg);


            }
        };
    }
}