#pragma once

#include <cstdint>

#include <ros/ros.h>
#include <can_plugins/Frame.h>

#include "StewLib/judge_lower_cost_than_ref.hpp"
#include "StewLib/serialize.hpp"
#include "StewLib/reverse_buffer.hpp"

#include "CRSLib/std_msgs_type_convertor.hpp"
#include "CRSLib/can_data_convertor.hpp"

namespace CRSLib
{
    namespace
    {
        class CanPublisher final
        {
            inline static ros::Publisher pub{};

            // template<class RawData>
            // static constexpr auto default_can_data_convertor =
            // [](StewLib::low_cost_ref_val_t<StewLib::ReverseBuffer<RawData>> reverse_buffer) noexcept -> StewLib::Serialize<RawData, 8>
            // {
            //     if(!reverse_buffer.is_reversed) reverse_buffer.reverse();
            //     return {static_cast<RawData>(reverse_buffer)};
            // };

        public:
            CanPublisher() noexcept
            {
                if(!pub)
                {
                    ros::NodeHandle nh{"CRSLib"};
                    // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                    pub = nh.advertise<can_plugins::Frame>("can_tx", 1000);
                    /// DEBUG:
                    ROS_INFO("Stew: can_publisher has initialized.");
                }
            }

//             template<class RawData, auto can_data_convertor = CanPublisher::default_can_data_convertor<RawData>>
// #ifdef __cpp_concepts
//             requires requires(const RawData raw_data)
//             {
//                 {can_data_convertor(raw_data)} noexcept -> std::same_as<StewLib::Serialize<RawData, 8>>;
//             }
// #endif
            template<class RawData>
            void can_publish(const std::uint32_t can_id, const StewLib::low_cost_ref_val_t<RawData> raw_data) const noexcept
            {
                const auto serialized_data = CanDataConvertor<RawData>::convert(raw_data);
                using SerializeType = decltype(serialized_data);

                can_plugins::Frame frame_msg;
                frame_msg.id = can_id;
                frame_msg.is_rtr = false;
                frame_msg.is_extended = false;
                frame_msg.is_error = false;
                frame_msg.dlc = 8;
                const auto data_p = frame_msg.data.c_array();

                for(std::size_t i = 0; i < SerializeType::chunks_size - 1; ++i)
                {
                    std::memcpy(data_p, serialized_data.chunks[i], 8);

                    pub.publish(frame_msg);
                }

                frame_msg.dlc = SerializeType::last_size;
                std::memcpy(data_p, serialized_data.chunks[SerializeType::chunks_size - 1], SerializeType::last_size);
                pub.publish(frame_msg);
            }

#if 0
            template<class RawData>
            void publish(const std::uint32_t can_id, const StewLib::low_cost_ref_val_t<RawData> raw_data) const noexcept
            {
                /// TODO: 余力があったらデバッグ用に/stew_can_(ca_id)にRawDataに対応するメッセージ型でパブリッシュする機能を付ける。
                using Message = CRSLib::StdMsgsTypeConvertor<RawData>;
                Message msgs;
                msgs.data = raw_data;
                // 書きかけ。メッセージ型の生成にテンプレートが使えない -> じゃあトピックにcan_id、値をメッセージに入れるか -> Publisherの管理だるいな...一元管理するクラス作るか...？(ここで投げ出す)
            }
#endif
        };
    }
}