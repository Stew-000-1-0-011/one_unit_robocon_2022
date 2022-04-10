#pragma once

#ifdef __cpp_concepts
#include <concepts>
#endif

#include <initializer_list>
#include <utility>
#include <memory>
#include <tuple>

#include <ros/ros.h>
#include <can_plugins/Frame.h>

#include "StewLib/serialize.hpp"

#include "can_data_convertor.hpp"


namespace CRSLib
{
    namespace
    {
        namespace CanSubscriberManagerImplement
        {
            template<class RawData>
            struct DefaultPush
            {
                static_assert([]{return false;}(), "DefaultPush supports only scalar type.");
            };

            template<class RawData>
#ifdef __cpp_concepts
            requires std::is_scalar_v<RawData>
#endif
            struct DefaultPush<RawData> final
            {
                static bool push(const char *const top_p, [[maybe_unused]] char * current_p, const can_plugins::Frame::ConstPtr& frame_p) noexcept
                {
                    // c_arrayはT*を返すので使えないっぽい。
                    std::memcpy(top_p, &frame_p->data[0], sizeof(RawData));
                    return true;
                }
            };
        }

        template<class RawData, class Callback, class Push = CanSubscriberManagerImplement::DefaultPush<RawData>>
        struct CanSubscriber;

        namespace CanSubscriberManagerImplement
        {
            template<class T>
            struct is_can_subscriber_t : std::false_type
            {};

            template<class RawData, class Callback, class Push>
            struct is_can_subscriber_t<CanSubscriber<RawData, Callback, Push>> : std::true_type
            {};
        }

#ifdef __cpp_concepts
        namespace CanSubscriberManagerConcept
        {
            template<class T>
            concept is_can_subscriber = CanSubscriberManagerImplement::is_can_subscriber_t<T>::value;

            template<class Callback, class RawData>
            concept is_callback = requires(const RawData raw_data)
            {
                {Callback::callback(raw_data)} noexcept;
            };

            template<class Push>
            concept is_push = requires(const char *const top_p, char * current_p, const can_plugins::Frame::ConstPtr& frame_p)
            {
                {Push::push(top_p, current_p, frame_p)} noexcept -> std::convertible_to<bool>;
            };
        }
#endif

        template<class RawData, class Callback, class Push /* = CanSubscriberManagerImplement::DefaultPush<RawData> */>
#ifdef __cpp_concepts
        requires CanSubscriberManagerConcept::is_callback<Callback, RawData> && CanSubscriberManagerConcept::is_push<Push>
#endif
        struct CanSubscriber<RawData, Callback, Push> final
        {
            uint32_t can_id;
            StewLib::Serialize<RawData, 8> buffer{};
            const char *const top_p{buffer.chuncks};
            char * current_p{buffer.chuncks};

            CanSubscriber(const uint32_t can_id) noexcept:
                can_id{can_id}
            {}

            void callback(const can_plugins::Frame::ConstPtr& frame_p) noexcept
            {
                if(frame_p->id != can_id) return;
                
                if(Push::push(top_p, current_p, frame_p)) Callback::callback(static_cast<RawData>(buffer));
            }
        };

        template<class ... CanSubscribers>
#ifdef __cpp_concepts
        requires requires
        {
            sizeof...(CanSubscribers) > 0;
            
            // requires (... && std::default_constructive<CanSubscribers>);
            // requires (... && CanSubscriberImplement::is_callback<CanSubscribers>);
        }
#endif
        class CanSubscriberManager final
        {
            ros::Subscriber sub{};
            std::tuple<CanSubscribers ...> can_subscribers;

        public:
            CanSubscriberManager(const std::initializer_list<uint32_t> init) noexcept:
            can_subscribers{init}
            {
                if(!sub)
                {
                    ros::NodeHandle nh{"CRSLib"};
                    // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                    sub = nh.advertise<can_plugins::Frame>("can_rx", 1000, &CanSubscriberManager::callback, this);
                    /// DEBUG:
                    ROS_INFO("Stew: can_subscriber is initialized.");
                }
                else
                {
                    // コールバックがあるのでCanSubscirberは一度しか初期化されない。
                    ROS_ERROR("Stew: can_subscriber has already been initialized.");
                }
            }

        private:
            void callback(const typename can_plugins::Frame::ConstPtr& frame_p) noexcept
            {
                [this, &frame_p]<size_t ... indexs>(const std::index_sequence<indexs ...>)
                {
                    (... , std::get<indexs>(can_subscribers).callback(frame_p));
                }(std::make_index_sequence<sizeof...(CanSubscribers)>());
            }
        };
    }
}