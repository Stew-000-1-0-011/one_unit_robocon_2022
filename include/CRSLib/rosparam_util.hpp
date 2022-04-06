#pragma once

#include <string>

#ifdef __cpp_concepts
#include <concepts>
#endif

#include <ros/ros.h>

#include "StewLib/judge_lower_cost_than_ref.hpp"

namespace CRSLib
{
    namespace
    {
        namespace RosparamUtil
        {
            template<class Variable, class Functor, class DefaultValue>
#ifdef __cpp_concepts
            requires requires (Variable obj, Functor pred)
            {
                {pred.template operator()<Variable>(obj)} noexcept -> std::same_as<bool>;
            }
#endif
            void get_param(ros::NodeHandle& pnh, const std::string& obj_name, Variable& obj, Functor pred, const DefaultValue default_value)
            {
                pnh.getParam(obj_name, obj);
                if(!(pred.template operator()<Variable>(obj)))
                {
                    ROS_ERROR_STREAM("Stew: rosparam error. " << obj_name << " does not satisfy predicate. " << obj_name << " is set to the default value of " << default_value << '.');
                }
            }

            inline constexpr auto is_positive = []<typename T>(const StewLib::low_cost_ref_val_t<T> obj) noexcept -> bool
            {
                return obj > 0;
            };

            inline constexpr auto is_nonzero = []<typename T>(const StewLib::low_cost_ref_val_t<T> obj) noexcept -> bool
            {
                return static_cast<bool>(obj);
            };
        }
    }
}