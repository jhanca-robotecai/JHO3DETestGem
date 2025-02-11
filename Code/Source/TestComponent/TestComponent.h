/*
 * Copyright (c) Robotec.ai.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace JHO3DETestGem
{
    class TestComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(TestComponent, "{98D5A3B5-D8C7-4D4F-8B75-4CBE47D05E08}");
        TestComponent(){};
        ~TestComponent(){};

        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        ROS2::TopicConfiguration m_subscriberConfiguration;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscriber;

        void HelperMethod(const AZStd::string& prefabPath);
        AZStd::unordered_map<AZStd::string, AzFramework::EntitySpawnTicket> m_tickets;
    };
} // namespace JHO3DETestGem
