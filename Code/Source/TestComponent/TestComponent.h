/*
 * Copyright (c) Robotec.ai.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/utility/pair.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace JHO3DETestGem
{
    class TestComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
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
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_lightsOnService;
        AZStd::string m_serviceName{ "service_name" };
        AZ::EntityId m_lightsEntityId;

        ROS2::TopicConfiguration m_subscriberConfiguration;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subscriber;

        ROS2::TopicConfiguration m_publisherConfiguration;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_publisher;

        bool SetLightsIntensity(const float targetIntensity);
    };
} // namespace JHO3DETestGem
