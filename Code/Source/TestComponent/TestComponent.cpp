/*
 * Copyright (c) Robotec.ai.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "TestComponent.h"
#include <AtomLyIntegration/CommonFeatures/CoreLights/AreaLightBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace JHO3DETestGem
{
    void TestComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TestComponent>()
                ->Version(0)
                ->Field("ServiceName", &TestComponent::m_serviceName)
                ->Field("LightsEntityId", &TestComponent::m_lightsEntityId)
                ->Field("SubscriberConfiguration", &TestComponent::m_subscriberConfiguration)
                ->Field("PublisherConfiguration", &TestComponent::m_publisherConfiguration);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<TestComponent>("TestComponent", "TestComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "JHO3DETestGem")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &TestComponent::m_serviceName, "Service name", "Service name for turning on lights")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &TestComponent::m_lightsEntityId, "Lights Entity", "EntityId served by the service")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &TestComponent::m_subscriberConfiguration,
                        "Subscriber configuration",
                        "ROS 2 subscriber configuration for switching off the lights")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &TestComponent::m_publisherConfiguration,
                        "Publisher configuration",
                        "ROS 2 publisher configuration for the lights status");
            }
        }
    }

    void TestComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void TestComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (!ros2Node)
        {
            AZ_Error("TestComponent", false, "ROS2 node is not available. ROS 2 services will not be created.");
            return;
        }

        auto ros2Frame = ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity());
        if (!ros2Frame)
        {
            AZ_Error("TestComponent", false, "ROS2 frame is not available. ROS 2 services will not be created.");
            return;
        }

        AZStd::string serviceName = ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_serviceName);
        m_lightsOnService = ros2Node->create_service<std_srvs::srv::Trigger>(
            serviceName.c_str(),
            [this](
                [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                if (!m_lightsEntityId.IsValid())
                {
                    response->success = false;
                    response->message = "Cannot turn on lights, entity ID not found.";
                    return;
                }

                constexpr float targetIntensity = 500.0f;
                response->success = SetLightsIntensity(targetIntensity);
                if (response->success)
                {
                    response->message = "Lights turned on.";
                }
                else
                {
                    response->message = "Failed to turn the lights on.";
                }
            });

        m_subscriber = ros2Node->create_subscription<std_msgs::msg::Float32>(
            ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_subscriberConfiguration.m_topic).c_str(),
            m_subscriberConfiguration.GetQoS(),
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                // Copy the message to avoid lifetime issues.
                const float targetIntensity = msg->data;

                if (m_lightsEntityId.IsValid())
                {
                    SetLightsIntensity(targetIntensity);
                }
            });

        m_publisher = ros2Node->create_publisher<std_msgs::msg::Float32>(
            ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_publisherConfiguration.m_topic).c_str(),
            m_publisherConfiguration.GetQoS());

        AZ::TickBus::Handler::BusConnect();
    }

    void TestComponent::Deactivate()
    {
        m_lightsOnService.reset();
        m_subscriber.reset();
        m_publisher.reset();

        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    bool TestComponent::SetLightsIntensity(const float targetIntensity)
    {
        const AZ::Render::PhotometricUnit photoUnit = AZ::Render::PhotometricUnit::Lumen;
        AZ::Render::AreaLightRequestBus::Event(
            m_lightsEntityId, &AZ::Render::AreaLightRequests::SetIntensityAndMode, targetIntensity, photoUnit);

        float currentIntensity = -100.0f;
        AZ::Render::AreaLightRequestBus::EventResult(currentIntensity, m_lightsEntityId, &AZ::Render::AreaLightRequests::GetIntensity);

        return AZ::IsClose(targetIntensity, currentIntensity);
    }

    void TestComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        if (!m_lightsEntityId.IsValid())
        {
            return;
        }

        // Note: demo only, it makes not sense to publish info every OnTick()
        float currentIntensity = -100.0f;
        AZ::Render::AreaLightRequestBus::EventResult(currentIntensity, m_lightsEntityId, &AZ::Render::AreaLightRequests::GetIntensity);

        std_msgs::msg::Float32 msg;
        msg.data = currentIntensity;

        m_publisher->publish(msg);
    }
} // namespace JHO3DETestGem
