/*
 * Copyright (c) Robotec.ai.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "TestComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <AzFramework/Asset/GenericAssetHandler.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

#include <iostream>

namespace JHO3DETestGem
{
    void TestComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TestComponent, AZ::Component>()->Version(0)->Field(
                "SubscriberConfiguration", &TestComponent::m_subscriberConfiguration);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<TestComponent>("TestComponent", "TestComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "JHO3DETestGem")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &TestComponent::m_subscriberConfiguration,
                        "Subscriber configuration",
                        "ROS 2 subscriber configuration for printing id of the spawnable");
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

        m_subscriber = ros2Node->create_subscription<std_msgs::msg::String>(
            ROS2::ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_subscriberConfiguration.m_topic).c_str(),
            m_subscriberConfiguration.GetQoS(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                const AZStd::string prefabPath = msg->data.c_str();
                std::cout << "JHDEBUG JHDEBUG JHDEBUG SpawnablePath: " << prefabPath.c_str() << "\n";
                HelperMethod(prefabPath);
            });
    }

    void TestComponent::Deactivate()
    {
        m_subscriber.reset();
    }

    void TestComponent::HelperMethod(const AZStd::string& prefabPath)
    {
        AZ::IO::Path spawnablePath(prefabPath);
        spawnablePath.ReplaceExtension("spawnable");
        AZ::Data::AssetId spawnableId = AZ::Data::AssetId();
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
            spawnableId,
            &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
            spawnablePath.c_str(),
            azrtti_typeid<AzFramework::Spawnable>(),
            false);

        std::cout << "JHDEBUG JHDEBUG JHDEBUG SpawnablePath: " << spawnablePath.c_str() << "\n";
        std::cout << "JHDEBUG JHDEBUG JHDEBUG SpawnableId: " << spawnableId.m_guid.ToString<AZStd::string>().c_str() << "\n";

        if (!spawnableId.IsValid())
        {
            std::cout << "JHDEBUG JHDEBUG JHDEBUG Cannot spawn\n";
            return;
        }

        AZ::Data::Asset<AzFramework::Spawnable> spawnableAsset =
            AZ::Data::AssetManager::Instance().GetAsset<AzFramework::Spawnable>(spawnableId, AZ::Data::AssetLoadBehavior::QueueLoad);

        auto spawnableTicket = AzFramework::EntitySpawnTicket(spawnableAsset);
        m_tickets.emplace(prefabPath, AZStd::move(spawnableTicket));

        auto* spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        spawner->SpawnAllEntities(m_tickets.at(prefabPath));

        std::cout << "JHDEBUG JHDEBUG JHDEBUG Spawned\n";
    }
} // namespace JHO3DETestGem
