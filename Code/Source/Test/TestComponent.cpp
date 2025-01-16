/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TestComponent.h"

#include <AzCore/Serialization/EditContext.h>

namespace JHO3DETestGem
{
    void TestComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<TestComponent, AZ::Component>()
                ->Version(0)
                ->Field("name", &TestComponent::m_name)
                ->Field("vector", &TestComponent::m_vector);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<TestComponent>("TestComponent", "TestComponent made for tests")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &TestComponent::m_name, "Name", "Test component name")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &TestComponent::m_vector, "Vector", "Test component vector");
            }
        }
    }

    TestComponent::TestComponent()
    {
    }

    void TestComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void TestComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void TestComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        // code goes here
    }
} // namespace JHO3DETestGem
