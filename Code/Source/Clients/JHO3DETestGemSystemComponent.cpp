
#include "JHO3DETestGemSystemComponent.h"

#include <JHO3DETestGem/JHO3DETestGemTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace JHO3DETestGem
{
    AZ_COMPONENT_IMPL(JHO3DETestGemSystemComponent, "JHO3DETestGemSystemComponent",
        JHO3DETestGemSystemComponentTypeId);

    void JHO3DETestGemSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JHO3DETestGemSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void JHO3DETestGemSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JHO3DETestGemService"));
    }

    void JHO3DETestGemSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JHO3DETestGemService"));
    }

    void JHO3DETestGemSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void JHO3DETestGemSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    JHO3DETestGemSystemComponent::JHO3DETestGemSystemComponent()
    {
        if (JHO3DETestGemInterface::Get() == nullptr)
        {
            JHO3DETestGemInterface::Register(this);
        }
    }

    JHO3DETestGemSystemComponent::~JHO3DETestGemSystemComponent()
    {
        if (JHO3DETestGemInterface::Get() == this)
        {
            JHO3DETestGemInterface::Unregister(this);
        }
    }

    void JHO3DETestGemSystemComponent::Init()
    {
    }

    void JHO3DETestGemSystemComponent::Activate()
    {
        JHO3DETestGemRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void JHO3DETestGemSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        JHO3DETestGemRequestBus::Handler::BusDisconnect();
    }

    void JHO3DETestGemSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace JHO3DETestGem
