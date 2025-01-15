
#include <AzCore/Serialization/SerializeContext.h>
#include "JHO3DETestGemEditorSystemComponent.h"

#include <JHO3DETestGem/JHO3DETestGemTypeIds.h>
#include <iostream>

namespace JHO3DETestGem
{
    AZ_COMPONENT_IMPL(JHO3DETestGemEditorSystemComponent, "JHO3DETestGemEditorSystemComponent",
        JHO3DETestGemEditorSystemComponentTypeId, BaseSystemComponent);

    void JHO3DETestGemEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JHO3DETestGemEditorSystemComponent, JHO3DETestGemSystemComponent>()
                ->Version(0);
        }
    }

    JHO3DETestGemEditorSystemComponent::JHO3DETestGemEditorSystemComponent() = default;

    JHO3DETestGemEditorSystemComponent::~JHO3DETestGemEditorSystemComponent() = default;

    void JHO3DETestGemEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("JHO3DETestGemEditorService"));
    }

    void JHO3DETestGemEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("JHO3DETestGemEditorService"));
    }

    void JHO3DETestGemEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void JHO3DETestGemEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void JHO3DETestGemEditorSystemComponent::Activate()
    {
        JHO3DETestGemSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void JHO3DETestGemEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        JHO3DETestGemSystemComponent::Deactivate();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void JHO3DETestGemEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        std::cout << "JHDEBUG: OnStartPlayInEditorBegin!!!!\n";
        AZ_Error("JHDEBUG", false, "Triggering OnStartPlayInEditorBegin");
    }
    void JHO3DETestGemEditorSystemComponent::OnStopPlayInEditor()
    {
        std::cout << "JHDEBUG: OnStopPlayInEditor!!!!\n";
        AZ_Error("JHDEBUG", false, "Triggering OnStopPlayInEditor");
    }

} // namespace JHO3DETestGem
