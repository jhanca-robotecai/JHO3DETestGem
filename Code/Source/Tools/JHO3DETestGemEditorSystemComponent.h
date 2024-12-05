
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/JHO3DETestGemSystemComponent.h>

namespace JHO3DETestGem
{
    /// System component for JHO3DETestGem editor
    class JHO3DETestGemEditorSystemComponent
        : public JHO3DETestGemSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = JHO3DETestGemSystemComponent;
    public:
        AZ_COMPONENT_DECL(JHO3DETestGemEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        JHO3DETestGemEditorSystemComponent();
        ~JHO3DETestGemEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace JHO3DETestGem
