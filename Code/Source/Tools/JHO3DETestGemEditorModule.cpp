
#include <JHO3DETestGem/JHO3DETestGemTypeIds.h>
#include <JHO3DETestGemModuleInterface.h>
#include "JHO3DETestGemEditorSystemComponent.h"

namespace JHO3DETestGem
{
    class JHO3DETestGemEditorModule
        : public JHO3DETestGemModuleInterface
    {
    public:
        AZ_RTTI(JHO3DETestGemEditorModule, JHO3DETestGemEditorModuleTypeId, JHO3DETestGemModuleInterface);
        AZ_CLASS_ALLOCATOR(JHO3DETestGemEditorModule, AZ::SystemAllocator);

        JHO3DETestGemEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                JHO3DETestGemEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<JHO3DETestGemEditorSystemComponent>(),
            };
        }
    };
}// namespace JHO3DETestGem

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), JHO3DETestGem::JHO3DETestGemEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_JHO3DETestGem_Editor, JHO3DETestGem::JHO3DETestGemEditorModule)
#endif
