
#include "JHO3DETestGemModuleInterface.h"
#include "TestComponent/TestComponent.h"
#include <AzCore/Memory/Memory.h>

#include <JHO3DETestGem/JHO3DETestGemTypeIds.h>

#include <Clients/JHO3DETestGemSystemComponent.h>

namespace JHO3DETestGem
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(JHO3DETestGemModuleInterface,
        "JHO3DETestGemModuleInterface", JHO3DETestGemModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(JHO3DETestGemModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(JHO3DETestGemModuleInterface, AZ::SystemAllocator);

    JHO3DETestGemModuleInterface::JHO3DETestGemModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            JHO3DETestGemSystemComponent::CreateDescriptor(),
            TestComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList JHO3DETestGemModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<JHO3DETestGemSystemComponent>(),
        };
    }
} // namespace JHO3DETestGem
