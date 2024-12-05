
#include <JHO3DETestGem/JHO3DETestGemTypeIds.h>
#include <JHO3DETestGemModuleInterface.h>
#include "JHO3DETestGemSystemComponent.h"

namespace JHO3DETestGem
{
    class JHO3DETestGemModule
        : public JHO3DETestGemModuleInterface
    {
    public:
        AZ_RTTI(JHO3DETestGemModule, JHO3DETestGemModuleTypeId, JHO3DETestGemModuleInterface);
        AZ_CLASS_ALLOCATOR(JHO3DETestGemModule, AZ::SystemAllocator);
    };
}// namespace JHO3DETestGem

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), JHO3DETestGem::JHO3DETestGemModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_JHO3DETestGem, JHO3DETestGem::JHO3DETestGemModule)
#endif
