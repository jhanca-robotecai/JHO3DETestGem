
#pragma once

#include <JHO3DETestGem/JHO3DETestGemTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace JHO3DETestGem
{
    class JHO3DETestGemRequests
    {
    public:
        AZ_RTTI(JHO3DETestGemRequests, JHO3DETestGemRequestsTypeId);
        virtual ~JHO3DETestGemRequests() = default;
        // Put your public methods here
    };

    class JHO3DETestGemBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using JHO3DETestGemRequestBus = AZ::EBus<JHO3DETestGemRequests, JHO3DETestGemBusTraits>;
    using JHO3DETestGemInterface = AZ::Interface<JHO3DETestGemRequests>;

} // namespace JHO3DETestGem
