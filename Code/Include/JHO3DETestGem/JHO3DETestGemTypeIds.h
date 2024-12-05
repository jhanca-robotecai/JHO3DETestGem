
#pragma once

namespace JHO3DETestGem
{
    // System Component TypeIds
    inline constexpr const char* JHO3DETestGemSystemComponentTypeId = "{9EC1A3C8-FD92-4BD1-B1D4-F97FBB74F275}";
    inline constexpr const char* JHO3DETestGemEditorSystemComponentTypeId = "{2740ABC7-ECC5-478F-8C66-71B435858012}";

    // Module derived classes TypeIds
    inline constexpr const char* JHO3DETestGemModuleInterfaceTypeId = "{A113B67F-FB58-41F6-97AD-1CDC2F0A6FCA}";
    inline constexpr const char* JHO3DETestGemModuleTypeId = "{A380F80F-B3CE-4380-A299-5B1CA2C09B85}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* JHO3DETestGemEditorModuleTypeId = JHO3DETestGemModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* JHO3DETestGemRequestsTypeId = "{AD46AC56-ED50-465D-8BE6-2F763D385DC4}";
} // namespace JHO3DETestGem
