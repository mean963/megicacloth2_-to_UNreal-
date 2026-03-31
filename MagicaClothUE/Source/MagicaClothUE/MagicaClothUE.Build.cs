using UnrealBuildTool;

public class MagicaClothUE : ModuleRules
{
	public MagicaClothUE(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		IWYUSupport = IWYUSupport.Full;
		CppStandard = CppStandardVersion.Cpp20;

		// UE 5.6: Disable unity build for cleaner IWYU compliance
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"AnimGraphRuntime",
			"PhysicsCore"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"RenderCore",
			"RHI"
		});
	}
}
