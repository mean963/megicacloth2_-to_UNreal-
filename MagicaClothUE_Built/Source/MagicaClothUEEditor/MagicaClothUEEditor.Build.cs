using UnrealBuildTool;

public class MagicaClothUEEditor : ModuleRules
{
	public MagicaClothUEEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		IWYUSupport = IWYUSupport.Full;
		CppStandard = CppStandardVersion.Cpp20;
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"AnimGraph",
			"AnimGraphRuntime",
			"BlueprintGraph",
			"MagicaClothUE"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"UnrealEd",
			"SlateCore",
			"Slate",
			"PropertyEditor"
		});
	}
}
