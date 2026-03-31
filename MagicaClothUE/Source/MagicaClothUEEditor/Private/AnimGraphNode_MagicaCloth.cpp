#include "AnimGraphNode_MagicaCloth.h"

#define LOCTEXT_NAMESPACE "MagicaClothUEEditor"

FText UAnimGraphNode_MagicaCloth::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	if (TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle)
	{
		return LOCTEXT("NodeTitle_List", "Magica Cloth Simulation");
	}

	// Full title: show root bone name like KawaiiPhysics ("Magica Cloth - Root: BoneName")
	const FName RootBoneName = Node.RootBone.BoneName;
	if (RootBoneName.IsNone())
	{
		return LOCTEXT("NodeTitle_NoRoot", "Magica Cloth Simulation\nRoot: (none)");
	}

	FFormatNamedArguments Args;
	Args.Add(TEXT("RootBone"), FText::FromName(RootBoneName));
	return FText::Format(LOCTEXT("NodeTitle_WithRoot", "Magica Cloth Simulation\nRoot: {RootBone}"), Args);
}

FText UAnimGraphNode_MagicaCloth::GetTooltipText() const
{
	return LOCTEXT("Tooltip",
		"Magica Cloth Simulation\n"
		"PBD-based cloth physics for bone chains and skirts.\n"
		"Supports Physics Asset colliders, DataAsset limits, and multi-chain (skirt) mode.\n"
		"Runs on a dedicated simulation thread at configurable Hz.\n"
		"Set 'Root Bone' to define the starting joint of the cloth chain.");
}

FString UAnimGraphNode_MagicaCloth::GetNodeCategory() const
{
	return TEXT("Magica Cloth");
}

FText UAnimGraphNode_MagicaCloth::GetControllerDescription() const
{
	return LOCTEXT("ControllerDescription", "Magica Cloth (PBD Bone Chain Physics)");
}

#undef LOCTEXT_NAMESPACE
