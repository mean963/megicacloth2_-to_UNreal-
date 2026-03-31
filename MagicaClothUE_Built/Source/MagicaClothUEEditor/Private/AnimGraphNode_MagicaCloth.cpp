#include "AnimGraphNode_MagicaCloth.h"

#define LOCTEXT_NAMESPACE "MagicaClothUEEditor"

FText UAnimGraphNode_MagicaCloth::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return LOCTEXT("NodeTitle", "Magica Cloth Simulation");
}

FText UAnimGraphNode_MagicaCloth::GetTooltipText() const
{
	return LOCTEXT("Tooltip", "PBD-based cloth physics for bone chains and skirts. Supports Physics Asset colliders and multi-chain mode. Runs on a dedicated thread at configurable Hz.");
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
