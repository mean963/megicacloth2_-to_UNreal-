#pragma once

#include "CoreMinimal.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "AnimNode/AnimNode_MagicaCloth.h"
#include "AnimGraphNode_MagicaCloth.generated.h"

/**
 * Editor node for the Magica Cloth AnimBP node.
 * Provides the visual representation in the AnimGraph editor (UE 5.6).
 */
UCLASS()
class MAGICACLOTHUEEDITOR_API UAnimGraphNode_MagicaCloth : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Settings")
	FAnimNode_MagicaCloth Node;

	// UEdGraphNode interface
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	virtual FString GetNodeCategory() const override;

	// UAnimGraphNode_SkeletalControlBase interface
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }

protected:
	virtual FText GetControllerDescription() const override;
};
