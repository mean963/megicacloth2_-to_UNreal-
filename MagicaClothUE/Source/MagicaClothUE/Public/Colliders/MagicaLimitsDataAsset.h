// MagicaClothUE/Public/Colliders/MagicaLimitsDataAsset.h
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Colliders/ColliderLimits.h"
#include "MagicaLimitsDataAsset.generated.h"

/**
 * Reusable collider configuration asset (KawaiiPhysics LimitsDataAsset pattern).
 * Store collision setup once, reference from multiple AnimBP nodes.
 */
UCLASS(BlueprintType)
class MAGICACLOTHUE_API UMagicaLimitsDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spherical Limits")
	TArray<FMagicaSphericalLimit> SphericalLimits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Capsule Limits")
	TArray<FMagicaCapsuleLimit> CapsuleLimits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Limits")
	TArray<FMagicaBoxLimit> BoxLimits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planar Limits")
	TArray<FMagicaPlanarLimit> PlanarLimits;
};
