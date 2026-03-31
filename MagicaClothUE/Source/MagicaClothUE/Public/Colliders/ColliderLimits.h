// MagicaClothUE/Public/Colliders/ColliderLimits.h
#pragma once

#include "CoreMinimal.h"
#include "BoneContainer.h"
#include "Core/ClothTypes.h"
#include "ColliderLimits.generated.h"

/** Base struct for all collision limits (KawaiiPhysics compatible pattern). */
USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Collision")
	FBoneReference DrivingBone;

	UPROPERTY(EditAnywhere, Category = "Collision")
	FVector OffsetLocation = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "-360", ClampMax = "360"))
	FRotator OffsetRotation = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Collision")
	EMagicaLimitType LimitType = EMagicaLimitType::Outer;

	UPROPERTY()
	FVector Location = FVector::ZeroVector;

	UPROPERTY()
	FQuat Rotation = FQuat::Identity;

	UPROPERTY()
	bool bEnable = true;

	UPROPERTY(VisibleAnywhere, Category = "Collision")
	EMagicaCollisionSourceType SourceType = EMagicaCollisionSourceType::AnimNode;
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaSphericalLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Spherical Limit", meta = (ClampMin = "0"))
	float Radius = 5.0f;
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaCapsuleLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Capsule Limit", meta = (ClampMin = "0"))
	float Radius = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Capsule Limit", meta = (ClampMin = "0"))
	float Length = 10.0f;
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaBoxLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Box Limit")
	FVector Extent = FVector(5.0f, 5.0f, 5.0f);
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaPlanarLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY()
	FPlane Plane = FPlane(0, 0, 0, 0);
};
