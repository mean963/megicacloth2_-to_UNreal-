// MagicaClothUE/Public/Core/ClothTypes.h
#pragma once

#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"
#include "BoneContainer.h"
#include "ClothTypes.generated.h"

/** Cloth simulation mode. */
UENUM(BlueprintType)
enum class EMagicaClothType : uint8
{
	BoneCloth  = 0 UMETA(DisplayName = "Bone Cloth"),
	MeshCloth  = 1 UMETA(DisplayName = "Mesh Cloth"),
};

/** Simulation coordinate space. */
UENUM(BlueprintType)
enum class EMagicaSimulationSpace : uint8
{
	ComponentSpace UMETA(DisplayName = "Component Space"),
	WorldSpace     UMETA(DisplayName = "World Space"),
	BaseBoneSpace  UMETA(DisplayName = "Base Bone Space"),
};

/** Collider limit type: push particles outside or keep inside. */
UENUM(BlueprintType)
enum class EMagicaLimitType : uint8
{
	Outer UMETA(DisplayName = "Outer (Push Out)"),
	Inner UMETA(DisplayName = "Inner (Keep Inside)"),
};

/** Where a collision limit was defined. */
UENUM()
enum class EMagicaCollisionSourceType : uint8
{
	AnimNode,
	DataAsset,
	PhysicsAsset,
};

/** Range within a flat global array. Used by TeamManager. */
USTRUCT()
struct FMagicaDataChunk
{
	GENERATED_BODY()

	int32 StartIndex = 0;
	int32 Count      = 0;

	int32 End() const { return StartIndex + Count; }
};

/** Per-chain range within a Team's particle array. */
USTRUCT()
struct FMagicaChainRange
{
	GENERATED_BODY()

	int32 StartIndex = 0;
	int32 Count      = 0;
};

/** Physics settings bundle (KawaiiPhysics style grouping). */
USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaPhysicsSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Stiffness = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendStiffness = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.01"))
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Radius = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float MaxVelocity = 5000.0f;
};

/** Root bone setting for multi-chain (KawaiiPhysics AdditionalRootBones pattern). */
USTRUCT(BlueprintType)
struct FMagicaRootBoneSetting
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Bones")
	FBoneReference RootBone;

	UPROPERTY(EditAnywhere, Category = "Bones", meta = (InlineEditConditionToggle))
	bool bUseOverrideExcludeBones = false;

	UPROPERTY(EditAnywhere, Category = "Bones", meta = (EditCondition = "bUseOverrideExcludeBones"))
	TArray<FBoneReference> OverrideExcludeBones;
};

/** Inertia parameters. */
USTRUCT(BlueprintType)
struct FMagicaInertiaParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WorldInertia = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float LocalInertia = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float TeleportDistanceThreshold = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float TeleportRotationThreshold = 10.0f;
};

/** Wind parameters. */
USTRUCT(BlueprintType)
struct FMagicaWindParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Influence = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Frequency = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Turbulence = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float DepthWeight = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float MovingWind = 0.0f;
};

/** Unique identifier for a cloth Team. */
using FMagicaTeamId = int32;
constexpr FMagicaTeamId MAGICA_INVALID_TEAM_ID = INDEX_NONE;
