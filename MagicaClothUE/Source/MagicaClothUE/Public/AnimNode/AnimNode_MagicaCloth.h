#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "Math/Transform.h"
#include "Math/Vector.h"
#include "Curves/CurveFloat.h"
#include "GameplayTagContainer.h"
#include "BoneContainer.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "Core/ClothTypes.h"
#include "Colliders/ColliderLimits.h"
#include "Simulation/SimulationManager.h"
#include "VirtualMesh/VirtualMesh.h"
#include "AnimNode_MagicaCloth.generated.h"

class UPhysicsAsset;
class UMagicaLimitsDataAsset;
class UMagicaClothSubsystem;
class FMagicaVirtualMesh;

/**
 * AnimBP runtime node: Magica Cloth Simulation (KawaiiPhysics-style parameters).
 *
 * Supports:
 * - Single bone chain (hair, cape, tail)
 * - Multi-chain / skirt mode (all children of root simulated independently)
 * - Three collision sources: inline limits, DataAsset, PhysicsAsset
 * - Depth-based parameter curves for per-bone stiffness/damping/mass/radius
 * - Subsystem-managed simulation (centralized worker thread)
 */
USTRUCT(BlueprintInternalUseOnly)
struct MAGICACLOTHUE_API FAnimNode_MagicaCloth : public FAnimNode_SkeletalControlBase
{
	GENERATED_BODY()

	// =====================================================================
	// Bones
	// =====================================================================

	/** Cloth simulation type. */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault))
	EMagicaClothType ClothType = EMagicaClothType::BoneCloth;

	/** Root bone of the cloth chain. */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault))
	FBoneReference RootBone;

	/** End bone (single-chain only). If unset, the chain walks to the leaf. */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault, EditCondition = "!bMultiChainMode"))
	FBoneReference EndBone;

	/** Bones to EXCLUDE from multi-chain simulation.
	 *  Chains starting from these bones will not be simulated. */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault, EditCondition = "bMultiChainMode"))
	TArray<FBoneReference> ExcludeBones;

	/** Additional root bones for multi-chain setups (KawaiiPhysics pattern). */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault))
	TArray<FMagicaRootBoneSetting> AdditionalRootBones;

	/** Length of dummy bone appended at chain tips (0 = disabled). */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault, ClampMin = "0.0"))
	float DummyBoneLength = 0.0f;

	/** Enable multi-chain (skirt) mode: simulate ALL child branches from root. */
	UPROPERTY(EditAnywhere, Category = "Bones", meta = (PinHiddenByDefault))
	bool bMultiChainMode = false;

	// =====================================================================
	// Physics Settings
	// =====================================================================

	/** Physics parameter bundle (stiffness, bend, damping, mass, radius, maxVelocity). */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault))
	FMagicaPhysicsSettings PhysicsSettings;

	/** Simulation coordinate space. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault))
	EMagicaSimulationSpace SimulationSpace = EMagicaSimulationSpace::ComponentSpace;

	/** Target simulation framerate (Hz). */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, ClampMin = "30", ClampMax = "120"))
	int32 TargetFramerate = 90;

	/** PBD solver iteration count per sub-step. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, ClampMin = "1", ClampMax = "16"))
	int32 SolverIterations = 5;

	/** Maximum simulation sub-steps per frame (prevents spiral of death). */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, ClampMin = "1", ClampMax = "8"))
	int32 MaxStepsPerFrame = 3;

	/** Scale applied to skeletal component movement before feeding to simulation. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault))
	FVector SkelCompMoveScale = FVector(1.0, 1.0, 1.0);

	// =====================================================================
	// Physics Settings (Advanced - Curves)
	// =====================================================================

	/** Stiffness curve along chain depth (0=root, 1=tip). */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, AdvancedDisplay))
	FRuntimeFloatCurve StiffnessCurve;

	/** Bend stiffness curve along chain depth. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, AdvancedDisplay))
	FRuntimeFloatCurve BendStiffnessCurve;

	/** Damping curve along chain depth. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, AdvancedDisplay))
	FRuntimeFloatCurve DampingCurve;

	/** Mass curve along chain depth. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, AdvancedDisplay))
	FRuntimeFloatCurve MassCurve;

	/** Particle radius curve along chain depth. */
	UPROPERTY(EditAnywhere, Category = "Physics Settings", meta = (PinHiddenByDefault, AdvancedDisplay))
	FRuntimeFloatCurve RadiusCurve;

	// =====================================================================
	// Limits (Colliders)
	// =====================================================================

	/** Inline spherical collision limits. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TArray<FMagicaSphericalLimit> SphericalLimits;

	/** Inline capsule collision limits. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TArray<FMagicaCapsuleLimit> CapsuleLimits;

	/** Inline box collision limits. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TArray<FMagicaBoxLimit> BoxLimits;

	/** Inline planar collision limits. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TArray<FMagicaPlanarLimit> PlanarLimits;

	/** Reusable collision limits from a DataAsset. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TObjectPtr<UMagicaLimitsDataAsset> LimitsDataAsset;

	/** Physics Asset to extract colliders from (sphere/capsule bodies). */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TObjectPtr<UPhysicsAsset> PhysicsAssetForLimits;

	/** Only include PA bodies attached to these bones. If empty, ALL PA bodies are used. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	TArray<FBoneReference> PhysicsAssetBoneFilter;

	/** Friction applied during collision response. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault, ClampMin = "0.0", ClampMax = "1.0"))
	float ColliderFriction = 0.1f;

	/** Enable self-collision between particles. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault))
	bool bEnableSelfCollision = false;

	/** Self-collision particle radius. */
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (PinHiddenByDefault, ClampMin = "0.1", EditCondition = "bEnableSelfCollision"))
	float SelfCollisionRadius = 2.0f;

	// =====================================================================
	// Mesh Net (multi-chain)
	// =====================================================================

	/** Enable mesh-net constraints between adjacent chains (horizontal ring + diagonal shear). */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (PinHiddenByDefault, EditCondition = "bMultiChainMode"))
	bool bEnableMeshNet = true;

	/** Stiffness of horizontal (ring) constraints connecting same-depth particles across chains. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (PinHiddenByDefault, ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bMultiChainMode && bEnableMeshNet"))
	float HorizontalStiffness = 0.5f;

	/** Stiffness of diagonal (shear) constraints for cross-chain rigidity. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (PinHiddenByDefault, ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bMultiChainMode && bEnableMeshNet"))
	float ShearStiffness = 0.3f;

	/** Wrap last chain to first chain (closed loop for skirts). Disable for open cloth like capes. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (PinHiddenByDefault, EditCondition = "bMultiChainMode && bEnableMeshNet"))
	bool bClosedLoop = true;

	// =====================================================================
	// Inertia
	// =====================================================================

	/** Inertia parameters (world/local damping, teleport thresholds). */
	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (PinHiddenByDefault))
	FMagicaInertiaParams InertiaParams;

	/** Inertia strength curve along chain depth. */
	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (PinHiddenByDefault, AdvancedDisplay))
	FRuntimeFloatCurve InertiaCurve;

	// =====================================================================
	// External Force
	// =====================================================================

	/** Gravity vector. */
	UPROPERTY(EditAnywhere, Category = "External Force", meta = (PinHiddenByDefault))
	FVector Gravity = FVector(0.0, 0.0, -98.0);

	/** Enable wind influence from UE Wind Directional Source. */
	UPROPERTY(EditAnywhere, Category = "External Force", meta = (PinHiddenByDefault))
	bool bEnableWind = false;

	/** Multiplier for wind strength. */
	UPROPERTY(EditAnywhere, Category = "External Force", meta = (PinHiddenByDefault, ClampMin = "0.0", EditCondition = "bEnableWind"))
	float WindScale = 1.0f;

	// =====================================================================
	// Debug
	// =====================================================================

	/** Draw debug lines showing particles, constraints, and colliders in the viewport. */
	UPROPERTY(EditAnywhere, Category = "Debug", meta = (PinHiddenByDefault))
	bool bShowDebug = false;

	/** Size of debug particle spheres. */
	UPROPERTY(EditAnywhere, Category = "Debug", meta = (PinHiddenByDefault, ClampMin = "0.1", ClampMax = "10.0", EditCondition = "bShowDebug"))
	float DebugParticleSize = 1.5f;

	// =====================================================================
	// Tag
	// =====================================================================

	/** Gameplay tag for identifying this cloth instance (for external queries). */
	UPROPERTY(EditAnywhere, Category = "Tag", meta = (PinHiddenByDefault))
	FGameplayTag MagicaClothTag;

	// =====================================================================
	// FAnimNode_SkeletalControlBase interface
	// =====================================================================

	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual void UpdateInternal(const FAnimationUpdateContext& Context) override;
	virtual void EvaluateSkeletalControl_AnyThread(
		FComponentSpacePoseContext& Output,
		TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;

private:
	// ── Subsystem integration ──────────────────────────────
	FMagicaTeamId TeamId = MAGICA_INVALID_TEAM_ID;
	FMagicaVirtualMesh* VirtualMeshPtr = nullptr;  // Owned by this node
	TSharedPtr<FMagicaVirtualMesh> VirtualMeshOwned;

	// ── Single-chain data ──────────────────────────────────
	TArray<FCompactPoseBoneIndex> BoneChainIndices;
	TArray<int32> BoneChainSkeletonIndices;

	// ── Multi-chain (skirt) data ───────────────────────────
	// Flat arrays: [0] = root, then chain0 bones, chain1 bones, ...
	TArray<FCompactPoseBoneIndex> AllBoneCompactIndices;
	TArray<int32> AllBoneSkeletonIndices;
	int32 TotalParticleCount = 0;

	// ── Collider bone mappings ─────────────────────────────
	struct FColliderBoneMapping
	{
		FCompactPoseBoneIndex CompactBoneIndex = FCompactPoseBoneIndex(INDEX_NONE);
		int32 ColliderIndex = INDEX_NONE;
		FTransform LocalOffset = FTransform::Identity;
	};
	TArray<FColliderBoneMapping> ColliderBoneMappings;

	// ── State ──────────────────────────────────────────────
	TArray<FTransform> PrevResults;
	bool bInitialized  = false;
	bool bNeedsRebuild = true;

	// Cached subsystem pointer (refreshed each frame)
	TWeakObjectPtr<UMagicaClothSubsystem> CachedSubsystem;

	// ── Methods ────────────────────────────────────────────
	void BuildBoneChain(const FBoneContainer& RequiredBones);
	void BuildMultiChain(const FBoneContainer& RequiredBones);
	void InitializeSimulation(FComponentSpacePoseContext& Context);
	void ShutdownSimulation();
	void DrawDebug(FComponentSpacePoseContext& Output) const;

	// Collider building (3 sources)
	void BuildInlineLimitColliders(FComponentSpacePoseContext& Context, FMagicaSimulationManager::FTeamData& Team);
	void BuildDataAssetColliders(FComponentSpacePoseContext& Context, FMagicaSimulationManager::FTeamData& Team);
	void BuildPhysicsAssetColliders(FComponentSpacePoseContext& Context, FMagicaSimulationManager::FTeamData& Team);
	void UpdateColliderTransformsFromPose(FComponentSpacePoseContext& Output);

	// Constraint building
	void SetupConstraints(FMagicaSimulationManager::FTeamData& Team);

	// Helper: get the subsystem from world
	UMagicaClothSubsystem* GetSubsystem(FComponentSpacePoseContext& Context) const;
};
