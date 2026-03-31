#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "Math/Transform.h"
#include "Math/Vector.h"
#include "Templates/SharedPointer.h"
#include "BoneContainer.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_MagicaCloth.generated.h"

class FClothSimThread;
class UPhysicsAsset;

/**
 * AnimBP runtime node: Magica Cloth Simulation.
 *
 * Supports:
 * - Single bone chain (hair, cape, tail)
 * - Multi-chain / skirt mode (all children of root simulated independently)
 * - Physics Asset colliders (auto-read sphere/capsule bodies from PA)
 */
USTRUCT(BlueprintInternalUseOnly)
struct MAGICACLOTHUE_API FAnimNode_MagicaCloth : public FAnimNode_SkeletalControlBase
{
	GENERATED_BODY()

	// ── Bone Chain Setup ────────────────────────────────

	UPROPERTY(EditAnywhere, Category = "Chain")
	FBoneReference RootBone;

	/** End bone (single-chain only. Ignored in multi-chain mode). */
	UPROPERTY(EditAnywhere, Category = "Chain", meta = (EditCondition = "!bMultiChainMode"))
	FBoneReference EndBone;

	UPROPERTY(EditAnywhere, Category = "Chain", meta = (ClampMin = "1"))
	int32 FixedBoneCount = 1;

	/** Enable multi-chain (skirt) mode: simulate ALL child branches from root. */
	UPROPERTY(EditAnywhere, Category = "Chain")
	bool bMultiChainMode = false;

	// ── Simulation Parameters ───────────────────────────

	UPROPERTY(EditAnywhere, Category = "Physics", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Stiffness = 0.8f;

	UPROPERTY(EditAnywhere, Category = "Physics", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendStiffness = 0.3f;

	UPROPERTY(EditAnywhere, Category = "Physics", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Physics")
	FVector Gravity = FVector(0.0, 0.0, -980.0);

	UPROPERTY(EditAnywhere, Category = "Physics", meta = (ClampMin = "0.0"))
	float MaxVelocity = 5000.f;

	// ── Performance ─────────────────────────────────────

	UPROPERTY(EditAnywhere, Category = "Performance", meta = (ClampMin = "30.0", ClampMax = "120.0"))
	float SimulationHz = 90.f;

	UPROPERTY(EditAnywhere, Category = "Performance", meta = (ClampMin = "1", ClampMax = "8"))
	int32 SolverIterations = 4;

	// ── Collision ───────────────────────────────────────

	/** Use Physics Asset bodies (Sphere/Capsule) as colliders. */
	UPROPERTY(EditAnywhere, Category = "Collision")
	bool bUsePhysicsAssetColliders = true;

	/** Optional: override physics asset (if null, uses SkeletalMesh's default PA). */
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (EditCondition = "bUsePhysicsAssetColliders"))
	TObjectPtr<UPhysicsAsset> PhysicsAssetOverride;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ColliderFriction = 0.1f;

	/** Only include PA bodies attached to these bones. If empty, ALL PA bodies are used (NOT recommended).
	 *  Example: add Hips, UpperLeg_L, UpperLeg_R, LowerLeg_L, LowerLeg_R for a skirt. */
	UPROPERTY(EditAnywhere, Category = "Collision", meta = (EditCondition = "bUsePhysicsAssetColliders"))
	TArray<FBoneReference> ColliderBoneFilter;

	UPROPERTY(EditAnywhere, Category = "Collision")
	bool bEnableSelfCollision = false;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "0.1", EditCondition = "bEnableSelfCollision"))
	float SelfCollisionRadius = 2.f;

	// ── Cross-Chain Mesh Net (Skirt) ────────────────────

	/** Enable mesh-net constraints between adjacent chains (horizontal ring + diagonal shear).
	 *  This makes the skirt behave like a connected cloth mesh instead of independent chains. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (EditCondition = "bMultiChainMode"))
	bool bEnableMeshNet = true;

	/** Stiffness of horizontal (ring) constraints connecting same-depth particles across chains. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bMultiChainMode && bEnableMeshNet"))
	float HorizontalStiffness = 0.5f;

	/** Stiffness of diagonal (shear) constraints for cross-chain rigidity. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bMultiChainMode && bEnableMeshNet"))
	float ShearStiffness = 0.3f;

	/** Wrap last chain to first chain (closed loop for skirts). Disable for open cloth like capes. */
	UPROPERTY(EditAnywhere, Category = "Mesh Net", meta = (EditCondition = "bMultiChainMode && bEnableMeshNet"))
	bool bClosedLoop = true;

	/** Bones to EXCLUDE from multi-chain simulation.
	 *  Chains starting from these bones will not be simulated (e.g. accessory bones, ribbons).
	 *  Add the direct child bone of root that starts the unwanted chain. */
	UPROPERTY(EditAnywhere, Category = "Chain", meta = (EditCondition = "bMultiChainMode"))
	TArray<FBoneReference> ExcludeChainBones;

	// ── Debug ───────────────────────────────────────────

	/** Draw debug lines showing particles, constraints, and colliders in the viewport. */
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bShowDebug = false;

	/** Size of debug particle spheres. */
	UPROPERTY(EditAnywhere, Category = "Debug", meta = (ClampMin = "0.1", ClampMax = "10.0", EditCondition = "bShowDebug"))
	float DebugParticleSize = 1.5f;

	// ── FAnimNode_SkeletalControlBase interface ─────────

	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual void UpdateInternal(const FAnimationUpdateContext& Context) override;
	virtual void EvaluateSkeletalControl_AnyThread(
		FComponentSpacePoseContext& Output,
		TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;

private:
	TSharedPtr<FClothSimThread> SimThread;

	// ── Single-chain data ───────────────────────────────
	TArray<FCompactPoseBoneIndex> BoneChainIndices;
	TArray<int32> BoneChainSkeletonIndices;

	// ── Multi-chain (skirt) data ────────────────────────
	// Flat arrays: [0] = root, then chain0 bones, chain1 bones, ...
	TArray<FCompactPoseBoneIndex> AllBoneCompactIndices;
	TArray<int32> AllBoneSkeletonIndices;
	int32 TotalParticleCount = 0;

	// ── Physics Asset collider mappings ─────────────────
	struct FColliderBoneMapping
	{
		FCompactPoseBoneIndex CompactBoneIndex = FCompactPoseBoneIndex(INDEX_NONE);
		int32 ColliderIndex = INDEX_NONE;
		FTransform LocalOffset = FTransform::Identity; // shape offset relative to bone
	};
	TArray<FColliderBoneMapping> ColliderBoneMappings;

	// ── State ───────────────────────────────────────────
	TArray<FTransform> PrevResults;
	bool bInitialized  = false;
	bool bNeedsRebuild = true;

	// ── Methods ─────────────────────────────────────────
	void BuildBoneChain(const FBoneContainer& RequiredBones);
	void BuildMultiChain(const FBoneContainer& RequiredBones);
	void InitializeSimulation(FComponentSpacePoseContext& Context);
	void ShutdownSimulation();
	void DrawDebug(FComponentSpacePoseContext& Output) const;

	void BuildPhysicsAssetColliders(FComponentSpacePoseContext& Context);
	void UpdateColliderTransformsFromPose(FComponentSpacePoseContext& Output);
};
