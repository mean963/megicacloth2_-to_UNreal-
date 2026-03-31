#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Math/Transform.h"
#include "Templates/SharedPointer.h"

struct FClothConstraintBase;
struct FMagicaColliderShape;

/**
 * Single particle in the PBD simulation.
 * Each particle maps 1:1 to a bone.
 */
struct FPBDParticle
{
	FVector Position          = FVector::ZeroVector;
	FVector PrevPosition      = FVector::ZeroVector;
	FVector Velocity          = FVector::ZeroVector;
	FVector PredictedPosition = FVector::ZeroVector;
	float InvMass             = 1.0f;   // 0 = pinned (fixed), > 0 = free
	int32 BoneIndex           = INDEX_NONE;
	int32 ChainId             = INDEX_NONE; // which chain this particle belongs to (-1 = shared root)
};

/** Per-chain range within the flat particle array. */
struct FChainRange
{
	int32 StartIndex  = 0;  // first particle index of this chain (excluding shared root)
	int32 Count       = 0;  // number of particles in this chain
};

/**
 * Position Based Dynamics solver.
 * Supports single-chain AND multi-chain (skirt) modes.
 */
class MAGICACLOTHUE_API FPBDSolver
{
public:
	FPBDSolver();
	~FPBDSolver();

	/** Initialize from a single bone chain. */
	void Initialize(const TArray<FTransform>& BoneTransforms, const TArray<int32>& BoneIndices, int32 FixedCount);

	/** Initialize from multiple chains sharing a common root.
	 *  AllTransforms[0..SharedRootCount-1] = shared root bones (pinned).
	 *  Followed by concatenated chain bones.
	 *  InChainRanges describes where each chain starts/ends in the particle array. */
	void InitializeMultiChain(
		const TArray<FTransform>& AllTransforms,
		const TArray<int32>& AllBoneIndices,
		int32 InSharedRootCount,
		const TArray<FChainRange>& InChainRanges);

	/** Main simulation step. */
	void Step(float DeltaTime);

	/** Update root/fixed bone transforms from animation. */
	void UpdateFixedParticles(const TArray<FTransform>& AnimTransforms);

	/** Get current particle transforms as bone transforms. */
	void GetResultTransforms(TArray<FTransform>& OutTransforms) const;

	/** Reset to rest pose. */
	void ResetToRestPose(const TArray<FTransform>& RestTransforms);

	// --- Simulation Parameters ---
	FVector Gravity        = FVector(0.0, 0.0, -980.0);
	float Damping          = 0.1f;
	float MaxVelocity      = 5000.f;
	int32 SolverIterations = 4;

	// --- Constraints ---
	TArray<TSharedPtr<FClothConstraintBase>> Constraints;

	// --- Colliders ---
	TArray<TSharedPtr<FMagicaColliderShape>> Colliders;

	// --- Particles ---
	TArray<FPBDParticle> Particles;

	// --- Multi-chain info ---
	TArray<FChainRange> ChainRanges;
	int32 SharedRootCount = 0;

	// Rest-pose local offsets for rotation reconstruction
	TArray<FVector> RestLocalOffsets;
	TArray<FQuat>   RestLocalRotations;

private:
	void PredictPositions(float DeltaTime);
	void SolveConstraints();
	void SolveCollisions();
	void UpdateVelocities(float DeltaTime);
	void ClampVelocities();

	/** Get result transforms for multi-chain mode (per-chain rotation reconstruction). */
	void GetResultTransformsMultiChain(TArray<FTransform>& OutTransforms) const;

	float InvDt = 0.f;
};
