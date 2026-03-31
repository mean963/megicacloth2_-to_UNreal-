#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "Math/Vector.h"
#include "Math/UnrealMathUtility.h"
#include "Templates/SharedPointer.h"

struct FPBDParticle;

/**
 * Base class for all PBD constraints.
 * Constraints project particle positions to satisfy geometric conditions.
 */
struct MAGICACLOTHUE_API FClothConstraintBase
{
	virtual ~FClothConstraintBase() = default;
	virtual void Solve(TArray<FPBDParticle>& Particles) = 0;
};

/**
 * Structural Constraint: maintains rest-length distance between two connected particles.
 * Used for every consecutive pair in a bone chain.
 */
struct MAGICACLOTHUE_API FStructuralConstraint : public FClothConstraintBase
{
	int32 ParticleA  = INDEX_NONE;
	int32 ParticleB  = INDEX_NONE;
	float RestLength = 0.f;
	float Stiffness  = 0.8f; // 0..1

	FStructuralConstraint() = default;

	FStructuralConstraint(int32 InA, int32 InB, float InRestLength, float InStiffness)
		: ParticleA(InA)
		, ParticleB(InB)
		, RestLength(InRestLength)
		, Stiffness(FMath::Clamp(InStiffness, 0.f, 1.f))
	{}

	virtual void Solve(TArray<FPBDParticle>& Particles) override;
};

/**
 * Bend Constraint: resists bending between three consecutive particles.
 * Maintains the rest angle at the middle joint.
 */
struct MAGICACLOTHUE_API FBendConstraint : public FClothConstraintBase
{
	int32 ParticleA = INDEX_NONE; // parent
	int32 ParticleB = INDEX_NONE; // middle (pivot)
	int32 ParticleC = INDEX_NONE; // child
	float RestAngle = 0.f;
	float Stiffness = 0.3f; // 0..1

	FBendConstraint() = default;

	FBendConstraint(int32 InA, int32 InB, int32 InC, float InRestAngle, float InStiffness)
		: ParticleA(InA)
		, ParticleB(InB)
		, ParticleC(InC)
		, RestAngle(InRestAngle)
		, Stiffness(FMath::Clamp(InStiffness, 0.f, 1.f))
	{}

	virtual void Solve(TArray<FPBDParticle>& Particles) override;
};

/**
 * Self-Collision Constraint: prevents non-adjacent particles from overlapping.
 * O(n^2) but acceptable for typical bone chain lengths (< 30 bones).
 */
struct MAGICACLOTHUE_API FSelfCollisionConstraint : public FClothConstraintBase
{
	float CollisionRadius = 2.f;

	explicit FSelfCollisionConstraint(float InRadius = 2.f)
		: CollisionRadius(InRadius)
	{}

	virtual void Solve(TArray<FPBDParticle>& Particles) override;
};

struct FChainRange;

/** Factory: builds constraints for single-chain and multi-chain modes. */
namespace FClothConstraintFactory
{
	/** Build structural + bend constraints for a single linear chain. */
	MAGICACLOTHUE_API void BuildChainConstraints(
		const TArray<FPBDParticle>& Particles,
		float StructuralStiffness,
		float BendStiffness,
		TArray<TSharedPtr<FClothConstraintBase>>& OutConstraints
	);

	/** Build structural + bend constraints for multiple chains sharing a root.
	 *  Each chain gets its own constraints. No cross-chain bend constraints.
	 *  Shared root particle (index 0) is connected to each chain's first particle. */
	MAGICACLOTHUE_API void BuildMultiChainConstraints(
		const TArray<FPBDParticle>& Particles,
		int32 SharedRootCount,
		const TArray<FChainRange>& ChainRanges,
		float StructuralStiffness,
		float BendStiffness,
		TArray<TSharedPtr<FClothConstraintBase>>& OutConstraints
	);

	/** Build cross-chain "mesh net" constraints between adjacent chains.
	 *  Creates a net-like topology (horizontal ring + diagonal shear).
	 *  bClosedLoop = true wraps last chain to first chain (for skirts).
	 *
	 *  Topology created (example: chain A and adjacent chain B):
	 *    A0──B0     ← Horizontal (ring) constraints
	 *    |╲╱|
	 *    A1──B1     ← Diagonal (shear) constraints
	 *    |╲╱|
	 *    A2──B2
	 */
	MAGICACLOTHUE_API void BuildCrossChainMeshConstraints(
		const TArray<FPBDParticle>& Particles,
		const TArray<FChainRange>& ChainRanges,
		float HorizontalStiffness,
		float ShearStiffness,
		bool bClosedLoop,
		TArray<TSharedPtr<FClothConstraintBase>>& OutConstraints
	);
}
