// MagicaClothUE/Public/Simulation/Constraints/BendingConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"
#include "Core/ClothTypes.h"

/** Triangle pair for dihedral angle bending (MagicaCloth2 DirectionDihedralAngle). */
struct FMagicaBendPair
{
	int32 V0 = INDEX_NONE;  // Shared edge vertex 0
	int32 V1 = INDEX_NONE;  // Shared edge vertex 1
	int32 V2 = INDEX_NONE;  // Triangle A opposite vertex
	int32 V3 = INDEX_NONE;  // Triangle B opposite vertex
	float RestAngle = 0.0f; // Rest dihedral angle (radians)
};

/**
 * Bending Constraint using Dihedral Angle method.
 *
 * For BoneCloth (chain mode): uses 3-consecutive-bone angle preservation.
 * For MeshCloth: uses triangle-pair dihedral angles (Phase 2).
 */
struct MAGICACLOTHUE_API FMagicaBendingConstraint : public FMagicaConstraintBase
{
	float Stiffness = 0.5f;
	TArray<FMagicaBendPair> Pairs;

	/** Build from bone chain: creates bend constraints for every 3 consecutive bones. */
	void BuildFromChain(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices, float InStiffness);

	/** Build from multi-chain: per-chain 3-bone bends. */
	void BuildFromMultiChain(
		const TArray<FVector>& Positions,
		const TArray<FMagicaChainRange>& ChainRanges,
		int32 SharedRootCount,
		float InStiffness);

	virtual void Solve(FMagicaParticleArrays& Particles) override;

private:
	/** Calculate dihedral angle between two triangles sharing edge (V0, V1). */
	static float CalcDihedralAngle(const FVector& V0, const FVector& V1, const FVector& V2, const FVector& V3);
};
