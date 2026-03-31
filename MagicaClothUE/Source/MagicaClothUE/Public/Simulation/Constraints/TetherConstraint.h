// MagicaClothUE/Public/Simulation/Constraints/TetherConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"
#include "VirtualMesh/VertexAttribute.h"

/** Tether pair: particle tethered to a root particle with max distance. */
struct FMagicaTetherPair
{
	int32 ParticleIndex = INDEX_NONE;
	int32 RootIndex = INDEX_NONE;
	float MaxDistance = 0.0f;
};

/**
 * Tether Constraint: limits maximum distance from root.
 * Prevents cloth from stretching too far.
 */
struct MAGICACLOTHUE_API FMagicaTetherConstraint : public FMagicaConstraintBase
{
	float TetherScale = 1.1f;  // MaxDistance = restDistance * TetherScale
	TArray<FMagicaTetherPair> Pairs;

	/** Build from VirtualMesh. Each non-fixed particle tethered to nearest fixed ancestor. */
	void Build(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices,
		const TArray<FMagicaVertexAttribute>& Attributes, float InTetherScale);

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
