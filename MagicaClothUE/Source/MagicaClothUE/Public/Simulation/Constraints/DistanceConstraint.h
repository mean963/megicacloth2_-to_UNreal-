// MagicaClothUE/Public/Simulation/Constraints/DistanceConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"
#include "Core/ClothTypes.h"

/** Type of distance constraint (MagicaCloth2 separation). */
enum class EMagicaDistanceType : uint8
{
	Vertical,    // Parent-child (structural)
	Horizontal,  // Same-depth neighbors (shear/ring)
	Diagonal,    // Cross-chain shear
};

/** Single distance constraint between two particles. */
struct FMagicaDistancePair
{
	int32 IndexA = INDEX_NONE;
	int32 IndexB = INDEX_NONE;
	float RestLength = 0.0f;
};

/**
 * Distance Constraint: maintains rest-length between particle pairs.
 * Supports depth-based stiffness via external curve sampling.
 */
struct MAGICACLOTHUE_API FMagicaDistanceConstraint : public FMagicaConstraintBase
{
	EMagicaDistanceType Type = EMagicaDistanceType::Vertical;
	float Stiffness = 0.8f;
	TArray<FMagicaDistancePair> Pairs;

	/** Build from VirtualMesh parent-child relationships (Vertical). */
	void BuildVertical(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices, float InStiffness);

	/** Build horizontal (ring) constraints between same-depth particles across chains. */
	void BuildHorizontal(
		const TArray<FVector>& Positions,
		const TArray<FMagicaChainRange>& ChainRanges,
		int32 SharedRootCount,
		float InStiffness,
		bool bClosedLoop);

	/** Build diagonal (shear) constraints between adjacent chains. */
	void BuildDiagonal(
		const TArray<FVector>& Positions,
		const TArray<FMagicaChainRange>& ChainRanges,
		int32 SharedRootCount,
		float InStiffness,
		bool bClosedLoop);

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
