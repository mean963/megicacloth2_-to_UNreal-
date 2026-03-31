// MagicaClothUE/Public/Simulation/Constraints/AngleConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"

/** Angle constraint pair: maintains rotation at a joint. */
struct FMagicaAnglePair
{
	int32 ParentIndex = INDEX_NONE;
	int32 ChildIndex = INDEX_NONE;
	FQuat RestLocalRotation = FQuat::Identity;
	FVector RestLocalOffset = FVector::ZeroVector;
};

/**
 * Angle Constraint (BoneCloth only): preserves bone joint angles.
 * Restores each bone toward its rest-pose rotation relative to parent.
 */
struct MAGICACLOTHUE_API FMagicaAngleConstraint : public FMagicaConstraintBase
{
	float Stiffness = 0.3f;
	TArray<FMagicaAnglePair> Pairs;

	void Build(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices,
		const TArray<FQuat>& RestRotations, const TArray<FVector>& RestOffsets, float InStiffness);

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
