// MagicaClothUE/Private/Simulation/Constraints/AngleConstraint.cpp
#include "Simulation/Constraints/AngleConstraint.h"

void FMagicaAngleConstraint::Build(
	const TArray<FVector>& Positions,
	const TArray<int32>& ParentIndices,
	const TArray<FQuat>& RestRotations,
	const TArray<FVector>& RestOffsets,
	float InStiffness)
{
	Stiffness = InStiffness;
	Pairs.Reset();

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		if (ParentIndices[i] == INDEX_NONE) continue;

		FMagicaAnglePair Pair;
		Pair.ParentIndex = ParentIndices[i];
		Pair.ChildIndex = i;
		Pair.RestLocalRotation = RestRotations[i];
		Pair.RestLocalOffset = RestOffsets[i];
		Pairs.Add(Pair);
	}
}

void FMagicaAngleConstraint::Solve(FMagicaParticleArrays& Particles)
{
	for (const FMagicaAnglePair& Pair : Pairs)
	{
		if (Particles.InvMasses[Pair.ChildIndex] <= 0.0f) continue;

		const FVector& ParentPos = Particles.PredictedPositions[Pair.ParentIndex];
		FVector& ChildPos = Particles.PredictedPositions[Pair.ChildIndex];

		// Current direction from parent to child
		const FVector CurrentDir = ChildPos - ParentPos;
		const float CurrentLen = CurrentDir.Size();
		if (CurrentLen < UE_SMALL_NUMBER) continue;

		// Target position based on rest rotation
		// Reconstruct parent rotation from its direction
		const FVector RestDir = Pair.RestLocalOffset;
		const float RestLen = RestDir.Size();
		if (RestLen < UE_SMALL_NUMBER) continue;

		// Target: parent position + rest offset direction scaled to current length
		const FVector TargetPos = ParentPos + (CurrentDir / CurrentLen) * CurrentLen;

		// Blend toward rest-pose direction
		const FVector RestWorldDir = Pair.RestLocalOffset.GetSafeNormal();
		const FVector CurWorldDir = CurrentDir / CurrentLen;

		// Slerp current direction toward rest direction
		const FQuat CurRot = FQuat::FindBetweenNormals(FVector::ForwardVector, CurWorldDir);
		const FQuat RestRot = FQuat::FindBetweenNormals(FVector::ForwardVector, RestWorldDir);
		const FQuat BlendedRot = FQuat::Slerp(CurRot, RestRot, Stiffness);
		const FVector BlendedDir = BlendedRot.RotateVector(FVector::ForwardVector);

		ChildPos = ParentPos + BlendedDir * CurrentLen;
	}
}
