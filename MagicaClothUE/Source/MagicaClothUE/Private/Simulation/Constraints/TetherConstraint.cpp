// MagicaClothUE/Private/Simulation/Constraints/TetherConstraint.cpp
#include "Simulation/Constraints/TetherConstraint.h"
#include "VirtualMesh/VertexAttribute.h"

void FMagicaTetherConstraint::Build(
	const TArray<FVector>& Positions,
	const TArray<int32>& ParentIndices,
	const TArray<FMagicaVertexAttribute>& Attributes,
	float InTetherScale)
{
	TetherScale = InTetherScale;
	Pairs.Reset();

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		if (Attributes[i].IsFixed()) continue;

		// Walk up to find nearest fixed ancestor
		int32 RootIdx = ParentIndices[i];
		float AccumDist = FVector::Dist(Positions[i], Positions[RootIdx]);

		while (RootIdx != INDEX_NONE && !Attributes[RootIdx].IsFixed())
		{
			const int32 Parent = ParentIndices[RootIdx];
			if (Parent == INDEX_NONE) break;
			AccumDist += FVector::Dist(Positions[RootIdx], Positions[Parent]);
			RootIdx = Parent;
		}

		if (RootIdx != INDEX_NONE)
		{
			FMagicaTetherPair Pair;
			Pair.ParticleIndex = i;
			Pair.RootIndex = RootIdx;
			Pair.MaxDistance = AccumDist * TetherScale;
			Pairs.Add(Pair);
		}
	}
}

void FMagicaTetherConstraint::Solve(FMagicaParticleArrays& Particles)
{
	for (const FMagicaTetherPair& Pair : Pairs)
	{
		FVector& Pos = Particles.PredictedPositions[Pair.ParticleIndex];
		const FVector& RootPos = Particles.PredictedPositions[Pair.RootIndex];

		const FVector Delta = Pos - RootPos;
		const float Dist = Delta.Size();

		if (Dist > Pair.MaxDistance && Dist > UE_SMALL_NUMBER)
		{
			Pos = RootPos + (Delta / Dist) * Pair.MaxDistance;
		}
	}
}
