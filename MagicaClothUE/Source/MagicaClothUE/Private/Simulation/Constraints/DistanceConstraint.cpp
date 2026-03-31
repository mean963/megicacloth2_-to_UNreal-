// MagicaClothUE/Private/Simulation/Constraints/DistanceConstraint.cpp
#include "Simulation/Constraints/DistanceConstraint.h"
#include "Core/ClothTypes.h"

void FMagicaDistanceConstraint::BuildVertical(
	const TArray<FVector>& Positions,
	const TArray<int32>& ParentIndices,
	float InStiffness)
{
	Type = EMagicaDistanceType::Vertical;
	Stiffness = InStiffness;
	Pairs.Reset();

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		if (ParentIndices[i] != INDEX_NONE)
		{
			FMagicaDistancePair Pair;
			Pair.IndexA = ParentIndices[i];
			Pair.IndexB = i;
			Pair.RestLength = FVector::Dist(Positions[i], Positions[ParentIndices[i]]);
			Pairs.Add(Pair);
		}
	}
}

void FMagicaDistanceConstraint::BuildHorizontal(
	const TArray<FVector>& Positions,
	const TArray<FMagicaChainRange>& ChainRanges,
	int32 SharedRootCount,
	float InStiffness,
	bool bClosedLoop)
{
	Type = EMagicaDistanceType::Horizontal;
	Stiffness = InStiffness;
	Pairs.Reset();

	const int32 NumChains = ChainRanges.Num();
	if (NumChains < 2) return;

	// Find max chain depth
	int32 MaxDepth = 0;
	for (const FMagicaChainRange& Range : ChainRanges)
	{
		MaxDepth = FMath::Max(MaxDepth, Range.Count);
	}

	// Connect same-depth particles between adjacent chains
	for (int32 Depth = 0; Depth < MaxDepth; ++Depth)
	{
		const int32 LoopEnd = bClosedLoop ? NumChains : (NumChains - 1);
		for (int32 c = 0; c < LoopEnd; ++c)
		{
			const int32 NextC = (c + 1) % NumChains;
			const FMagicaChainRange& CurChain = ChainRanges[c];
			const FMagicaChainRange& NextChain = ChainRanges[NextC];

			if (Depth < CurChain.Count && Depth < NextChain.Count)
			{
				FMagicaDistancePair Pair;
				Pair.IndexA = CurChain.StartIndex + Depth;
				Pair.IndexB = NextChain.StartIndex + Depth;
				Pair.RestLength = FVector::Dist(Positions[Pair.IndexA], Positions[Pair.IndexB]);
				Pairs.Add(Pair);
			}
		}
	}
}

void FMagicaDistanceConstraint::BuildDiagonal(
	const TArray<FVector>& Positions,
	const TArray<FMagicaChainRange>& ChainRanges,
	int32 SharedRootCount,
	float InStiffness,
	bool bClosedLoop)
{
	Type = EMagicaDistanceType::Diagonal;
	Stiffness = InStiffness;
	Pairs.Reset();

	const int32 NumChains = ChainRanges.Num();
	if (NumChains < 2) return;

	int32 MaxDepth = 0;
	for (const FMagicaChainRange& Range : ChainRanges)
	{
		MaxDepth = FMath::Max(MaxDepth, Range.Count);
	}

	// Diagonal: connect (chain[c], depth) to (chain[c+1], depth+1) and vice versa
	for (int32 Depth = 0; Depth < MaxDepth - 1; ++Depth)
	{
		const int32 LoopEnd = bClosedLoop ? NumChains : (NumChains - 1);
		for (int32 c = 0; c < LoopEnd; ++c)
		{
			const int32 NextC = (c + 1) % NumChains;
			const FMagicaChainRange& CurChain = ChainRanges[c];
			const FMagicaChainRange& NextChain = ChainRanges[NextC];

			// Forward diagonal: cur[depth] -> next[depth+1]
			if (Depth < CurChain.Count && (Depth + 1) < NextChain.Count)
			{
				FMagicaDistancePair Pair;
				Pair.IndexA = CurChain.StartIndex + Depth;
				Pair.IndexB = NextChain.StartIndex + Depth + 1;
				Pair.RestLength = FVector::Dist(Positions[Pair.IndexA], Positions[Pair.IndexB]);
				Pairs.Add(Pair);
			}

			// Backward diagonal: next[depth] -> cur[depth+1]
			if (Depth < NextChain.Count && (Depth + 1) < CurChain.Count)
			{
				FMagicaDistancePair Pair;
				Pair.IndexA = NextChain.StartIndex + Depth;
				Pair.IndexB = CurChain.StartIndex + Depth + 1;
				Pair.RestLength = FVector::Dist(Positions[Pair.IndexA], Positions[Pair.IndexB]);
				Pairs.Add(Pair);
			}
		}
	}
}

void FMagicaDistanceConstraint::Solve(FMagicaParticleArrays& Particles)
{
	for (const FMagicaDistancePair& Pair : Pairs)
	{
		FVector& PosA = Particles.PredictedPositions[Pair.IndexA];
		FVector& PosB = Particles.PredictedPositions[Pair.IndexB];
		const float InvMassA = Particles.InvMasses[Pair.IndexA];
		const float InvMassB = Particles.InvMasses[Pair.IndexB];

		const float WSum = InvMassA + InvMassB;
		if (WSum < UE_SMALL_NUMBER) continue;

		const FVector Delta = PosB - PosA;
		const float Dist = Delta.Size();
		if (Dist < UE_SMALL_NUMBER) continue;

		const FVector Dir = Delta / Dist;
		const float Error = Dist - Pair.RestLength;

		// Use average depth of the two particles to sample stiffness
		const float AvgDepth = (Particles.Depths[Pair.IndexA] + Particles.Depths[Pair.IndexB]) * 0.5f;
		const float EffectiveStiffness = Stiffness; // Depth curve applied externally by solver

		const FVector Correction = Dir * (Error * EffectiveStiffness / WSum);
		PosA += Correction * InvMassA;
		PosB -= Correction * InvMassB;
	}
}
