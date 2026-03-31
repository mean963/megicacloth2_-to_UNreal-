// MagicaClothUE/Private/Simulation/Constraints/BendingConstraint.cpp
#include "Simulation/Constraints/BendingConstraint.h"

float FMagicaBendingConstraint::CalcDihedralAngle(
	const FVector& V0, const FVector& V1, const FVector& V2, const FVector& V3)
{
	const FVector Edge = V1 - V0;
	const FVector N1 = FVector::CrossProduct(Edge, V2 - V0);
	const FVector N2 = FVector::CrossProduct(Edge, V3 - V0);

	const float N1Len = N1.Size();
	const float N2Len = N2.Size();
	if (N1Len < UE_SMALL_NUMBER || N2Len < UE_SMALL_NUMBER) return 0.0f;

	const FVector N1Norm = N1 / N1Len;
	const FVector N2Norm = N2 / N2Len;

	const float CosAngle = FMath::Clamp(FVector::DotProduct(N1Norm, N2Norm), -1.0f, 1.0f);
	const float Sign = FMath::Sign(FVector::DotProduct(FVector::CrossProduct(N1Norm, N2Norm), Edge));

	return FMath::Acos(CosAngle) * (Sign >= 0.0f ? 1.0f : -1.0f);
}

void FMagicaBendingConstraint::BuildFromChain(
	const TArray<FVector>& Positions,
	const TArray<int32>& ParentIndices,
	float InStiffness)
{
	Stiffness = InStiffness;
	Pairs.Reset();

	// For bone chains, use 3-consecutive-bone angle: grandparent-parent-child
	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		const int32 Parent = ParentIndices[i];
		if (Parent == INDEX_NONE) continue;
		const int32 GrandParent = ParentIndices[Parent];
		if (GrandParent == INDEX_NONE) continue;

		FMagicaBendPair Pair;
		Pair.V0 = GrandParent;  // Shared edge start
		Pair.V1 = Parent;       // Shared edge end (pivot)
		Pair.V2 = i;            // Child (opposite vertex in "triangle")
		Pair.V3 = i;            // Same as V2 for chain mode (degenerate)
		// For chain mode, store rest angle as the angle at the pivot
		const FVector A = Positions[GrandParent] - Positions[Parent];
		const FVector B = Positions[i] - Positions[Parent];
		const float ALen = A.Size();
		const float BLen = B.Size();
		if (ALen > UE_SMALL_NUMBER && BLen > UE_SMALL_NUMBER)
		{
			Pair.RestAngle = FMath::Acos(FMath::Clamp(
				FVector::DotProduct(A / ALen, B / BLen), -1.0f, 1.0f));
		}
		Pairs.Add(Pair);
	}
}

void FMagicaBendingConstraint::BuildFromMultiChain(
	const TArray<FVector>& Positions,
	const TArray<FMagicaChainRange>& ChainRanges,
	int32 SharedRootCount,
	float InStiffness)
{
	Stiffness = InStiffness;
	Pairs.Reset();

	// Per-chain: 3-bone bends within each chain
	for (const FMagicaChainRange& Range : ChainRanges)
	{
		for (int32 j = 2; j < Range.Count; ++j)
		{
			const int32 Idx = Range.StartIndex + j;
			const int32 Parent = Range.StartIndex + j - 1;
			const int32 GrandParent = Range.StartIndex + j - 2;

			FMagicaBendPair Pair;
			Pair.V0 = GrandParent;
			Pair.V1 = Parent;
			Pair.V2 = Idx;
			Pair.V3 = Idx;

			const FVector A = Positions[GrandParent] - Positions[Parent];
			const FVector B = Positions[Idx] - Positions[Parent];
			const float ALen = A.Size();
			const float BLen = B.Size();
			if (ALen > UE_SMALL_NUMBER && BLen > UE_SMALL_NUMBER)
			{
				Pair.RestAngle = FMath::Acos(FMath::Clamp(
					FVector::DotProduct(A / ALen, B / BLen), -1.0f, 1.0f));
			}
			Pairs.Add(Pair);
		}
	}

	// Root-to-first-bone bends (shared root → first chain bone → second chain bone)
	if (SharedRootCount > 0)
	{
		const int32 LastRoot = SharedRootCount - 1;
		for (const FMagicaChainRange& Range : ChainRanges)
		{
			if (Range.Count >= 2)
			{
				FMagicaBendPair Pair;
				Pair.V0 = LastRoot;
				Pair.V1 = Range.StartIndex;
				Pair.V2 = Range.StartIndex + 1;
				Pair.V3 = Range.StartIndex + 1;

				const FVector A = Positions[LastRoot] - Positions[Range.StartIndex];
				const FVector B = Positions[Range.StartIndex + 1] - Positions[Range.StartIndex];
				const float ALen = A.Size();
				const float BLen = B.Size();
				if (ALen > UE_SMALL_NUMBER && BLen > UE_SMALL_NUMBER)
				{
					Pair.RestAngle = FMath::Acos(FMath::Clamp(
						FVector::DotProduct(A / ALen, B / BLen), -1.0f, 1.0f));
				}
				Pairs.Add(Pair);
			}
		}
	}
}

void FMagicaBendingConstraint::Solve(FMagicaParticleArrays& Particles)
{
	for (const FMagicaBendPair& Pair : Pairs)
	{
		const FVector& PivotPos = Particles.PredictedPositions[Pair.V1];
		FVector& PosA = Particles.PredictedPositions[Pair.V0];
		FVector& PosC = Particles.PredictedPositions[Pair.V2];

		const float InvMassA = Particles.InvMasses[Pair.V0];
		const float InvMassP = Particles.InvMasses[Pair.V1];
		const float InvMassC = Particles.InvMasses[Pair.V2];

		// Chain-mode bend: angle at pivot between A-Pivot-C
		const FVector DirA = PosA - PivotPos;
		const FVector DirC = PosC - PivotPos;
		const float LenA = DirA.Size();
		const float LenC = DirC.Size();
		if (LenA < UE_SMALL_NUMBER || LenC < UE_SMALL_NUMBER) continue;

		const FVector NormA = DirA / LenA;
		const FVector NormC = DirC / LenC;

		const float CosAngle = FMath::Clamp(FVector::DotProduct(NormA, NormC), -1.0f, 1.0f);
		const float CurrentAngle = FMath::Acos(CosAngle);
		const float AngleError = CurrentAngle - Pair.RestAngle;

		if (FMath::Abs(AngleError) < UE_KINDA_SMALL_NUMBER) continue;

		// Correction direction: perpendicular to the bisector in the plane of A-Pivot-C
		FVector Axis = FVector::CrossProduct(NormA, NormC);
		const float AxisLen = Axis.Size();
		if (AxisLen < UE_SMALL_NUMBER) continue;
		Axis /= AxisLen;

		// Tangent corrections
		const FVector CorrA = FVector::CrossProduct(Axis, NormA) * (AngleError * Stiffness * 0.5f);
		const FVector CorrC = FVector::CrossProduct(NormC, Axis) * (AngleError * Stiffness * 0.5f);

		if (InvMassA > 0.0f) PosA += CorrA;
		if (InvMassC > 0.0f) PosC += CorrC;
	}
}
