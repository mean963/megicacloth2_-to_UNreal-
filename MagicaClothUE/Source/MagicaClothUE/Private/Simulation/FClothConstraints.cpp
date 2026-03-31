#include "Simulation/FClothConstraints.h"
#include "Simulation/FPBDSolver.h"
#include "Math/UnrealMathUtility.h"
#include "Math/Vector.h"
#include "Containers/Array.h"

// ---------- Structural Constraint ----------

void FStructuralConstraint::Solve(TArray<FPBDParticle>& Particles)
{
	if (!Particles.IsValidIndex(ParticleA) || !Particles.IsValidIndex(ParticleB))
	{
		return;
	}

	FPBDParticle& A = Particles[ParticleA];
	FPBDParticle& B = Particles[ParticleB];

	const FVector Delta = B.PredictedPosition - A.PredictedPosition;
	const float CurrentLength = Delta.Size();

	if (CurrentLength < UE_SMALL_NUMBER)
	{
		return;
	}

	const float Error = CurrentLength - RestLength;
	const FVector Correction = Delta.GetSafeNormal() * Error;

	const float TotalInvMass = A.InvMass + B.InvMass;
	if (TotalInvMass <= UE_SMALL_NUMBER)
	{
		return;
	}

	if (A.InvMass > 0.f)
	{
		A.PredictedPosition += Correction * (A.InvMass / TotalInvMass) * Stiffness;
	}
	if (B.InvMass > 0.f)
	{
		B.PredictedPosition -= Correction * (B.InvMass / TotalInvMass) * Stiffness;
	}
}

// ---------- Bend Constraint ----------

void FBendConstraint::Solve(TArray<FPBDParticle>& Particles)
{
	if (!Particles.IsValidIndex(ParticleA) || !Particles.IsValidIndex(ParticleB) || !Particles.IsValidIndex(ParticleC))
	{
		return;
	}

	FPBDParticle& A = Particles[ParticleA];
	FPBDParticle& B = Particles[ParticleB];
	FPBDParticle& C = Particles[ParticleC];

	const FVector AB = B.PredictedPosition - A.PredictedPosition;
	const FVector BC = C.PredictedPosition - B.PredictedPosition;

	const float ABLen = AB.Size();
	const float BCLen = BC.Size();

	if (ABLen < UE_SMALL_NUMBER || BCLen < UE_SMALL_NUMBER)
	{
		return;
	}

	const FVector ABNorm = AB / ABLen;
	const FVector BCNorm = BC / BCLen;

	const float CosAngle = FMath::Clamp(FVector::DotProduct(ABNorm, BCNorm), -1.f, 1.f);
	const float CurrentAngle = FMath::Acos(CosAngle);
	const float AngleError = CurrentAngle - RestAngle;

	if (FMath::Abs(AngleError) < UE_SMALL_NUMBER)
	{
		return;
	}

	FVector Axis = FVector::CrossProduct(ABNorm, BCNorm);
	if (Axis.SizeSquared() < UE_SMALL_NUMBER)
	{
		return;
	}
	Axis.Normalize();

	const float CorrectionMag = AngleError * Stiffness * 0.5f;

	if (B.InvMass > 0.f)
	{
		const FVector Tangent = FVector::CrossProduct(Axis, ABNorm);
		B.PredictedPosition += Tangent * CorrectionMag * ABLen * 0.5f;
	}

	if (C.InvMass > 0.f)
	{
		const FVector Tangent = FVector::CrossProduct(Axis, BCNorm);
		C.PredictedPosition -= Tangent * CorrectionMag * BCLen * 0.5f;
	}
}

// ---------- Self-Collision Constraint ----------

void FSelfCollisionConstraint::Solve(TArray<FPBDParticle>& Particles)
{
	const float RadiusSq = CollisionRadius * CollisionRadius;
	const float Diameter = CollisionRadius * 2.f;
	const int32 Count = Particles.Num();

	for (int32 i = 0; i < Count; ++i)
	{
		if (Particles[i].InvMass <= 0.f)
		{
			continue;
		}

		for (int32 j = i + 2; j < Count; ++j) // skip adjacent
		{
			if (Particles[j].InvMass <= 0.f)
			{
				continue;
			}

			const FVector Delta = Particles[j].PredictedPosition - Particles[i].PredictedPosition;
			const float DistSq = Delta.SizeSquared();

			if (DistSq < RadiusSq * 4.f && DistSq > UE_SMALL_NUMBER)
			{
				const float Dist = FMath::Sqrt(DistSq);
				const float Penetration = Diameter - Dist;

				if (Penetration > 0.f)
				{
					const FVector Dir = Delta / Dist;
					const float TotalInvMass = Particles[i].InvMass + Particles[j].InvMass;

					Particles[i].PredictedPosition -= Dir * Penetration * (Particles[i].InvMass / TotalInvMass) * 0.5f;
					Particles[j].PredictedPosition += Dir * Penetration * (Particles[j].InvMass / TotalInvMass) * 0.5f;
				}
			}
		}
	}
}

// ---------- Factory ----------

void FClothConstraintFactory::BuildChainConstraints(
	const TArray<FPBDParticle>& Particles,
	float StructuralStiffness,
	float BendStiffness,
	TArray<TSharedPtr<FClothConstraintBase>>& OutConstraints)
{
	const int32 Count = Particles.Num();

	// Structural: each consecutive pair
	for (int32 i = 0; i < Count - 1; ++i)
	{
		const float RestLen = FVector::Dist(Particles[i].Position, Particles[i + 1].Position);
		OutConstraints.Add(MakeShared<FStructuralConstraint>(i, i + 1, RestLen, StructuralStiffness));
	}

	// Bend: each consecutive triple
	for (int32 i = 0; i < Count - 2; ++i)
	{
		const FVector AB = (Particles[i + 1].Position - Particles[i].Position).GetSafeNormal();
		const FVector BC = (Particles[i + 2].Position - Particles[i + 1].Position).GetSafeNormal();
		const float CosAngle = FMath::Clamp(FVector::DotProduct(AB, BC), -1.f, 1.f);
		const float RestAngle = FMath::Acos(CosAngle);
		OutConstraints.Add(MakeShared<FBendConstraint>(i, i + 1, i + 2, RestAngle, BendStiffness));
	}
}

// ---------- Multi-Chain Factory ----------

void FClothConstraintFactory::BuildMultiChainConstraints(
	const TArray<FPBDParticle>& Particles,
	int32 SharedRootCount,
	const TArray<FChainRange>& ChainRanges,
	float StructuralStiffness,
	float BendStiffness,
	TArray<TSharedPtr<FClothConstraintBase>>& OutConstraints)
{
	const int32 RootIdx = 0; // shared root particle

	for (const FChainRange& Range : ChainRanges)
	{
		if (Range.Count <= 0)
		{
			continue;
		}

		// Structural: root → first chain particle
		{
			const int32 FirstIdx = Range.StartIndex;
			if (Particles.IsValidIndex(RootIdx) && Particles.IsValidIndex(FirstIdx))
			{
				const float RestLen = FVector::Dist(Particles[RootIdx].Position, Particles[FirstIdx].Position);
				OutConstraints.Add(MakeShared<FStructuralConstraint>(RootIdx, FirstIdx, RestLen, StructuralStiffness));
			}
		}

		// Structural: consecutive pairs within the chain
		for (int32 j = 0; j < Range.Count - 1; ++j)
		{
			const int32 IdxA = Range.StartIndex + j;
			const int32 IdxB = Range.StartIndex + j + 1;
			if (Particles.IsValidIndex(IdxA) && Particles.IsValidIndex(IdxB))
			{
				const float RestLen = FVector::Dist(Particles[IdxA].Position, Particles[IdxB].Position);
				OutConstraints.Add(MakeShared<FStructuralConstraint>(IdxA, IdxB, RestLen, StructuralStiffness));
			}
		}

		// Bend: root → first → second
		if (Range.Count >= 2)
		{
			const int32 A = RootIdx;
			const int32 B = Range.StartIndex;
			const int32 C = Range.StartIndex + 1;
			if (Particles.IsValidIndex(A) && Particles.IsValidIndex(B) && Particles.IsValidIndex(C))
			{
				const FVector AB = (Particles[B].Position - Particles[A].Position).GetSafeNormal();
				const FVector BC = (Particles[C].Position - Particles[B].Position).GetSafeNormal();
				const float CosAngle = FMath::Clamp(FVector::DotProduct(AB, BC), -1.f, 1.f);
				OutConstraints.Add(MakeShared<FBendConstraint>(A, B, C, FMath::Acos(CosAngle), BendStiffness));
			}
		}

		// Bend: consecutive triples within the chain
		for (int32 j = 0; j < Range.Count - 2; ++j)
		{
			const int32 A = Range.StartIndex + j;
			const int32 B = Range.StartIndex + j + 1;
			const int32 C = Range.StartIndex + j + 2;
			if (Particles.IsValidIndex(A) && Particles.IsValidIndex(B) && Particles.IsValidIndex(C))
			{
				const FVector AB = (Particles[B].Position - Particles[A].Position).GetSafeNormal();
				const FVector BC = (Particles[C].Position - Particles[B].Position).GetSafeNormal();
				const float CosAngle = FMath::Clamp(FVector::DotProduct(AB, BC), -1.f, 1.f);
				OutConstraints.Add(MakeShared<FBendConstraint>(A, B, C, FMath::Acos(CosAngle), BendStiffness));
			}
		}
	}
}

// ---------- Cross-Chain Mesh Net Factory ----------
//
// Sorts chains by angle around root (using actual particle positions),
// then connects spatially adjacent chains with horizontal + diagonal constraints.

void FClothConstraintFactory::BuildCrossChainMeshConstraints(
	const TArray<FPBDParticle>& Particles,
	const TArray<FChainRange>& ChainRanges,
	float HorizontalStiffness,
	float ShearStiffness,
	bool bClosedLoop,
	TArray<TSharedPtr<FClothConstraintBase>>& OutConstraints)
{
	const int32 NumChains = ChainRanges.Num();
	if (NumChains < 2 || !Particles.IsValidIndex(0))
	{
		return;
	}

	// Step 1: Get each chain's first particle position (component space)
	TArray<FVector> ChainPositions;
	ChainPositions.SetNum(NumChains);
	for (int32 i = 0; i < NumChains; ++i)
	{
		const FChainRange& Range = ChainRanges[i];
		ChainPositions[i] = (Range.Count > 0 && Particles.IsValidIndex(Range.StartIndex))
			? Particles[Range.StartIndex].Position
			: FVector::ZeroVector;
	}

	// Step 2: Nearest-neighbor traversal to build spatially ordered chain ring.
	// This works regardless of skeleton orientation / coordinate system.
	// Start from chain 0, greedily pick closest unvisited chain.
	TArray<int32> SortedChainIndices;
	TArray<bool> Visited;
	Visited.SetNumZeroed(NumChains);

	SortedChainIndices.Add(0);
	Visited[0] = true;

	for (int32 Step = 1; Step < NumChains; ++Step)
	{
		const int32 LastIdx = SortedChainIndices.Last();
		const FVector& LastPos = ChainPositions[LastIdx];

		int32 NearestIdx = INDEX_NONE;
		float NearestDistSq = TNumericLimits<float>::Max();

		for (int32 i = 0; i < NumChains; ++i)
		{
			if (Visited[i])
			{
				continue;
			}
			const float DistSq = FVector::DistSquared(LastPos, ChainPositions[i]);
			if (DistSq < NearestDistSq)
			{
				NearestDistSq = DistSq;
				NearestIdx = i;
			}
		}

		if (NearestIdx != INDEX_NONE)
		{
			SortedChainIndices.Add(NearestIdx);
			Visited[NearestIdx] = true;
		}
	}

	const int32 NumSorted = SortedChainIndices.Num();
	if (NumSorted < 2)
	{
		return;
	}

	// Step 3: Connect adjacent chains in sorted order
	const int32 NumPairs = bClosedLoop ? NumSorted : (NumSorted - 1);

	// Helper lambda to connect two chains
	auto ConnectChains = [&](const FChainRange& ChainA, const FChainRange& ChainB)
	{
		const int32 MinCount = FMath::Min(ChainA.Count, ChainB.Count);

		for (int32 Depth = 0; Depth < MinCount; ++Depth)
		{
			const int32 IdxA = ChainA.StartIndex + Depth;
			const int32 IdxB = ChainB.StartIndex + Depth;

			if (!Particles.IsValidIndex(IdxA) || !Particles.IsValidIndex(IdxB))
			{
				continue;
			}

			// Horizontal constraint: A[d] ↔ B[d]
			const float RestLen = FVector::Dist(Particles[IdxA].Position, Particles[IdxB].Position);
			OutConstraints.Add(MakeShared<FStructuralConstraint>(IdxA, IdxB, RestLen, HorizontalStiffness));

			// Diagonal (shear) constraints
			if (Depth < MinCount - 1)
			{
				const int32 IdxA_next = ChainA.StartIndex + Depth + 1;
				const int32 IdxB_next = ChainB.StartIndex + Depth + 1;

				if (Particles.IsValidIndex(IdxA) && Particles.IsValidIndex(IdxB_next))
				{
					const float D = FVector::Dist(Particles[IdxA].Position, Particles[IdxB_next].Position);
					OutConstraints.Add(MakeShared<FStructuralConstraint>(IdxA, IdxB_next, D, ShearStiffness));
				}
				if (Particles.IsValidIndex(IdxB) && Particles.IsValidIndex(IdxA_next))
				{
					const float D = FVector::Dist(Particles[IdxB].Position, Particles[IdxA_next].Position);
					OutConstraints.Add(MakeShared<FStructuralConstraint>(IdxB, IdxA_next, D, ShearStiffness));
				}
			}
		}
	};

	for (int32 PairIdx = 0; PairIdx < NumPairs; ++PairIdx)
	{
		const int32 ChainIdxA = SortedChainIndices[PairIdx];
		const int32 ChainIdxB = SortedChainIndices[(PairIdx + 1) % NumSorted];
		ConnectChains(ChainRanges[ChainIdxA], ChainRanges[ChainIdxB]);
	}
}
