#include "Simulation/FPBDSolver.h"
#include "Simulation/FClothConstraints.h"
#include "Colliders/MagicaColliderShapes.h"
#include "Math/UnrealMathUtility.h"

FPBDSolver::FPBDSolver() = default;
FPBDSolver::~FPBDSolver() = default;

// ── Single-Chain Initialize ─────────────────────────────

void FPBDSolver::Initialize(const TArray<FTransform>& BoneTransforms, const TArray<int32>& BoneIndices, int32 FixedCount)
{
	ChainRanges.Empty();
	SharedRootCount = 0;

	const int32 Count = BoneTransforms.Num();
	Particles.SetNum(Count);
	RestLocalOffsets.SetNum(Count);
	RestLocalRotations.SetNum(Count);

	for (int32 i = 0; i < Count; ++i)
	{
		FPBDParticle& P = Particles[i];
		P.Position          = BoneTransforms[i].GetTranslation();
		P.PrevPosition      = P.Position;
		P.Velocity          = FVector::ZeroVector;
		P.PredictedPosition = P.Position;
		P.BoneIndex         = BoneIndices.IsValidIndex(i) ? BoneIndices[i] : i;
		P.InvMass           = (i < FixedCount) ? 0.f : 1.f;
		P.ChainId           = 0; // single chain = chain 0

		if (i > 0)
		{
			RestLocalOffsets[i]   = BoneTransforms[i - 1].InverseTransformPosition(BoneTransforms[i].GetTranslation());
			RestLocalRotations[i] = BoneTransforms[i - 1].GetRotation().Inverse() * BoneTransforms[i].GetRotation();
		}
		else
		{
			RestLocalOffsets[i]   = FVector::ZeroVector;
			RestLocalRotations[i] = FQuat::Identity;
		}
	}
}

// ── Multi-Chain Initialize ──────────────────────────────

void FPBDSolver::InitializeMultiChain(
	const TArray<FTransform>& AllTransforms,
	const TArray<int32>& AllBoneIndices,
	int32 InSharedRootCount,
	const TArray<FChainRange>& InChainRanges)
{
	SharedRootCount = InSharedRootCount;
	ChainRanges = InChainRanges;

	const int32 Count = AllTransforms.Num();
	Particles.SetNum(Count);
	RestLocalOffsets.SetNum(Count);
	RestLocalRotations.SetNum(Count);

	// Initialize shared root particles (pinned)
	for (int32 i = 0; i < SharedRootCount && i < Count; ++i)
	{
		FPBDParticle& P = Particles[i];
		P.Position          = AllTransforms[i].GetTranslation();
		P.PrevPosition      = P.Position;
		P.Velocity          = FVector::ZeroVector;
		P.PredictedPosition = P.Position;
		P.BoneIndex         = AllBoneIndices.IsValidIndex(i) ? AllBoneIndices[i] : i;
		P.InvMass           = 0.f; // pinned
		P.ChainId           = INDEX_NONE; // shared root

		RestLocalOffsets[i]   = FVector::ZeroVector;
		RestLocalRotations[i] = FQuat::Identity;
	}

	// Initialize per-chain particles
	for (int32 ChainIdx = 0; ChainIdx < ChainRanges.Num(); ++ChainIdx)
	{
		const FChainRange& Range = ChainRanges[ChainIdx];

		for (int32 j = 0; j < Range.Count; ++j)
		{
			const int32 ParticleIdx = Range.StartIndex + j;
			if (!AllTransforms.IsValidIndex(ParticleIdx))
			{
				continue;
			}

			FPBDParticle& P = Particles[ParticleIdx];
			P.Position          = AllTransforms[ParticleIdx].GetTranslation();
			P.PrevPosition      = P.Position;
			P.Velocity          = FVector::ZeroVector;
			P.PredictedPosition = P.Position;
			P.BoneIndex         = AllBoneIndices.IsValidIndex(ParticleIdx) ? AllBoneIndices[ParticleIdx] : ParticleIdx;
			P.InvMass           = 1.f; // free
			P.ChainId           = ChainIdx;

			// Compute rest-pose local offset relative to parent
			int32 ParentIdx;
			if (j == 0)
			{
				// First particle in chain → parent is shared root (particle 0)
				ParentIdx = 0;
			}
			else
			{
				ParentIdx = ParticleIdx - 1;
			}

			RestLocalOffsets[ParticleIdx] = AllTransforms[ParentIdx].InverseTransformPosition(
				AllTransforms[ParticleIdx].GetTranslation());
			RestLocalRotations[ParticleIdx] = AllTransforms[ParentIdx].GetRotation().Inverse() *
				AllTransforms[ParticleIdx].GetRotation();
		}
	}
}

// ── Step ────────────────────────────────────────────────

void FPBDSolver::Step(float DeltaTime)
{
	if (DeltaTime <= UE_SMALL_NUMBER || Particles.Num() == 0)
	{
		return;
	}

	InvDt = 1.f / DeltaTime;

	PredictPositions(DeltaTime);
	SolveConstraints();
	SolveCollisions();
	UpdateVelocities(DeltaTime);
	ClampVelocities();
}

void FPBDSolver::UpdateFixedParticles(const TArray<FTransform>& AnimTransforms)
{
	for (int32 i = 0; i < Particles.Num(); ++i)
	{
		if (Particles[i].InvMass <= 0.f && AnimTransforms.IsValidIndex(i))
		{
			Particles[i].Position     = AnimTransforms[i].GetTranslation();
			Particles[i].PrevPosition = Particles[i].Position;
			Particles[i].Velocity     = FVector::ZeroVector;
		}
	}
}

// ── GetResultTransforms ─────────────────────────────────

void FPBDSolver::GetResultTransforms(TArray<FTransform>& OutTransforms) const
{
	// Multi-chain mode: use per-chain rotation reconstruction
	if (ChainRanges.Num() > 0)
	{
		GetResultTransformsMultiChain(OutTransforms);
		return;
	}

	// Single-chain mode (original logic)
	const int32 Count = Particles.Num();
	OutTransforms.SetNum(Count);

	for (int32 i = 0; i < Count; ++i)
	{
		FQuat Rotation = FQuat::Identity;

		if (i < Count - 1)
		{
			const FVector CurrentDir = (Particles[i + 1].Position - Particles[i].Position).GetSafeNormal();
			const FVector RestDir    = RestLocalOffsets.IsValidIndex(i + 1)
				? RestLocalOffsets[i + 1].GetSafeNormal()
				: FVector::DownVector;

			if (!RestDir.IsNearlyZero() && !CurrentDir.IsNearlyZero())
			{
				Rotation = FQuat::FindBetweenNormals(RestDir, CurrentDir);
			}
		}
		else if (i > 0)
		{
			Rotation = OutTransforms[i - 1].GetRotation();
		}

		OutTransforms[i] = FTransform(Rotation, Particles[i].Position, FVector::OneVector);
	}
}

void FPBDSolver::GetResultTransformsMultiChain(TArray<FTransform>& OutTransforms) const
{
	const int32 Count = Particles.Num();
	OutTransforms.SetNum(Count);

	// Shared root bones: use identity rotation (they are pinned, animation drives them)
	for (int32 i = 0; i < SharedRootCount && i < Count; ++i)
	{
		OutTransforms[i] = FTransform(FQuat::Identity, Particles[i].Position, FVector::OneVector);
	}

	// Per-chain rotation reconstruction
	for (const FChainRange& Range : ChainRanges)
	{
		for (int32 j = 0; j < Range.Count; ++j)
		{
			const int32 Idx = Range.StartIndex + j;
			if (Idx >= Count)
			{
				break;
			}

			FQuat Rotation = FQuat::Identity;

			// Determine parent particle
			const int32 ParentIdx = (j == 0) ? 0 : (Idx - 1);

			// Look at child to determine direction
			const int32 ChildIdx = (j < Range.Count - 1) ? (Idx + 1) : INDEX_NONE;

			if (ChildIdx != INDEX_NONE && ChildIdx < Count)
			{
				const FVector CurrentDir = (Particles[ChildIdx].Position - Particles[Idx].Position).GetSafeNormal();
				const FVector RestDir = RestLocalOffsets.IsValidIndex(ChildIdx)
					? RestLocalOffsets[ChildIdx].GetSafeNormal()
					: FVector::DownVector;

				if (!RestDir.IsNearlyZero() && !CurrentDir.IsNearlyZero())
				{
					Rotation = FQuat::FindBetweenNormals(RestDir, CurrentDir);
				}
			}
			else if (j > 0)
			{
				// Last bone in chain: inherit parent's rotation
				Rotation = OutTransforms[Idx - 1].GetRotation();
			}

			OutTransforms[Idx] = FTransform(Rotation, Particles[Idx].Position, FVector::OneVector);
		}
	}

	// Update root rotation based on first chain's first particle direction
	if (SharedRootCount > 0 && ChainRanges.Num() > 0 && ChainRanges[0].Count > 0)
	{
		const int32 FirstChildIdx = ChainRanges[0].StartIndex;
		if (FirstChildIdx < Count)
		{
			const FVector Dir = (Particles[FirstChildIdx].Position - Particles[0].Position).GetSafeNormal();
			const FVector RestDir = RestLocalOffsets.IsValidIndex(FirstChildIdx)
				? RestLocalOffsets[FirstChildIdx].GetSafeNormal()
				: FVector::DownVector;

			if (!RestDir.IsNearlyZero() && !Dir.IsNearlyZero())
			{
				OutTransforms[0] = FTransform(
					FQuat::FindBetweenNormals(RestDir, Dir),
					Particles[0].Position, FVector::OneVector);
			}
		}
	}
}

void FPBDSolver::ResetToRestPose(const TArray<FTransform>& RestTransforms)
{
	for (int32 i = 0; i < Particles.Num() && i < RestTransforms.Num(); ++i)
	{
		Particles[i].Position          = RestTransforms[i].GetTranslation();
		Particles[i].PrevPosition      = Particles[i].Position;
		Particles[i].Velocity          = FVector::ZeroVector;
		Particles[i].PredictedPosition = Particles[i].Position;
	}
}

// ── PBD Substeps ────────────────────────────────────────

void FPBDSolver::PredictPositions(float DeltaTime)
{
	for (FPBDParticle& P : Particles)
	{
		if (P.InvMass <= 0.f)
		{
			P.PredictedPosition = P.Position;
			continue;
		}

		P.Velocity *= FMath::Clamp(1.f - Damping, 0.f, 1.f);
		P.Velocity += Gravity * DeltaTime;
		P.PredictedPosition = P.Position + P.Velocity * DeltaTime;
	}
}

void FPBDSolver::SolveConstraints()
{
	for (int32 Iter = 0; Iter < SolverIterations; ++Iter)
	{
		for (const TSharedPtr<FClothConstraintBase>& Constraint : Constraints)
		{
			if (Constraint.IsValid())
			{
				Constraint->Solve(Particles);
			}
		}
	}
}

void FPBDSolver::SolveCollisions()
{
	for (const TSharedPtr<FMagicaColliderShape>& Collider : Colliders)
	{
		if (Collider.IsValid())
		{
			for (FPBDParticle& P : Particles)
			{
				if (P.InvMass > 0.f)
				{
					Collider->ResolveCollision(P.PredictedPosition);
				}
			}
		}
	}
}

void FPBDSolver::UpdateVelocities(float DeltaTime)
{
	for (FPBDParticle& P : Particles)
	{
		if (P.InvMass <= 0.f)
		{
			continue;
		}

		P.Velocity     = (P.PredictedPosition - P.Position) * InvDt;
		P.PrevPosition = P.Position;
		P.Position     = P.PredictedPosition;
	}
}

void FPBDSolver::ClampVelocities()
{
	if (MaxVelocity <= 0.f)
	{
		return;
	}

	const float MaxVelSq = MaxVelocity * MaxVelocity;
	for (FPBDParticle& P : Particles)
	{
		if (P.Velocity.SizeSquared() > MaxVelSq)
		{
			P.Velocity = P.Velocity.GetSafeNormal() * MaxVelocity;
		}
	}
}
