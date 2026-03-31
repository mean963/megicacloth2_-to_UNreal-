// MagicaClothUE/Private/Simulation/FPBDSolver.cpp
#include "Simulation/FPBDSolver.h"
#include "VirtualMesh/VirtualMesh.h"
#include "Simulation/Constraints/InertiaConstraint.h"
#include "Colliders/MagicaColliderShapes.h"

FMagicaPBDSolver::FMagicaPBDSolver() = default;
FMagicaPBDSolver::~FMagicaPBDSolver() = default;

void FMagicaPBDSolver::Initialize(const FMagicaVirtualMesh& Mesh)
{
	const int32 Count = Mesh.GetVertexCount();

	Positions.SetNum(Count);
	PrevPositions.SetNum(Count);
	PredictedPositions.SetNum(Count);
	Velocities.SetNumZeroed(Count);
	InvMasses.SetNum(Count);
	Depths.SetNum(Count);

	for (int32 i = 0; i < Count; ++i)
	{
		Positions[i] = Mesh.Positions[i];
		PrevPositions[i] = Mesh.Positions[i];
		PredictedPositions[i] = Mesh.Positions[i];
		InvMasses[i] = Mesh.Attributes[i].IsFixed() ? 0.0f : (1.0f / 1.0f); // Default mass 1.0
		Depths[i] = Mesh.Attributes[i].Depth;
	}
}

FMagicaParticleArrays FMagicaPBDSolver::MakeParticleArrays()
{
	return FMagicaParticleArrays{
		Positions,
		PredictedPositions,
		Velocities,
		InvMasses,
		Depths
	};
}

void FMagicaPBDSolver::Step(float DeltaTime)
{
	if (DeltaTime <= 0.0f || Positions.Num() == 0) return;

	// 1. Inertia update (before prediction)
	if (InertiaConstraint.IsValid())
	{
		auto Arrays = MakeParticleArrays();
		InertiaConstraint->Solve(Arrays);
	}

	// 2. Predict
	PredictPositions(DeltaTime);

	// 3. Pre-collision constraints
	SolveConstraints(PreCollisionConstraints);

	// 4. Collisions
	SolveCollisions();

	// 5. Post-collision constraints
	SolveConstraints(PostCollisionConstraints);

	// 6. Update velocities and apply damping
	UpdateVelocities(DeltaTime);
	ClampVelocities();
	ApplyDamping();

	// 7. Accept predicted positions
	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		PrevPositions[i] = Positions[i];
		Positions[i] = PredictedPositions[i];
	}
}

void FMagicaPBDSolver::UpdateFixedParticles(const TArray<FVector>& AnimPositions)
{
	const int32 Count = FMath::Min(AnimPositions.Num(), Positions.Num());
	for (int32 i = 0; i < Count; ++i)
	{
		if (InvMasses[i] <= 0.0f)
		{
			Positions[i] = AnimPositions[i];
			PrevPositions[i] = AnimPositions[i];
			PredictedPositions[i] = AnimPositions[i];
			Velocities[i] = FVector::ZeroVector;
		}
	}
}

void FMagicaPBDSolver::Reset(const TArray<FVector>& RestPositions)
{
	const int32 Count = FMath::Min(RestPositions.Num(), Positions.Num());
	for (int32 i = 0; i < Count; ++i)
	{
		Positions[i] = RestPositions[i];
		PrevPositions[i] = RestPositions[i];
		PredictedPositions[i] = RestPositions[i];
		Velocities[i] = FVector::ZeroVector;
	}
}

void FMagicaPBDSolver::PredictPositions(float Dt)
{
	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		if (InvMasses[i] <= 0.0f)
		{
			PredictedPositions[i] = Positions[i];
			continue;
		}

		Velocities[i] += Gravity * Dt;
		PredictedPositions[i] = Positions[i] + Velocities[i] * Dt;
	}
}

void FMagicaPBDSolver::SolveConstraints(TArray<TSharedPtr<FMagicaConstraintBase>>& Constraints)
{
	auto Arrays = MakeParticleArrays();
	for (int32 Iter = 0; Iter < SolverIterations; ++Iter)
	{
		for (auto& Constraint : Constraints)
		{
			Constraint->Solve(Arrays);
		}
	}
}

void FMagicaPBDSolver::SolveCollisions()
{
	for (int32 i = 0; i < PredictedPositions.Num(); ++i)
	{
		if (InvMasses[i] <= 0.0f) continue;

		for (const auto& Collider : Colliders)
		{
			Collider->ResolveCollision(PredictedPositions[i]);
		}
	}
}

void FMagicaPBDSolver::UpdateVelocities(float Dt)
{
	const float InvDt = 1.0f / Dt;
	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		if (InvMasses[i] > 0.0f)
		{
			Velocities[i] = (PredictedPositions[i] - Positions[i]) * InvDt;
		}
	}
}

void FMagicaPBDSolver::ClampVelocities()
{
	const float MaxVelSq = MaxVelocity * MaxVelocity;
	for (int32 i = 0; i < Velocities.Num(); ++i)
	{
		if (Velocities[i].SizeSquared() > MaxVelSq)
		{
			Velocities[i] = Velocities[i].GetSafeNormal() * MaxVelocity;
		}
	}
}

void FMagicaPBDSolver::ApplyDamping()
{
	const float DampFactor = 1.0f - Damping;
	for (int32 i = 0; i < Velocities.Num(); ++i)
	{
		if (InvMasses[i] > 0.0f)
		{
			// Depth-based damping: root damps more, tip damps less
			const float DepthDamp = FMath::Lerp(DampFactor, 1.0f, Depths[i] * 0.3f);
			Velocities[i] *= DepthDamp;
		}
	}
}
