// MagicaClothUE/Public/Simulation/Constraints/ConstraintBase.h
#pragma once

#include "CoreMinimal.h"

/** Particle data used by constraints (SoA layout for cache efficiency). */
struct FMagicaParticleArrays
{
	TArray<FVector>& Positions;
	TArray<FVector>& PredictedPositions;
	TArray<FVector>& Velocities;
	const TArray<float>& InvMasses;
	const TArray<float>& Depths;  // 0.0=root, 1.0=tip
};

/**
 * Base constraint interface for PBD solver.
 * Constraints operate on particle arrays by index.
 */
struct MAGICACLOTHUE_API FMagicaConstraintBase
{
	virtual ~FMagicaConstraintBase() = default;

	/** Solve this constraint, modifying PredictedPositions. */
	virtual void Solve(FMagicaParticleArrays& Particles) = 0;
};
