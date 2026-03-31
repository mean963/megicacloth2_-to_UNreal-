// MagicaClothUE/Public/Simulation/FPBDSolver.h
#pragma once

#include "CoreMinimal.h"
#include "Core/ClothTypes.h"
#include "Simulation/Constraints/ConstraintBase.h"

class FMagicaVirtualMesh;
struct FMagicaColliderShape;
struct FMagicaConstraintBase;
struct FMagicaDistanceConstraint;
struct FMagicaBendingConstraint;
struct FMagicaTetherConstraint;
struct FMagicaInertiaConstraint;
struct FMagicaAngleConstraint;

/**
 * Position Based Dynamics solver.
 * Generic: operates on flat particle arrays (works for BoneCloth and MeshCloth).
 * Constraint objects are built by the caller and owned by the solver.
 */
class MAGICACLOTHUE_API FMagicaPBDSolver
{
public:
	FMagicaPBDSolver();
	~FMagicaPBDSolver();

	/** Initialize from a VirtualMesh. Sets up particle arrays. */
	void Initialize(const FMagicaVirtualMesh& Mesh);

	/** Main simulation step. */
	void Step(float DeltaTime);

	/** Update pinned particle positions from animation. */
	void UpdateFixedParticles(const TArray<FVector>& AnimPositions);

	/** Get current particle positions. */
	const TArray<FVector>& GetPositions() const { return Positions; }

	/** Get particle count. */
	int32 GetParticleCount() const { return Positions.Num(); }

	/** Reset all particles to given positions and zero velocities. */
	void Reset(const TArray<FVector>& RestPositions);

	// --- Parameters ---
	FVector Gravity = FVector(0.0, 0.0, -980.0);
	float Damping = 0.1f;
	float MaxVelocity = 5000.0f;
	int32 SolverIterations = 5;

	// --- Constraints (owned by solver, built by caller) ---
	TArray<TSharedPtr<FMagicaConstraintBase>> PreCollisionConstraints;
	TArray<TSharedPtr<FMagicaConstraintBase>> PostCollisionConstraints;
	TSharedPtr<FMagicaInertiaConstraint> InertiaConstraint;

	// --- Colliders ---
	TArray<TSharedPtr<FMagicaColliderShape>> Colliders;

	// --- Particle data (SoA) ---
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<FVector> PredictedPositions;
	TArray<FVector> Velocities;
	TArray<float> InvMasses;
	TArray<float> Depths;

private:
	void PredictPositions(float Dt);
	void SolveConstraints(TArray<TSharedPtr<FMagicaConstraintBase>>& Constraints);
	void SolveCollisions();
	void UpdateVelocities(float Dt);
	void ClampVelocities();
	void ApplyDamping();

	FMagicaParticleArrays MakeParticleArrays();
};
