# MagicaCloth UE Plugin — Phase 1 Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the MagicaClothUE plugin from BoneCloth-only to a full MagicaCloth2 architecture with Manager/Team pattern, 6 constraint types, 3-way collision sourcing, and KawaiiPhysics-style parameters.

**Architecture:** UWorldSubsystem coordinates all cloth instances. A single SimulationManager worker thread runs PBD at 90Hz for all Teams. Each Team has its own DoubleBuffer for lock-free result passing. AnimBP node reads results and applies bone overrides.

**Tech Stack:** UE 5.5+ C++20, FRunnable worker thread, FAnimNode_SkeletalControlBase, FRuntimeFloatCurve, UWorldSubsystem, UDataAsset

**Existing code location:** `D:\AI_document\physics_plugins\MagicaClothUE\Source\`

**Note on testing:** UE plugin code requires the full engine to compile and test. Each task ends with a compile check. Runtime verification happens after Task 15 when all pieces connect through the AnimNode.

---

## File Map

### New Files (Create)

| File | Responsibility |
|------|---------------|
| `MagicaClothUE/Public/Core/ClothTypes.h` | Shared enums, structs, constants (EMagicaClothType, FDataChunk, FMagicaPhysicsSettings, etc.) |
| `MagicaClothUE/Public/Core/MagicaClothSubsystem.h` | UWorldSubsystem — owns TeamManager + SimulationManager |
| `MagicaClothUE/Private/Core/MagicaClothSubsystem.cpp` | Subsystem lifecycle, Team registration |
| `MagicaClothUE/Public/Core/TeamManager.h` | Team registration, particle array management |
| `MagicaClothUE/Private/Core/TeamManager.cpp` | Team CRUD, DataChunk allocation |
| `MagicaClothUE/Public/VirtualMesh/VertexAttribute.h` | Vertex attribute flags (Fixed/Move/Limit/etc.) |
| `MagicaClothUE/Public/VirtualMesh/VirtualMesh.h` | CPU mesh representation |
| `MagicaClothUE/Private/VirtualMesh/VirtualMesh.cpp` | BoneCloth mesh building, depth calculation |
| `MagicaClothUE/Public/Simulation/Constraints/ConstraintBase.h` | Base constraint interface |
| `MagicaClothUE/Public/Simulation/Constraints/DistanceConstraint.h` | Vertical + Horizontal distance |
| `MagicaClothUE/Private/Simulation/Constraints/DistanceConstraint.cpp` | Distance solve implementation |
| `MagicaClothUE/Public/Simulation/Constraints/BendingConstraint.h` | Dihedral angle bending |
| `MagicaClothUE/Private/Simulation/Constraints/BendingConstraint.cpp` | Bending solve implementation |
| `MagicaClothUE/Public/Simulation/Constraints/TetherConstraint.h` | Max distance from root |
| `MagicaClothUE/Private/Simulation/Constraints/TetherConstraint.cpp` | Tether solve implementation |
| `MagicaClothUE/Public/Simulation/Constraints/InertiaConstraint.h` | World/Local inertia + teleport |
| `MagicaClothUE/Private/Simulation/Constraints/InertiaConstraint.cpp` | Inertia solve implementation |
| `MagicaClothUE/Public/Simulation/Constraints/AngleConstraint.h` | Joint angle preservation (BoneCloth) |
| `MagicaClothUE/Private/Simulation/Constraints/AngleConstraint.cpp` | Angle solve implementation |
| `MagicaClothUE/Public/Simulation/SimulationManager.h` | Central simulation coordinator (worker thread) |
| `MagicaClothUE/Private/Simulation/SimulationManager.cpp` | Worker thread loop, Team batching |
| `MagicaClothUE/Public/Colliders/ColliderLimits.h` | KawaiiPhysics-style limit structs (FSphericalLimit, FCapsuleLimit, etc.) |
| `MagicaClothUE/Public/Colliders/MagicaLimitsDataAsset.h` | Reusable collider DataAsset |
| `MagicaClothUE/Private/Colliders/MagicaLimitsDataAsset.cpp` | DataAsset implementation |

### Modified Files (Refactor)

| File | Changes |
|------|---------|
| `MagicaClothUE/Public/Simulation/FClothDoubleBuffer.h` | Minor: keep as-is (already good) |
| `MagicaClothUE/Public/Colliders/MagicaColliderShapes.h` | Add Plane collider, Inner/Outer mode, friction model |
| `MagicaClothUE/Public/Simulation/FPBDSolver.h` | Major refactor: generic particle arrays, depth support, new constraint interface |
| `MagicaClothUE/Private/Simulation/FPBDSolver.cpp` | Rewrite solver loop |
| `MagicaClothUE/Public/AnimNode/AnimNode_MagicaCloth.h` | Major refactor: KawaiiPhysics-style categories, depth curves, 3-way colliders |
| `MagicaClothUE/Private/AnimNode/AnimNode_MagicaCloth.cpp` | Rewrite to use new architecture |
| `MagicaClothUEEditor/Public/AnimGraphNode_MagicaCloth.h` | Add debug toggles, detail customization |
| `MagicaClothUEEditor/Private/AnimGraphNode_MagicaCloth.cpp` | Implement customization |
| `MagicaClothUE/MagicaClothUE.Build.cs` | Add StructUtils dependency for FInstancedStruct |
| `MagicaClothUE/Public/MagicaClothComponent.h` | Refactor to use Subsystem |
| `MagicaClothUE/Private/MagicaClothComponent.cpp` | Refactor to register with Subsystem |

### Deleted Files

| File | Reason |
|------|--------|
| `MagicaClothUE/Public/Simulation/FClothConstraints.h` | Replaced by individual constraint files in Constraints/ |
| `MagicaClothUE/Private/Simulation/FClothConstraints.cpp` | Replaced |
| `MagicaClothUE/Public/Simulation/FClothSimThread.h` | Replaced by SimulationManager |
| `MagicaClothUE/Private/Simulation/FClothSimThread.cpp` | Replaced |

---

## Task 1: Foundation Types (ClothTypes.h + VertexAttribute.h)

**Files:**
- Create: `MagicaClothUE/Public/Core/ClothTypes.h`
- Create: `MagicaClothUE/Public/VirtualMesh/VertexAttribute.h`

- [ ] **Step 1: Create Core directory**

```bash
mkdir -p "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Public/Core"
mkdir -p "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Private/Core"
mkdir -p "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Public/VirtualMesh"
mkdir -p "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Private/VirtualMesh"
mkdir -p "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Public/Simulation/Constraints"
mkdir -p "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Private/Simulation/Constraints"
```

- [ ] **Step 2: Write ClothTypes.h**

```cpp
// MagicaClothUE/Public/Core/ClothTypes.h
#pragma once

#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"
#include "ClothTypes.generated.h"

/** Cloth simulation mode. */
UENUM(BlueprintType)
enum class EMagicaClothType : uint8
{
	BoneCloth  = 0 UMETA(DisplayName = "Bone Cloth"),
	MeshCloth  = 1 UMETA(DisplayName = "Mesh Cloth"),
};

/** Simulation coordinate space. */
UENUM(BlueprintType)
enum class EMagicaSimulationSpace : uint8
{
	ComponentSpace UMETA(DisplayName = "Component Space"),
	WorldSpace     UMETA(DisplayName = "World Space"),
	BaseBoneSpace  UMETA(DisplayName = "Base Bone Space"),
};

/** Collider limit type: push particles outside or keep inside. */
UENUM(BlueprintType)
enum class EMagicaLimitType : uint8
{
	Outer UMETA(DisplayName = "Outer (Push Out)"),
	Inner UMETA(DisplayName = "Inner (Keep Inside)"),
};

/** Where a collision limit was defined. */
UENUM()
enum class EMagicaCollisionSourceType : uint8
{
	AnimNode,
	DataAsset,
	PhysicsAsset,
};

/** Range within a flat global array. Used by TeamManager. */
USTRUCT()
struct FMagicaDataChunk
{
	GENERATED_BODY()

	int32 StartIndex = 0;
	int32 Count      = 0;

	int32 End() const { return StartIndex + Count; }
};

/** Per-chain range within a Team's particle array. */
USTRUCT()
struct FMagicaChainRange
{
	GENERATED_BODY()

	int32 StartIndex = 0;
	int32 Count      = 0;
};

/** Physics settings bundle (KawaiiPhysics style grouping). */
USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaPhysicsSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Stiffness = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendStiffness = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.01"))
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Radius = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float MaxVelocity = 5000.0f;
};

/** Root bone setting for multi-chain (KawaiiPhysics AdditionalRootBones pattern). */
USTRUCT(BlueprintType)
struct FMagicaRootBoneSetting
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Bones")
	FBoneReference RootBone;

	UPROPERTY(EditAnywhere, Category = "Bones", meta = (InlineEditConditionToggle))
	bool bUseOverrideExcludeBones = false;

	UPROPERTY(EditAnywhere, Category = "Bones", meta = (EditCondition = "bUseOverrideExcludeBones"))
	TArray<FBoneReference> OverrideExcludeBones;
};

/** Inertia parameters. */
USTRUCT(BlueprintType)
struct FMagicaInertiaParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WorldInertia = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float LocalInertia = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float TeleportDistanceThreshold = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float TeleportRotationThreshold = 10.0f;
};

/** Wind parameters. */
USTRUCT(BlueprintType)
struct FMagicaWindParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Influence = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Frequency = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Turbulence = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float DepthWeight = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float MovingWind = 0.0f;
};

/** Unique identifier for a cloth Team. */
using FMagicaTeamId = int32;
constexpr FMagicaTeamId MAGICA_INVALID_TEAM_ID = INDEX_NONE;
```

- [ ] **Step 3: Write VertexAttribute.h**

```cpp
// MagicaClothUE/Public/VirtualMesh/VertexAttribute.h
#pragma once

#include "CoreMinimal.h"

/**
 * Per-vertex attribute flags (MagicaCloth2 compatible).
 * Packed as uint8 for cache efficiency.
 */
namespace EMagicaVertexFlag
{
	constexpr uint8 None              = 0x00;
	constexpr uint8 Fixed             = 0x01;  // Pinned to animation (InvMass = 0)
	constexpr uint8 Move              = 0x02;  // Simulated particle
	constexpr uint8 InvalidMotion     = 0x08;  // Motion constraint disabled
	constexpr uint8 DisableCollision  = 0x10;  // Skip collision for this vertex
	constexpr uint8 Triangle          = 0x80;  // Has triangle connectivity
}

/** Vertex attribute with depth value. */
struct FMagicaVertexAttribute
{
	uint8 Flags = EMagicaVertexFlag::None;
	float Depth = 0.0f;  // 0.0 = root, 1.0 = tip

	bool IsFixed() const    { return (Flags & EMagicaVertexFlag::Fixed) != 0; }
	bool IsMove() const     { return (Flags & EMagicaVertexFlag::Move) != 0; }
	bool IsCollisionEnabled() const { return (Flags & EMagicaVertexFlag::DisableCollision) == 0; }
};
```

- [ ] **Step 4: Verify compile**

```bash
# From UE project that references the plugin, or check syntax:
# Verify no include errors by reading the files
```

---

## Task 2: VirtualMesh (BoneCloth Mode)

**Files:**
- Create: `MagicaClothUE/Public/VirtualMesh/VirtualMesh.h`
- Create: `MagicaClothUE/Private/VirtualMesh/VirtualMesh.cpp`

- [ ] **Step 1: Write VirtualMesh.h**

```cpp
// MagicaClothUE/Public/VirtualMesh/VirtualMesh.h
#pragma once

#include "CoreMinimal.h"
#include "VirtualMesh/VertexAttribute.h"
#include "Core/ClothTypes.h"

/**
 * CPU-side mesh representation for cloth simulation.
 * Decouples simulation topology from rendering mesh.
 *
 * BoneCloth: Each bone = 1 vertex/particle.
 * MeshCloth: Each mesh vertex = 1 particle (Phase 2).
 */
class MAGICACLOTHUE_API FMagicaVirtualMesh
{
public:
	/** Build from a bone chain (BoneCloth mode).
	 *  @param BoneTransforms World-space transforms of each bone in chain order.
	 *  @param FixedCount Number of bones at the start that are pinned (Fixed).
	 */
	void BuildFromBoneChain(const TArray<FTransform>& BoneTransforms, int32 FixedCount);

	/** Build from multiple bone chains sharing common roots (skirt mode).
	 *  @param AllTransforms [0..SharedRootCount-1] = shared roots, then concatenated chain bones.
	 *  @param SharedRootCount Number of shared root bones (all pinned).
	 *  @param ChainRanges Per-chain start/count within AllTransforms (after shared roots).
	 */
	void BuildFromMultiChain(
		const TArray<FTransform>& AllTransforms,
		int32 SharedRootCount,
		const TArray<FMagicaChainRange>& ChainRanges);

	/** Calculate depth values for all vertices. 0.0=root, 1.0=tip. */
	void CalculateDepth();

	/** Get vertex count. */
	int32 GetVertexCount() const { return Positions.Num(); }

	/** Sample a depth curve at a vertex's depth value. Returns 1.0 if curve is empty. */
	static float SampleDepthCurve(const FRuntimeFloatCurve& Curve, float Depth);

	// --- Data arrays (parallel, indexed by vertex index) ---
	TArray<FVector> Positions;              // Rest-pose positions (world space)
	TArray<FMagicaVertexAttribute> Attributes;  // Per-vertex flags + depth
	TArray<int32> ParentIndices;            // Parent vertex index (-1 = root)
	TArray<TArray<int32>> ChildIndices;     // Children per vertex

	// Rest-pose local offsets for rotation reconstruction
	TArray<FVector> RestLocalOffsets;        // Local offset from parent
	TArray<FQuat> RestLocalRotations;        // Local rotation relative to parent

	// Multi-chain info
	TArray<FMagicaChainRange> ChainRanges;
	int32 SharedRootCount = 0;
	float TotalChainLength = 0.0f;          // Sum of all edge lengths (for depth normalization)
};
```

- [ ] **Step 2: Write VirtualMesh.cpp**

```cpp
// MagicaClothUE/Private/VirtualMesh/VirtualMesh.cpp
#include "VirtualMesh/VirtualMesh.h"

void FMagicaVirtualMesh::BuildFromBoneChain(const TArray<FTransform>& BoneTransforms, int32 FixedCount)
{
	const int32 Count = BoneTransforms.Num();
	if (Count == 0) return;

	Positions.SetNum(Count);
	Attributes.SetNum(Count);
	ParentIndices.SetNum(Count);
	ChildIndices.SetNum(Count);
	RestLocalOffsets.SetNum(Count);
	RestLocalRotations.SetNum(Count);

	SharedRootCount = FixedCount;
	ChainRanges.Empty();

	for (int32 i = 0; i < Count; ++i)
	{
		Positions[i] = BoneTransforms[i].GetTranslation();

		Attributes[i].Flags = (i < FixedCount)
			? EMagicaVertexFlag::Fixed
			: EMagicaVertexFlag::Move;

		ParentIndices[i] = (i > 0) ? (i - 1) : INDEX_NONE;
		ChildIndices[i].Reset();

		if (i > 0)
		{
			ChildIndices[i - 1].Add(i);
			const FVector ParentPos = BoneTransforms[i - 1].GetTranslation();
			RestLocalOffsets[i] = BoneTransforms[i - 1].InverseTransformPosition(Positions[i]);
			RestLocalRotations[i] = BoneTransforms[i - 1].GetRotation().Inverse() * BoneTransforms[i].GetRotation();
		}
		else
		{
			RestLocalOffsets[i] = FVector::ZeroVector;
			RestLocalRotations[i] = FQuat::Identity;
		}
	}

	CalculateDepth();
}

void FMagicaVirtualMesh::BuildFromMultiChain(
	const TArray<FTransform>& AllTransforms,
	int32 InSharedRootCount,
	const TArray<FMagicaChainRange>& InChainRanges)
{
	const int32 Count = AllTransforms.Num();
	if (Count == 0) return;

	SharedRootCount = InSharedRootCount;
	ChainRanges = InChainRanges;

	Positions.SetNum(Count);
	Attributes.SetNum(Count);
	ParentIndices.SetNum(Count);
	ChildIndices.SetNum(Count);
	RestLocalOffsets.SetNum(Count);
	RestLocalRotations.SetNum(Count);

	// Shared roots are Fixed
	for (int32 i = 0; i < SharedRootCount; ++i)
	{
		Positions[i] = AllTransforms[i].GetTranslation();
		Attributes[i].Flags = EMagicaVertexFlag::Fixed;
		ParentIndices[i] = (i > 0) ? (i - 1) : INDEX_NONE;
		ChildIndices[i].Reset();
		RestLocalOffsets[i] = (i > 0)
			? AllTransforms[i - 1].InverseTransformPosition(Positions[i])
			: FVector::ZeroVector;
		RestLocalRotations[i] = (i > 0)
			? (AllTransforms[i - 1].GetRotation().Inverse() * AllTransforms[i].GetRotation())
			: FQuat::Identity;
	}

	// Per-chain particles
	for (const FMagicaChainRange& Range : ChainRanges)
	{
		for (int32 j = 0; j < Range.Count; ++j)
		{
			const int32 Idx = Range.StartIndex + j;
			Positions[Idx] = AllTransforms[Idx].GetTranslation();

			// First bone of each chain connects to last shared root
			const int32 Parent = (j == 0)
				? (SharedRootCount - 1)
				: (Idx - 1);

			ParentIndices[Idx] = Parent;
			ChildIndices[Idx].Reset();
			ChildIndices[Parent].Add(Idx);

			// First bone of each chain is Fixed, rest are Move
			Attributes[Idx].Flags = (j == 0)
				? EMagicaVertexFlag::Fixed
				: EMagicaVertexFlag::Move;

			RestLocalOffsets[Idx] = AllTransforms[Parent].InverseTransformPosition(Positions[Idx]);
			RestLocalRotations[Idx] = AllTransforms[Parent].GetRotation().Inverse() * AllTransforms[Idx].GetRotation();
		}
	}

	CalculateDepth();
}

void FMagicaVirtualMesh::CalculateDepth()
{
	const int32 Count = Positions.Num();

	// Calculate total chain length for normalization
	TotalChainLength = 0.0f;
	TArray<float> DistFromRoot;
	DistFromRoot.SetNumZeroed(Count);

	for (int32 i = 0; i < Count; ++i)
	{
		if (ParentIndices[i] != INDEX_NONE)
		{
			const float EdgeLen = FVector::Dist(Positions[i], Positions[ParentIndices[i]]);
			DistFromRoot[i] = DistFromRoot[ParentIndices[i]] + EdgeLen;
			TotalChainLength = FMath::Max(TotalChainLength, DistFromRoot[i]);
		}
	}

	// Normalize to 0..1
	for (int32 i = 0; i < Count; ++i)
	{
		Attributes[i].Depth = (TotalChainLength > UE_SMALL_NUMBER)
			? (DistFromRoot[i] / TotalChainLength)
			: 0.0f;
	}
}

float FMagicaVirtualMesh::SampleDepthCurve(const FRuntimeFloatCurve& Curve, float Depth)
{
	if (const FRichCurve* RichCurve = Curve.GetRichCurveConst())
	{
		if (RichCurve->GetNumKeys() > 0)
		{
			return RichCurve->Eval(Depth);
		}
	}
	return 1.0f;
}
```

- [ ] **Step 3: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Core/ClothTypes.h
git add MagicaClothUE/Source/MagicaClothUE/Public/VirtualMesh/VertexAttribute.h
git add MagicaClothUE/Source/MagicaClothUE/Public/VirtualMesh/VirtualMesh.h
git add MagicaClothUE/Source/MagicaClothUE/Private/VirtualMesh/VirtualMesh.cpp
git commit -m "feat: add ClothTypes, VertexAttribute, and VirtualMesh (BoneCloth)"
```

---

## Task 3: Constraint System — Base + Distance

**Files:**
- Create: `MagicaClothUE/Public/Simulation/Constraints/ConstraintBase.h`
- Create: `MagicaClothUE/Public/Simulation/Constraints/DistanceConstraint.h`
- Create: `MagicaClothUE/Private/Simulation/Constraints/DistanceConstraint.cpp`

- [ ] **Step 1: Write ConstraintBase.h**

New constraint interface that operates on raw arrays instead of TSharedPtr overhead. Each constraint stores indices into global particle arrays.

```cpp
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
```

- [ ] **Step 2: Write DistanceConstraint.h**

```cpp
// MagicaClothUE/Public/Simulation/Constraints/DistanceConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"

/** Type of distance constraint (MagicaCloth2 separation). */
enum class EMagicaDistanceType : uint8
{
	Vertical,    // Parent-child (structural)
	Horizontal,  // Same-depth neighbors (shear/ring)
	Diagonal,    // Cross-chain shear
};

/** Single distance constraint between two particles. */
struct FMagicaDistancePair
{
	int32 IndexA = INDEX_NONE;
	int32 IndexB = INDEX_NONE;
	float RestLength = 0.0f;
};

/**
 * Distance Constraint: maintains rest-length between particle pairs.
 * Supports depth-based stiffness via external curve sampling.
 */
struct MAGICACLOTHUE_API FMagicaDistanceConstraint : public FMagicaConstraintBase
{
	EMagicaDistanceType Type = EMagicaDistanceType::Vertical;
	float Stiffness = 0.8f;
	TArray<FMagicaDistancePair> Pairs;

	/** Build from VirtualMesh parent-child relationships (Vertical). */
	void BuildVertical(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices, float InStiffness);

	/** Build horizontal (ring) constraints between same-depth particles across chains. */
	void BuildHorizontal(
		const TArray<FVector>& Positions,
		const TArray<FMagicaChainRange>& ChainRanges,
		int32 SharedRootCount,
		float InStiffness,
		bool bClosedLoop);

	/** Build diagonal (shear) constraints between adjacent chains. */
	void BuildDiagonal(
		const TArray<FVector>& Positions,
		const TArray<FMagicaChainRange>& ChainRanges,
		int32 SharedRootCount,
		float InStiffness,
		bool bClosedLoop);

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
```

- [ ] **Step 3: Write DistanceConstraint.cpp**

```cpp
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

			// Forward diagonal: cur[depth] → next[depth+1]
			if (Depth < CurChain.Count && (Depth + 1) < NextChain.Count)
			{
				FMagicaDistancePair Pair;
				Pair.IndexA = CurChain.StartIndex + Depth;
				Pair.IndexB = NextChain.StartIndex + Depth + 1;
				Pair.RestLength = FVector::Dist(Positions[Pair.IndexA], Positions[Pair.IndexB]);
				Pairs.Add(Pair);
			}

			// Backward diagonal: next[depth] → cur[depth+1]
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
```

- [ ] **Step 4: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Simulation/Constraints/
git add MagicaClothUE/Source/MagicaClothUE/Private/Simulation/Constraints/DistanceConstraint.cpp
git commit -m "feat: add ConstraintBase and DistanceConstraint (V/H/Diagonal)"
```

---

## Task 4: Bending Constraint (Dihedral Angle)

**Files:**
- Create: `MagicaClothUE/Public/Simulation/Constraints/BendingConstraint.h`
- Create: `MagicaClothUE/Private/Simulation/Constraints/BendingConstraint.cpp`

- [ ] **Step 1: Write BendingConstraint.h**

```cpp
// MagicaClothUE/Public/Simulation/Constraints/BendingConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"

/** Triangle pair for dihedral angle bending (MagicaCloth2 DirectionDihedralAngle). */
struct FMagicaBendPair
{
	int32 V0 = INDEX_NONE;  // Shared edge vertex 0
	int32 V1 = INDEX_NONE;  // Shared edge vertex 1
	int32 V2 = INDEX_NONE;  // Triangle A opposite vertex
	int32 V3 = INDEX_NONE;  // Triangle B opposite vertex
	float RestAngle = 0.0f; // Rest dihedral angle (radians)
};

/**
 * Bending Constraint using Dihedral Angle method.
 *
 * For BoneCloth (chain mode): uses 3-consecutive-bone angle preservation.
 * For MeshCloth: uses triangle-pair dihedral angles (Phase 2).
 */
struct MAGICACLOTHUE_API FMagicaBendingConstraint : public FMagicaConstraintBase
{
	float Stiffness = 0.5f;
	TArray<FMagicaBendPair> Pairs;

	/** Build from bone chain: creates bend constraints for every 3 consecutive bones. */
	void BuildFromChain(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices, float InStiffness);

	/** Build from multi-chain: per-chain 3-bone bends. */
	void BuildFromMultiChain(
		const TArray<FVector>& Positions,
		const TArray<FMagicaChainRange>& ChainRanges,
		int32 SharedRootCount,
		float InStiffness);

	virtual void Solve(FMagicaParticleArrays& Particles) override;

private:
	/** Calculate dihedral angle between two triangles sharing edge (V0, V1). */
	static float CalcDihedralAngle(const FVector& V0, const FVector& V1, const FVector& V2, const FVector& V3);
};
```

- [ ] **Step 2: Write BendingConstraint.cpp**

```cpp
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
```

- [ ] **Step 3: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Simulation/Constraints/BendingConstraint.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Simulation/Constraints/BendingConstraint.cpp
git commit -m "feat: add BendingConstraint (dihedral angle for bone chains)"
```

---

## Task 5: Tether + Inertia + Angle Constraints

**Files:**
- Create: `MagicaClothUE/Public/Simulation/Constraints/TetherConstraint.h`
- Create: `MagicaClothUE/Private/Simulation/Constraints/TetherConstraint.cpp`
- Create: `MagicaClothUE/Public/Simulation/Constraints/InertiaConstraint.h`
- Create: `MagicaClothUE/Private/Simulation/Constraints/InertiaConstraint.cpp`
- Create: `MagicaClothUE/Public/Simulation/Constraints/AngleConstraint.h`
- Create: `MagicaClothUE/Private/Simulation/Constraints/AngleConstraint.cpp`

- [ ] **Step 1: Write TetherConstraint.h and .cpp**

```cpp
// MagicaClothUE/Public/Simulation/Constraints/TetherConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"

/** Tether pair: particle tethered to a root particle with max distance. */
struct FMagicaTetherPair
{
	int32 ParticleIndex = INDEX_NONE;
	int32 RootIndex = INDEX_NONE;
	float MaxDistance = 0.0f;
};

/**
 * Tether Constraint: limits maximum distance from root.
 * Prevents cloth from stretching too far.
 */
struct MAGICACLOTHUE_API FMagicaTetherConstraint : public FMagicaConstraintBase
{
	float TetherScale = 1.1f;  // MaxDistance = restDistance * TetherScale
	TArray<FMagicaTetherPair> Pairs;

	/** Build from VirtualMesh. Each non-fixed particle tethered to nearest fixed ancestor. */
	void Build(const TArray<FVector>& Positions, const TArray<int32>& ParentIndices,
		const TArray<FMagicaVertexAttribute>& Attributes, float InTetherScale);

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
```

```cpp
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
```

- [ ] **Step 2: Write InertiaConstraint.h and .cpp**

```cpp
// MagicaClothUE/Public/Simulation/Constraints/InertiaConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"
#include "Core/ClothTypes.h"

/**
 * Inertia Constraint: stabilizes cloth when character moves/rotates.
 * Handles teleport detection and world/local inertia damping.
 */
struct MAGICACLOTHUE_API FMagicaInertiaConstraint : public FMagicaConstraintBase
{
	FMagicaInertiaParams Params;

	// Updated each frame by the AnimNode
	FVector PrevOrigin = FVector::ZeroVector;
	FQuat PrevRotation = FQuat::Identity;
	FVector CurrentOrigin = FVector::ZeroVector;
	FQuat CurrentRotation = FQuat::Identity;
	bool bFirstFrame = true;
	bool bTeleported = false;

	/** Call each frame before solving to update origin tracking. */
	void UpdateOrigin(const FVector& NewOrigin, const FQuat& NewRotation);

	/** Check if teleport occurred. If so, returns true (caller should reset sim). */
	bool CheckTeleport() const { return bTeleported; }

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
```

```cpp
// MagicaClothUE/Private/Simulation/Constraints/InertiaConstraint.cpp
#include "Simulation/Constraints/InertiaConstraint.h"

void FMagicaInertiaConstraint::UpdateOrigin(const FVector& NewOrigin, const FQuat& NewRotation)
{
	PrevOrigin = CurrentOrigin;
	PrevRotation = CurrentRotation;
	CurrentOrigin = NewOrigin;
	CurrentRotation = NewRotation;

	if (bFirstFrame)
	{
		PrevOrigin = NewOrigin;
		PrevRotation = NewRotation;
		bFirstFrame = false;
		bTeleported = false;
		return;
	}

	// Teleport detection
	const float MoveDist = FVector::Dist(CurrentOrigin, PrevOrigin);
	const float RotAngle = FMath::RadiansToDegrees(CurrentRotation.AngularDistance(PrevRotation));
	bTeleported = (MoveDist > Params.TeleportDistanceThreshold) ||
	              (RotAngle > Params.TeleportRotationThreshold);
}

void FMagicaInertiaConstraint::Solve(FMagicaParticleArrays& Particles)
{
	if (bFirstFrame || bTeleported) return;

	// World Inertia: apply character movement shift to all particles
	const FVector WorldShift = CurrentOrigin - PrevOrigin;
	if (!WorldShift.IsNearlyZero() && Params.WorldInertia > 0.0f)
	{
		const FVector InertiaShift = WorldShift * Params.WorldInertia;
		for (int32 i = 0; i < Particles.PredictedPositions.Num(); ++i)
		{
			if (Particles.InvMasses[i] > 0.0f)
			{
				// Reduce shift based on depth (deeper = less inertia compensation)
				const float DepthFactor = 1.0f - Particles.Depths[i] * 0.5f;
				Particles.PredictedPositions[i] += InertiaShift * DepthFactor;
			}
		}
	}

	// Local Inertia: apply rotation damping
	const FQuat DeltaRot = CurrentRotation * PrevRotation.Inverse();
	if (!DeltaRot.IsIdentity() && Params.LocalInertia > 0.0f)
	{
		for (int32 i = 0; i < Particles.PredictedPositions.Num(); ++i)
		{
			if (Particles.InvMasses[i] > 0.0f)
			{
				const FVector RelPos = Particles.PredictedPositions[i] - CurrentOrigin;
				const FVector RotatedPos = DeltaRot.RotateVector(RelPos);
				const FVector RotShift = (RotatedPos - RelPos) * Params.LocalInertia;
				const float DepthFactor = 1.0f - Particles.Depths[i] * 0.5f;
				Particles.PredictedPositions[i] += RotShift * DepthFactor;
			}
		}
	}
}
```

- [ ] **Step 3: Write AngleConstraint.h and .cpp**

```cpp
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
```

```cpp
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
```

- [ ] **Step 4: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Simulation/Constraints/TetherConstraint.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Simulation/Constraints/TetherConstraint.cpp
git add MagicaClothUE/Source/MagicaClothUE/Public/Simulation/Constraints/InertiaConstraint.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Simulation/Constraints/InertiaConstraint.cpp
git add MagicaClothUE/Source/MagicaClothUE/Public/Simulation/Constraints/AngleConstraint.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Simulation/Constraints/AngleConstraint.cpp
git commit -m "feat: add Tether, Inertia, and Angle constraints"
```

---

## Task 6: Collider System Refactor (Limits + DataAsset)

**Files:**
- Modify: `MagicaClothUE/Public/Colliders/MagicaColliderShapes.h`
- Create: `MagicaClothUE/Public/Colliders/ColliderLimits.h`
- Create: `MagicaClothUE/Public/Colliders/MagicaLimitsDataAsset.h`
- Create: `MagicaClothUE/Private/Colliders/MagicaLimitsDataAsset.cpp`

- [ ] **Step 1: Add Plane collider and Inner/Outer to MagicaColliderShapes.h**

Add after the existing `FMagicaBoxCollider` in `MagicaColliderShapes.h`:

```cpp
/** Plane collider — prevents particles from crossing a plane. */
struct MAGICACLOTHUE_API FMagicaPlaneCollider : public FMagicaColliderShape
{
	FPlane Plane = FPlane(FVector::UpVector, 0.0f);

	FMagicaPlaneCollider()
	{
		ShapeType = EMagicaColliderShapeType::Plane;
	}

	virtual void ResolveCollision(FVector& InOutPosition) const override
	{
		const FVector WorldNormal = WorldTransform.GetRotation().RotateVector(
			FVector(Plane.X, Plane.Y, Plane.Z));
		const FVector WorldPoint = WorldTransform.GetTranslation();
		const float Dist = FVector::DotProduct(InOutPosition - WorldPoint, WorldNormal);

		if (Dist < 0.0f)
		{
			InOutPosition -= WorldNormal * Dist;
		}
	}
};
```

Also add `Plane` to the enum:

```cpp
enum class EMagicaColliderShapeType : uint8
{
	Sphere,
	Capsule,
	Box,
	Plane
};
```

And add `EMagicaLimitType LimitType = EMagicaLimitType::Outer;` to `FMagicaColliderShape` base.

- [ ] **Step 2: Write ColliderLimits.h (KawaiiPhysics-style limit structs)**

```cpp
// MagicaClothUE/Public/Colliders/ColliderLimits.h
#pragma once

#include "CoreMinimal.h"
#include "BoneContainer.h"
#include "Core/ClothTypes.h"
#include "ColliderLimits.generated.h"

/** Base struct for all collision limits (KawaiiPhysics compatible pattern). */
USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Collision")
	FBoneReference DrivingBone;

	UPROPERTY(EditAnywhere, Category = "Collision")
	FVector OffsetLocation = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Collision", meta = (ClampMin = "-360", ClampMax = "360"))
	FRotator OffsetRotation = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Category = "Collision")
	EMagicaLimitType LimitType = EMagicaLimitType::Outer;

	UPROPERTY()
	FVector Location = FVector::ZeroVector;

	UPROPERTY()
	FQuat Rotation = FQuat::Identity;

	UPROPERTY()
	bool bEnable = true;

	UPROPERTY(VisibleAnywhere, Category = "Collision")
	EMagicaCollisionSourceType SourceType = EMagicaCollisionSourceType::AnimNode;
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaSphericalLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Spherical Limit", meta = (ClampMin = "0"))
	float Radius = 5.0f;
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaCapsuleLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Capsule Limit", meta = (ClampMin = "0"))
	float Radius = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Capsule Limit", meta = (ClampMin = "0"))
	float Length = 10.0f;
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaBoxLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Box Limit")
	FVector Extent = FVector(5.0f, 5.0f, 5.0f);
};

USTRUCT(BlueprintType)
struct MAGICACLOTHUE_API FMagicaPlanarLimit : public FMagicaCollisionLimitBase
{
	GENERATED_BODY()

	UPROPERTY()
	FPlane Plane = FPlane(0, 0, 0, 0);
};
```

- [ ] **Step 3: Write MagicaLimitsDataAsset.h and .cpp**

```cpp
// MagicaClothUE/Public/Colliders/MagicaLimitsDataAsset.h
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Colliders/ColliderLimits.h"
#include "MagicaLimitsDataAsset.generated.h"

/**
 * Reusable collider configuration asset (KawaiiPhysics LimitsDataAsset pattern).
 * Store collision setup once, reference from multiple AnimBP nodes.
 */
UCLASS(BlueprintType)
class MAGICACLOTHUE_API UMagicaLimitsDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spherical Limits")
	TArray<FMagicaSphericalLimit> SphericalLimits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Capsule Limits")
	TArray<FMagicaCapsuleLimit> CapsuleLimits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Limits")
	TArray<FMagicaBoxLimit> BoxLimits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planar Limits")
	TArray<FMagicaPlanarLimit> PlanarLimits;
};
```

```cpp
// MagicaClothUE/Private/Colliders/MagicaLimitsDataAsset.cpp
#include "Colliders/MagicaLimitsDataAsset.h"
// UDataAsset has no special implementation needed for now.
```

- [ ] **Step 4: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Colliders/
git add MagicaClothUE/Source/MagicaClothUE/Private/Colliders/MagicaLimitsDataAsset.cpp
git commit -m "feat: add KawaiiPhysics-style collision limits and LimitsDataAsset"
```

---

## Task 7: PBD Solver Refactor

**Files:**
- Modify: `MagicaClothUE/Public/Simulation/FPBDSolver.h`
- Modify: `MagicaClothUE/Private/Simulation/FPBDSolver.cpp`

- [ ] **Step 1: Rewrite FPBDSolver.h**

Replace the entire file. The new solver uses `FMagicaVirtualMesh` for initialization and operates on flat arrays with the new constraint interface.

```cpp
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
```

- [ ] **Step 2: Rewrite FPBDSolver.cpp**

```cpp
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
```

- [ ] **Step 3: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Simulation/FPBDSolver.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Simulation/FPBDSolver.cpp
git commit -m "refactor: rewrite PBD solver with generic particle arrays and new constraint interface"
```

---

## Task 8: SimulationManager (Replaces FClothSimThread)

**Files:**
- Create: `MagicaClothUE/Public/Simulation/SimulationManager.h`
- Create: `MagicaClothUE/Private/Simulation/SimulationManager.cpp`
- Delete: `MagicaClothUE/Public/Simulation/FClothSimThread.h`
- Delete: `MagicaClothUE/Private/Simulation/FClothSimThread.cpp`
- Delete: `MagicaClothUE/Public/Simulation/FClothConstraints.h`
- Delete: `MagicaClothUE/Private/Simulation/FClothConstraints.cpp`

- [ ] **Step 1: Write SimulationManager.h**

```cpp
// MagicaClothUE/Public/Simulation/SimulationManager.h
#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/CriticalSection.h"
#include "Core/ClothTypes.h"
#include "Simulation/FPBDSolver.h"
#include "Simulation/FClothDoubleBuffer.h"

/**
 * Central simulation manager: single worker thread for ALL cloth Teams.
 * Replaces per-instance FClothSimThread.
 */
class MAGICACLOTHUE_API FMagicaSimulationManager : public FRunnable
{
public:
	FMagicaSimulationManager();
	virtual ~FMagicaSimulationManager() override;

	void Start();
	void Stop();
	bool IsRunning() const { return bRunning.load(std::memory_order_acquire); }

	// --- FRunnable ---
	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Exit() override;

	// --- Team Management ---
	struct FTeamData
	{
		FMagicaTeamId TeamId = MAGICA_INVALID_TEAM_ID;
		FMagicaPBDSolver Solver;
		FClothDoubleBuffer DoubleBuffer;

		// Pending input (written by game thread, read by sim thread)
		TArray<FVector> PendingAnimPositions;
		TArray<FTransform> PendingColliderTransforms;
		bool bResetRequested = false;
		TArray<FVector> PendingResetPositions;
	};

	/** Register a new Team. Returns TeamId. Caller must configure Solver before calling Start(). */
	FMagicaTeamId RegisterTeam();

	/** Unregister a Team. */
	void UnregisterTeam(FMagicaTeamId TeamId);

	/** Get Team data (for configuration). NOT thread-safe — call before Start() or use locks. */
	FTeamData* GetTeam(FMagicaTeamId TeamId);

	/** Feed animation positions to a Team (game thread). */
	void UpdateAnimPositions(FMagicaTeamId TeamId, const TArray<FVector>& Positions);

	/** Feed collider transforms to a Team (game thread). */
	void UpdateColliderTransforms(FMagicaTeamId TeamId, const TArray<FTransform>& Transforms);

	/** Request simulation reset for a Team (game thread). */
	void RequestReset(FMagicaTeamId TeamId, const TArray<FVector>& RestPositions);

	/** Read results from a Team's double buffer (game thread). */
	const TArray<FTransform>& ReadResults(FMagicaTeamId TeamId) const;

	/** Get interpolation alpha for smooth blending. */
	float GetInterpolationAlpha() const;

	// --- Configuration ---
	float TargetHz = 90.0f;
	float MinHz = 30.0f;
	int32 MaxStepsPerFrame = 3;

	void SetTargetHz(float Hz);

private:
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bRunning{false};

	TMap<FMagicaTeamId, TUniquePtr<FTeamData>> Teams;
	FCriticalSection TeamsCS;

	int32 NextTeamId = 0;
	double LastSimTime = 0.0;
	float CurrentAlpha = 0.0f;

	// Empty buffer for invalid reads
	static const TArray<FTransform> EmptyTransforms;
};
```

- [ ] **Step 2: Write SimulationManager.cpp**

```cpp
// MagicaClothUE/Private/Simulation/SimulationManager.cpp
#include "Simulation/SimulationManager.h"
#include "HAL/PlatformProcess.h"
#include "Colliders/MagicaColliderShapes.h"

const TArray<FTransform> FMagicaSimulationManager::EmptyTransforms;

FMagicaSimulationManager::FMagicaSimulationManager() = default;

FMagicaSimulationManager::~FMagicaSimulationManager()
{
	Stop();
}

void FMagicaSimulationManager::Start()
{
	if (bRunning.load()) return;
	bRunning.store(true, std::memory_order_release);
	LastSimTime = FPlatformTime::Seconds();
	Thread = FRunnableThread::Create(this, TEXT("MagicaClothSimulation"), 0, TPri_Normal);
}

void FMagicaSimulationManager::Stop()
{
	bRunning.store(false, std::memory_order_release);
	if (Thread)
	{
		Thread->WaitForCompletion();
		delete Thread;
		Thread = nullptr;
	}
}

bool FMagicaSimulationManager::Init()
{
	return true;
}

uint32 FMagicaSimulationManager::Run()
{
	const float SimDt = 1.0f / TargetHz;

	while (bRunning.load(std::memory_order_acquire))
	{
		const double FrameStart = FPlatformTime::Seconds();

		{
			FScopeLock Lock(&TeamsCS);

			for (auto& Pair : Teams)
			{
				FTeamData& Team = *Pair.Value;

				// Handle reset
				if (Team.bResetRequested)
				{
					Team.Solver.Reset(Team.PendingResetPositions);
					Team.bResetRequested = false;
					continue;
				}

				// Feed animation positions
				if (Team.PendingAnimPositions.Num() > 0)
				{
					Team.Solver.UpdateFixedParticles(Team.PendingAnimPositions);
				}

				// Update collider transforms
				const int32 ColliderCount = FMath::Min(
					Team.PendingColliderTransforms.Num(),
					Team.Solver.Colliders.Num());
				for (int32 c = 0; c < ColliderCount; ++c)
				{
					Team.Solver.Colliders[c]->WorldTransform = Team.PendingColliderTransforms[c];
				}

				// Step simulation
				Team.Solver.Step(SimDt);

				// Write results to double buffer
				// Convert positions to transforms for bone override
				TArray<FTransform> Results;
				const auto& Positions = Team.Solver.GetPositions();
				Results.SetNum(Positions.Num());
				for (int32 i = 0; i < Positions.Num(); ++i)
				{
					Results[i].SetTranslation(Positions[i]);
					Results[i].SetRotation(FQuat::Identity);
					Results[i].SetScale3D(FVector::OneVector);
				}
				Team.DoubleBuffer.Write(Results);
			}
		}

		// Timing
		const double FrameEnd = FPlatformTime::Seconds();
		const double Elapsed = FrameEnd - FrameStart;
		const double SleepTime = SimDt - Elapsed;
		CurrentAlpha = static_cast<float>(Elapsed / SimDt);

		if (SleepTime > 0.0)
		{
			FPlatformProcess::Sleep(static_cast<float>(SleepTime));
		}
	}

	return 0;
}

void FMagicaSimulationManager::Exit()
{
}

FMagicaTeamId FMagicaSimulationManager::RegisterTeam()
{
	FScopeLock Lock(&TeamsCS);
	const FMagicaTeamId Id = NextTeamId++;
	auto TeamData = MakeUnique<FTeamData>();
	TeamData->TeamId = Id;
	Teams.Add(Id, MoveTemp(TeamData));
	return Id;
}

void FMagicaSimulationManager::UnregisterTeam(FMagicaTeamId TeamId)
{
	FScopeLock Lock(&TeamsCS);
	Teams.Remove(TeamId);
}

FMagicaSimulationManager::FTeamData* FMagicaSimulationManager::GetTeam(FMagicaTeamId TeamId)
{
	FScopeLock Lock(&TeamsCS);
	auto* Found = Teams.Find(TeamId);
	return Found ? Found->Get() : nullptr;
}

void FMagicaSimulationManager::UpdateAnimPositions(FMagicaTeamId TeamId, const TArray<FVector>& Positions)
{
	FScopeLock Lock(&TeamsCS);
	if (auto* Found = Teams.Find(TeamId))
	{
		(*Found)->PendingAnimPositions = Positions;
	}
}

void FMagicaSimulationManager::UpdateColliderTransforms(FMagicaTeamId TeamId, const TArray<FTransform>& Transforms)
{
	FScopeLock Lock(&TeamsCS);
	if (auto* Found = Teams.Find(TeamId))
	{
		(*Found)->PendingColliderTransforms = Transforms;
	}
}

void FMagicaSimulationManager::RequestReset(FMagicaTeamId TeamId, const TArray<FVector>& RestPositions)
{
	FScopeLock Lock(&TeamsCS);
	if (auto* Found = Teams.Find(TeamId))
	{
		(*Found)->bResetRequested = true;
		(*Found)->PendingResetPositions = RestPositions;
	}
}

const TArray<FTransform>& FMagicaSimulationManager::ReadResults(FMagicaTeamId TeamId) const
{
	// No lock needed — DoubleBuffer.Read() is lock-free
	const auto* Found = Teams.Find(TeamId);
	if (Found)
	{
		return (*Found)->DoubleBuffer.Read();
	}
	return EmptyTransforms;
}

float FMagicaSimulationManager::GetInterpolationAlpha() const
{
	return FMath::Clamp(CurrentAlpha, 0.0f, 1.0f);
}

void FMagicaSimulationManager::SetTargetHz(float Hz)
{
	TargetHz = FMath::Clamp(Hz, MinHz, 120.0f);
}
```

- [ ] **Step 3: Delete old files**

```bash
rm "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Public/Simulation/FClothSimThread.h"
rm "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Private/Simulation/FClothSimThread.cpp"
rm "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Public/Simulation/FClothConstraints.h"
rm "D:/AI_document/physics_plugins/MagicaClothUE/Source/MagicaClothUE/Private/Simulation/FClothConstraints.cpp"
```

- [ ] **Step 4: Commit**

```bash
git add -A MagicaClothUE/Source/MagicaClothUE/Public/Simulation/
git add -A MagicaClothUE/Source/MagicaClothUE/Private/Simulation/
git commit -m "feat: add SimulationManager, remove old FClothSimThread and FClothConstraints"
```

---

## Task 9: TeamManager + Subsystem

**Files:**
- Create: `MagicaClothUE/Public/Core/TeamManager.h`
- Create: `MagicaClothUE/Private/Core/TeamManager.cpp`
- Create: `MagicaClothUE/Public/Core/MagicaClothSubsystem.h`
- Create: `MagicaClothUE/Private/Core/MagicaClothSubsystem.cpp`

- [ ] **Step 1: Write TeamManager.h and .cpp**

```cpp
// MagicaClothUE/Public/Core/TeamManager.h
#pragma once

#include "CoreMinimal.h"
#include "Core/ClothTypes.h"
#include "TeamManager.generated.h"

class FMagicaSimulationManager;

/**
 * Manages cloth Team registration and lifecycle.
 * Thin wrapper around SimulationManager for team bookkeeping.
 */
UCLASS()
class MAGICACLOTHUE_API UMagicaTeamManager : public UObject
{
	GENERATED_BODY()

public:
	void Initialize(FMagicaSimulationManager* InSimManager);

	FMagicaTeamId CreateTeam();
	void DestroyTeam(FMagicaTeamId TeamId);
	int32 GetActiveTeamCount() const { return ActiveTeamIds.Num(); }

private:
	FMagicaSimulationManager* SimManager = nullptr;
	TSet<FMagicaTeamId> ActiveTeamIds;
};
```

```cpp
// MagicaClothUE/Private/Core/TeamManager.cpp
#include "Core/TeamManager.h"
#include "Simulation/SimulationManager.h"

void UMagicaTeamManager::Initialize(FMagicaSimulationManager* InSimManager)
{
	SimManager = InSimManager;
}

FMagicaTeamId UMagicaTeamManager::CreateTeam()
{
	if (!SimManager) return MAGICA_INVALID_TEAM_ID;

	const FMagicaTeamId Id = SimManager->RegisterTeam();
	ActiveTeamIds.Add(Id);
	return Id;
}

void UMagicaTeamManager::DestroyTeam(FMagicaTeamId TeamId)
{
	if (!SimManager) return;

	SimManager->UnregisterTeam(TeamId);
	ActiveTeamIds.Remove(TeamId);
}
```

- [ ] **Step 2: Write MagicaClothSubsystem.h and .cpp**

```cpp
// MagicaClothUE/Public/Core/MagicaClothSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Core/ClothTypes.h"
#include "MagicaClothSubsystem.generated.h"

class FMagicaSimulationManager;
class UMagicaTeamManager;

/**
 * World Subsystem: one per world, coordinates all cloth simulation.
 * Owns the SimulationManager (worker thread) and TeamManager.
 */
UCLASS()
class MAGICACLOTHUE_API UMagicaClothSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	FMagicaSimulationManager* GetSimulationManager() const { return SimulationManager.Get(); }
	UMagicaTeamManager* GetTeamManager() const { return TeamManager; }

	/** Start the simulation thread (called when first Team is registered). */
	void EnsureSimulationRunning();

private:
	TUniquePtr<FMagicaSimulationManager> SimulationManager;

	UPROPERTY()
	TObjectPtr<UMagicaTeamManager> TeamManager;

	bool bSimulationStarted = false;
};
```

```cpp
// MagicaClothUE/Private/Core/MagicaClothSubsystem.cpp
#include "Core/MagicaClothSubsystem.h"
#include "Core/TeamManager.h"
#include "Simulation/SimulationManager.h"

void UMagicaClothSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	SimulationManager = MakeUnique<FMagicaSimulationManager>();

	TeamManager = NewObject<UMagicaTeamManager>(this);
	TeamManager->Initialize(SimulationManager.Get());
}

void UMagicaClothSubsystem::Deinitialize()
{
	if (SimulationManager)
	{
		SimulationManager->Stop();
	}

	Super::Deinitialize();
}

void UMagicaClothSubsystem::EnsureSimulationRunning()
{
	if (!bSimulationStarted && SimulationManager)
	{
		SimulationManager->Start();
		bSimulationStarted = true;
	}
}
```

- [ ] **Step 3: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/Core/TeamManager.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Core/TeamManager.cpp
git add MagicaClothUE/Source/MagicaClothUE/Public/Core/MagicaClothSubsystem.h
git add MagicaClothUE/Source/MagicaClothUE/Private/Core/MagicaClothSubsystem.cpp
git commit -m "feat: add TeamManager and MagicaClothSubsystem (UWorldSubsystem)"
```

---

## Task 10: AnimNode Refactor (KawaiiPhysics Style)

**Files:**
- Modify: `MagicaClothUE/Public/AnimNode/AnimNode_MagicaCloth.h` (full rewrite)
- Modify: `MagicaClothUE/Private/AnimNode/AnimNode_MagicaCloth.cpp` (full rewrite)

This is the largest task. The AnimNode is rewritten to use the new Subsystem/Team architecture with KawaiiPhysics-style parameter categories.

- [ ] **Step 1: Rewrite AnimNode_MagicaCloth.h**

Replace the entire file with the new KawaiiPhysics-style parameter layout. Full code at: the spec section 8.1 defines the category structure. The header should include all UPROPERTY declarations with proper meta tags, the new constraint/VirtualMesh/Subsystem integration, and PhysicsAsset + DataAsset + inline 3-way collider sourcing.

Key changes from old:
- Remove: `FClothSimThread`, old Stiffness/BendStiffness/Damping/Gravity/MaxVelocity as individual properties
- Add: `FMagicaPhysicsSettings PhysicsSettings` struct, `EMagicaClothType ClothType`, `FRuntimeFloatCurve` depth curves, `FMagicaInertiaParams`, limit arrays, DataAsset refs, `FGameplayTag`
- Replace: Individual chain arrays with `FMagicaVirtualMesh` usage
- Add: `FMagicaTeamId` for Subsystem registration

The file is too large to include inline in the plan. The implementer should follow the spec section 8.1 parameter structure exactly, using the existing `AnimNode_MagicaCloth.h` as a starting skeleton and the KawaiiPhysics `AnimNode_KawaiiPhysics.h` as the UX reference. The key interface methods remain the same: `InitializeBoneReferences`, `Initialize_AnyThread`, `EvaluateSkeletalControl_AnyThread`, `IsValidToEvaluate`.

- [ ] **Step 2: Rewrite AnimNode_MagicaCloth.cpp**

The implementation should:
1. In `InitializeBoneReferences`: Build bone chain/multi-chain as before.
2. In `EvaluateSkeletalControl_AnyThread`:
   - On first frame: Get `UMagicaClothSubsystem`, create Team, build `FMagicaVirtualMesh`, initialize solver, build all constraints (Distance V/H, Bending, Tether, Inertia, Angle), build colliders from 3 sources.
   - Each frame: Feed anim positions and collider transforms via Subsystem, read DoubleBuffer results, interpolate, output bone transforms.
3. Physics Asset collider extraction: Reuse existing `BuildPhysicsAssetColliders` logic, extend to also merge inline limits and DataAsset limits.
4. Depth curve application: For each constraint, apply `FMagicaVirtualMesh::SampleDepthCurve()` to modulate stiffness.

- [ ] **Step 3: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/Public/AnimNode/AnimNode_MagicaCloth.h
git add MagicaClothUE/Source/MagicaClothUE/Private/AnimNode/AnimNode_MagicaCloth.cpp
git commit -m "refactor: rewrite AnimNode with KawaiiPhysics-style params and Subsystem integration"
```

---

## Task 11: AnimGraphNode + Editor Updates

**Files:**
- Modify: `MagicaClothUEEditor/Public/AnimGraphNode_MagicaCloth.h`
- Modify: `MagicaClothUEEditor/Private/AnimGraphNode_MagicaCloth.cpp`

- [ ] **Step 1: Add debug toggle properties and detail customization**

Add to `AnimGraphNode_MagicaCloth.h`:

```cpp
// Debug visualization toggles (stored on graph node, not runtime node)
UPROPERTY() bool bEnableDebugDrawBone = true;
UPROPERTY() bool bEnableDebugDrawSphereLimit = true;
UPROPERTY() bool bEnableDebugDrawCapsuleLimit = true;
UPROPERTY() bool bEnableDebugDrawBoxLimit = true;
UPROPERTY() bool bEnableDebugDrawPlanarLimit = true;
UPROPERTY() bool bEnableDebugDrawConstraint = true;
UPROPERTY() bool bEnableDebugDrawWind = false;

// Detail customization
virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
```

- [ ] **Step 2: Implement CustomizeDetails in .cpp**

Add category reordering and debug toggle buttons following KawaiiPhysics pattern.

- [ ] **Step 3: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUEEditor/
git commit -m "feat: add debug toggles and detail customization to AnimGraphNode"
```

---

## Task 12: Build System Updates

**Files:**
- Modify: `MagicaClothUE/MagicaClothUE.Build.cs`
- Modify: `MagicaClothUE/MagicaClothComponent.h` and `.cpp`

- [ ] **Step 1: Update Build.cs dependencies**

Add `StructUtils` for `FInstancedStruct` (external forces) and `GameplayTags`:

```csharp
PublicDependencyModuleNames.AddRange(new string[]
{
    "Core",
    "CoreUObject",
    "Engine",
    "AnimGraphRuntime",
    "PhysicsCore",
    "StructUtils",
    "GameplayTags"
});
```

- [ ] **Step 2: Update MagicaClothComponent to use Subsystem**

Refactor `BeginPlay` to register with `UMagicaClothSubsystem` and `EndPlay` to unregister.

- [ ] **Step 3: Compile verification**

```bash
# Open the UE project containing MagicaClothUE plugin and compile
# Expected: Clean compile with no errors
```

- [ ] **Step 4: Commit**

```bash
git add MagicaClothUE/Source/MagicaClothUE/MagicaClothUE.Build.cs
git add MagicaClothUE/Source/MagicaClothUE/Public/MagicaClothComponent.h
git add MagicaClothUE/Source/MagicaClothUE/Private/MagicaClothComponent.cpp
git commit -m "chore: update build deps and refactor MagicaClothComponent for Subsystem"
```

---

## Task 13: Integration Smoke Test

- [ ] **Step 1: Create test AnimBP setup**

In a UE project with the plugin:
1. Import a skeletal mesh with a skirt bone chain
2. Create an AnimBP
3. Add `Magica Cloth Simulation` node
4. Set `ClothType = BoneCloth`
5. Set `RootBone` to skirt root
6. Enable `bMultiChainMode`
7. Set `PhysicsAssetForLimits` to the mesh's PhAT
8. Play in editor

Expected: Skirt bones should simulate with gravity, constrained by PhAT colliders on legs.

- [ ] **Step 2: Verify debug visualization**

Enable `bShowDebug`. Expected: Red spheres for fixed bones, green for free, white lines for constraints, wireframe for colliders.

- [ ] **Step 3: Verify no frame drops**

Monitor frame time while running. Simulation should not cause frame drops > 1ms on game thread.

- [ ] **Step 4: Commit any fixes from testing**

```bash
git add -A MagicaClothUE/
git commit -m "fix: integration test fixes for Phase 1"
```

---

## Summary

| Task | Description | Files |
|------|-------------|-------|
| 1 | Foundation types | ClothTypes.h, VertexAttribute.h |
| 2 | VirtualMesh (BoneCloth) | VirtualMesh.h/.cpp |
| 3 | Distance Constraint | ConstraintBase.h, DistanceConstraint.h/.cpp |
| 4 | Bending Constraint | BendingConstraint.h/.cpp |
| 5 | Tether + Inertia + Angle | 6 files |
| 6 | Collider Limits + DataAsset | ColliderLimits.h, MagicaLimitsDataAsset.h/.cpp |
| 7 | PBD Solver refactor | FPBDSolver.h/.cpp |
| 8 | SimulationManager | SimulationManager.h/.cpp, delete old files |
| 9 | TeamManager + Subsystem | 4 files |
| 10 | AnimNode refactor | AnimNode_MagicaCloth.h/.cpp |
| 11 | AnimGraphNode editor | AnimGraphNode_MagicaCloth.h/.cpp |
| 12 | Build system + Component | Build.cs, MagicaClothComponent.h/.cpp |
| 13 | Integration smoke test | Runtime verification |
