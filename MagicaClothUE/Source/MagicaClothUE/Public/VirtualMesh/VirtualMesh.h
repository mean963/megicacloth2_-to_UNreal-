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
