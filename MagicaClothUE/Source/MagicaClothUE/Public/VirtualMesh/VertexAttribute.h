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
