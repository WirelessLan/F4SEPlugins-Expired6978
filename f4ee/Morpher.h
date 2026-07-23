#pragma once

#include "shape.hpp"

#include <cstddef>
#include <functional>

class BSTriShape;

namespace Morpher {
	bool HasRequiredMorphFlags(UInt64 vertexDesc);
	void ApplyMorph(BSTriShape* geometry, UInt8* srcBlock, UInt8* dstBlock, const std::function<void(Morpher::Vector3*, std::size_t)>& morph);
}
