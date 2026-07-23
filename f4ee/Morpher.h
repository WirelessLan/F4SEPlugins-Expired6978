#pragma once

#include "shape.hpp"
#include "kd_matcher.hpp"

#include <vector>
#include <functional>

class BSTriShape;

namespace Morpher {
	bool HasRequiredMorphFlags(UInt64 vertexDesc);
	void ApplyMorph(BSTriShape* geometry, UInt8* srcBlock, UInt8* dstBlock, const std::function<void(std::vector<Morpher::Vector3>&)>& morph);
}
