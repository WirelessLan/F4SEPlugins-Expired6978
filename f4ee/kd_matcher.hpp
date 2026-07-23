/*
BodySlide and Outfit Studio
Copyright (C) 2017  Caliente & ousnius
See the included LICENSE file
*/
#pragma once

#include "shape.hpp"

#include <cmath>
#include <cstddef>
#include <new>
#include <type_traits>
#include <utility>

// A specialized KD tree that finds duplicate vertices in a point cloud.  
namespace kd_matcher {
	namespace detail {
		using index_type = UInt32;

		constexpr index_type invalidIndex = static_cast<index_type>(-1);

		struct kd_node {
			index_type pointIndex;
			index_type less = invalidIndex;
			index_type more = invalidIndex;

			explicit kd_node(index_type index) noexcept : pointIndex(index) {}
		};
	}

	template <class PointAccessor, class Visitor>
	void for_each_match(std::size_t a_pointCount, const PointAccessor& a_pointAt, Morpher::Vector3* a_nodeStorage, Visitor&& a_visitor) {
		if (a_pointCount < 2) {
			return;
		}

		const auto createNode = [&a_nodeStorage](std::size_t nodeIndex, std::size_t pointIndex) {
			::new (static_cast<void*>(&a_nodeStorage[nodeIndex])) detail::kd_node(static_cast<detail::index_type>(pointIndex));
		};

		const auto nodeAt = [&a_nodeStorage](std::size_t index) -> detail::kd_node& {
			return *reinterpret_cast<detail::kd_node*>(static_cast<void*>(&a_nodeStorage[index]));
		};

		std::size_t nodeCount = 1;
		createNode(0, 0);

		for (std::size_t pointIndex = 1; pointIndex < a_pointCount; ++pointIndex) {
			detail::index_type nodeIndex = 0;
			UInt8 axis = 0;

			while (true) {
				detail::kd_node& node = nodeAt(nodeIndex);
				const detail::index_type existingPointIndex = node.pointIndex;

				const Morpher::Vector3& existingPoint = a_pointAt(existingPointIndex);
				const Morpher::Vector3& point = a_pointAt(pointIndex);

				const float dx = existingPoint.x - point.x;
				const float dy = existingPoint.y - point.y;
				const float dz = existingPoint.z - point.z;

				if (dx > -EPSILON && dx < EPSILON && dy > -EPSILON && dy < EPSILON && dz > -EPSILON && dz < EPSILON) {
					a_visitor(pointIndex, static_cast<std::size_t>(existingPointIndex));
					break;
				}

				float axisDifference;

				switch (axis) {
				case 0:
					axisDifference = dx;
					break;

				case 1:
					axisDifference = dy;
					break;

				default:
					axisDifference = dz;
					break;
				}

				const bool goMore = axisDifference > 0.0f;
				const detail::index_type childIndex = goMore ? node.more : node.less;

				if (childIndex != detail::invalidIndex) {
					nodeIndex = childIndex;
					axis = axis == 2 ? static_cast<UInt8>(0) : static_cast<UInt8>(axis + 1);
					continue;
				}

				const detail::index_type newNodeIndex = static_cast<detail::index_type>(nodeCount);
				createNode(nodeCount++, pointIndex);

				if (goMore) {
					node.more = newNodeIndex;
				}
				else {
					node.less = newNodeIndex;
				}

				break;
			}
		}

		for (std::size_t nodeIndex = 0; nodeIndex < nodeCount; ++nodeIndex) {
			::new (static_cast<void*>(&a_nodeStorage[nodeIndex])) Morpher::Vector3();
		}
	}

	static_assert(sizeof(Morpher::Vector3) == sizeof(detail::kd_node), "Vector3 and kd_node must have the same size");
	static_assert(alignof(Morpher::Vector3) == alignof(detail::kd_node), "Vector3 and kd_node must have the same alignment");
	static_assert(std::is_trivially_destructible<Morpher::Vector3>::value, "Vector3 must be trivially destructible");
	static_assert(std::is_trivially_destructible<detail::kd_node>::value, "kd_node must be trivially destructible");
}
