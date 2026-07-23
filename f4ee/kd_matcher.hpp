/*
BodySlide and Outfit Studio
Copyright (C) 2017  Caliente & ousnius
See the included LICENSE file
*/
#pragma once

#include "shape.hpp"

#include <cmath>
#include <cstddef>
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

	template <class Visitor>
	void for_each_match(const std::vector<Morpher::Vector3>& a_points, Visitor&& a_visitor) {
		if (a_points.size() < 2) {
			return;
		}

		std::vector<detail::kd_node> nodes;
		nodes.reserve(a_points.size());
		nodes.emplace_back(0);

		for (std::size_t pointIndex = 1; pointIndex < a_points.size(); ++pointIndex) {
			detail::index_type nodeIndex = 0;
			UInt8 axis = 0;

			while (true) {
				const detail::index_type existingPointIndex = nodes[nodeIndex].pointIndex;

				const Morpher::Vector3& existingPoint = a_points[existingPointIndex];
				const Morpher::Vector3& point = a_points[pointIndex];

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
				const detail::index_type childIndex = goMore ? nodes[nodeIndex].more : nodes[nodeIndex].less;

				if (childIndex != detail::invalidIndex) {
					nodeIndex = childIndex;
					axis = axis == 2 ? static_cast<UInt8>(0) : static_cast<UInt8>(axis + 1);
					continue;
				}

				const detail::index_type newNodeIndex = static_cast<detail::index_type>(nodes.size());
				nodes.emplace_back(static_cast<detail::index_type>(pointIndex));

				if (goMore) {
					nodes[nodeIndex].more = newNodeIndex;
				}
				else {
					nodes[nodeIndex].less = newNodeIndex;
				}

				break;
			}
		}
	}
}
