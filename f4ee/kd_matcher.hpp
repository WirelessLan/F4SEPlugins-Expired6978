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
	using point_ref = std::pair<Morpher::Vector3*, std::size_t>;
	using match = std::pair<point_ref, point_ref>;

	namespace detail {
		struct kd_node {
			point_ref p;
			kd_node* less;
			kd_node* more;

			kd_node() : p(nullptr, 0), less(nullptr), more(nullptr) {}
			explicit kd_node(point_ref point) : p(std::move(point)), less(nullptr), more(nullptr) {}

			point_ref add(const point_ref& point, int depth, std::vector<kd_node>& nodes) {
				int axis = depth % 3;
				bool domore = false;

				float dx = p.first->x - point.first->x;
				float dy = p.first->y - point.first->y;
				float dz = p.first->z - point.first->z;

				if (std::fabs(dx) < EPSILON && std::fabs(dy) < EPSILON && std::fabs(dz) < EPSILON) {
					return p;
				}

				switch (axis) {
				case 0:
					if (dx > 0) domore = true;
					break;
				case 1:
					if (dy > 0) domore = true;
					break;
				case 2:
					if (dz > 0) domore = true;
					break;
				}

				if (domore) {
					if (more) return more->add(point, depth + 1, nodes);
					nodes.emplace_back(point);
					more = &nodes.back();
				}
				else {
					if (less) return less->add(point, depth + 1, nodes);
					nodes.emplace_back(point);
					less = &nodes.back();
				}

				return point_ref(nullptr, 0);
			}
		};
	}

	inline std::vector<match> find_matches(std::vector<Morpher::Vector3>& points) {
		if (points.empty()) {
			return {};
		}

		std::vector<match> matches;
		std::vector<detail::kd_node> nodes;

		matches.reserve(points.size() - 1);
		nodes.reserve(static_cast<std::size_t>(points.size()));
		nodes.emplace_back(point_ref(&points[0], 0));
		auto* root = &nodes.back();

		for (std::size_t ii = 1; ii < points.size(); ++ii) {
			point_ref point(&points[ii], ii);
			point_ref pong = root->add(point, 0, nodes);
			if (pong.first) {
				matches.emplace_back(point, pong);
			}
		}

		return matches;
	}
}
