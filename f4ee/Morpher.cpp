#include "Morpher.h"

#include <cmath>
#include <vector>
#undef min
#undef max
#include "half.hpp"
#include "kd_matcher.hpp"

#include "f4se/BSGeometry.h"

namespace {
	float round_v(float num) {
		return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
	}

	struct MorphBuffers {
		static constexpr std::size_t kBufferCount = 5;

		std::size_t numVertices;
		std::vector<Morpher::Vector3> storage;

		explicit MorphBuffers(std::size_t count) : numVertices(count), storage(count * kBufferCount) {}

		Morpher::Vector3* vertices() {
			return storage.data();
		}

		Morpher::Vector3& vertex(std::size_t index) {
			return storage[index];
		}

		Morpher::Vector3& uv(std::size_t index) {
			return storage[numVertices + index];
		}

		Morpher::Vector3& transformedVertex(std::size_t index) {
			return storage[numVertices * 2 + index];
		}

		Morpher::Vector3& tangent(std::size_t index) {
			return transformedVertex(index);
		}

		Morpher::Vector3& normal(std::size_t index) {
			return storage[numVertices * 3 + index];
		}

		Morpher::Vector3& bitangent(std::size_t index) {
			return bitangents()[index];
		}

		Morpher::Vector3* bitangents() {
			return numVertices == 0 ? nullptr : &storage[numVertices * 4];
		}

		void beginTangentCalculation() {
			for (std::size_t i = 0; i < numVertices; ++i) {
				tangent(i).Zero();
			}
		}
	};

	void RecalcNormals(MorphBuffers& buffers, UInt32 numTriangles, Morpher::Triangle* triangles, bool smooth = true, float smoothThres = 60.0f) {
		const auto numVertices = buffers.numVertices;

		for (std::size_t i = 0; i < numVertices; ++i) {
			Morpher::Vector3& transformedVertex = buffers.transformedVertex(i);
			transformedVertex.x = buffers.vertex(i).x * -0.1f;
			transformedVertex.z = buffers.vertex(i).y * 0.1f;
			transformedVertex.y = buffers.vertex(i).z * 0.1f;
		}

		// Face normals
		for (UInt32 t = 0; t < numTriangles; ++t) {
			const Morpher::Triangle& triangle = triangles[t];
			const Morpher::Vector3& p1 = buffers.transformedVertex(triangle.p1);
			const Morpher::Vector3& p2 = buffers.transformedVertex(triangle.p2);
			const Morpher::Vector3& p3 = buffers.transformedVertex(triangle.p3);

			Morpher::Vector3 triangleNormal;
			triangleNormal.x = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
			triangleNormal.y = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
			triangleNormal.z = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

			buffers.normal(triangle.p1) += triangleNormal;
			buffers.normal(triangle.p2) += triangleNormal;
			buffers.normal(triangle.p3) += triangleNormal;
		}

		for (std::size_t i = 0; i < numVertices; ++i) {
			buffers.normal(i).Normalize();
		}

		// Smooth normals
		if (smooth) {
			kd_matcher::for_each_match(
				numVertices,
				[&buffers](std::size_t index) -> const Morpher::Vector3& {
					return buffers.transformedVertex(index);
				},
				buffers.bitangents(),
				[&buffers, smoothThres](std::size_t aIndex, std::size_t bIndex) {
					Morpher::Vector3& an = buffers.normal(aIndex);
					Morpher::Vector3& bn = buffers.normal(bIndex);

					if (an.angle(bn) < smoothThres * DEG2RAD) {
						const Morpher::Vector3 anTemp = an;
						an += bn;
						bn += anTemp;
					}
				}
			);

			for (std::size_t i = 0; i < numVertices; ++i) {
				buffers.normal(i).Normalize();
			}
		}

		for (std::size_t i = 0; i < numVertices; ++i) {
			Morpher::Vector3& outputNormal = buffers.normal(i);
			const Morpher::Vector3 normal = outputNormal;
			outputNormal.x = -normal.x;
			outputNormal.y = normal.z;
			outputNormal.z = normal.y;
		}
	}

	void CalcTangentSpace(MorphBuffers& buffers, UInt32 numTriangles, Morpher::Triangle* triangles) {
		const auto numVertices = buffers.numVertices;

		buffers.beginTangentCalculation();

		for (UInt32 i = 0; i < numTriangles; ++i) {
			const int i1 = triangles[i].p1;
			const int i2 = triangles[i].p2;
			const int i3 = triangles[i].p3;

			const Morpher::Vector3& v1 = buffers.vertex(i1);
			const Morpher::Vector3& v2 = buffers.vertex(i2);
			const Morpher::Vector3& v3 = buffers.vertex(i3);

			const Morpher::Vector3& w1 = buffers.uv(i1);
			const Morpher::Vector3& w2 = buffers.uv(i2);
			const Morpher::Vector3& w3 = buffers.uv(i3);

			float x1 = v2.x - v1.x;
			float x2 = v3.x - v1.x;
			float y1 = v2.y - v1.y;
			float y2 = v3.y - v1.y;
			float z1 = v2.z - v1.z;
			float z2 = v3.z - v1.z;

			float s1 = w2.x - w1.x;
			float s2 = w3.x - w1.x;
			float t1 = w2.y - w1.y;
			float t2 = w3.y - w1.y;

			float r = (s1 * t2 - s2 * t1);
			r = (r >= 0.0f ? +1.0f : -1.0f);

			Morpher::Vector3 sdir = Morpher::Vector3((t2 * x1 - t1 * x2) * r, (t2 * y1 - t1 * y2) * r, (t2 * z1 - t1 * z2) * r);
			Morpher::Vector3 tdir = Morpher::Vector3((s1 * x2 - s2 * x1) * r, (s1 * y2 - s2 * y1) * r, (s1 * z2 - s2 * z1) * r);

			sdir.Normalize();
			tdir.Normalize();

			buffers.tangent(i1) += tdir;
			buffers.tangent(i2) += tdir;
			buffers.tangent(i3) += tdir;

			buffers.bitangent(i1) += sdir;
			buffers.bitangent(i2) += sdir;
			buffers.bitangent(i3) += sdir;
		}

		for (std::size_t i = 0; i < numVertices; ++i) {
			Morpher::Vector3& tangent = buffers.tangent(i);
			Morpher::Vector3& bitangent = buffers.bitangent(i);
			Morpher::Vector3& normal = buffers.normal(i);

			if (tangent.IsZero() || bitangent.IsZero()) {
				tangent.x = normal.y;
				tangent.y = normal.z;
				tangent.z = normal.x;

				bitangent = normal.cross(tangent);
			}
			else {
				tangent.Normalize();
				tangent = tangent - normal * normal.dot(tangent);
				tangent.Normalize();

				bitangent.Normalize();
				bitangent = bitangent - normal * normal.dot(bitangent);
				bitangent = bitangent - tangent * tangent.dot(bitangent);
				bitangent.Normalize();
			}
		}
	}
}

namespace Morpher {
	bool HasRequiredMorphFlags(UInt64 vertexDesc) {
		constexpr UInt64 kRequiredMorphFlags = BSGeometry::kFlag_Vertex | BSTriShape::kFlag_UVs | BSTriShape::kFlag_Normals | BSTriShape::kFlag_Tangents;
		return (vertexDesc & kRequiredMorphFlags) == kRequiredMorphFlags;
	}

	void ApplyMorph(BSTriShape* geometry, UInt8* srcBlock, UInt8* dstBlock, const std::function<void(Morpher::Vector3*, std::size_t)>& morph) {
		UInt64 vertexDesc = geometry->vertexDesc;
		UInt32 vertexSize = geometry->GetVertexSize();
		BSGeometryData* geomData = geometry->geometryData;
		UInt32 numVertices = geometry->numVertices;

		MorphBuffers buffers(numVertices);

		UInt8* vertexBlock = srcBlock ? srcBlock : geomData->vertexData->vertexBlock;
		for (UInt32 i = 0; i < numVertices; ++i)
		{
			UInt8* vBegin = &vertexBlock[i * vertexSize];

			if ((vertexDesc & BSTriShape::kFlag_FullPrecision) == BSTriShape::kFlag_FullPrecision)
			{
				buffers.vertex(i).x = (*(float *)vBegin); vBegin += 4;
				buffers.vertex(i).y = (*(float *)vBegin); vBegin += 4;
				buffers.vertex(i).z = (*(float *)vBegin); vBegin += 4;

				vBegin += 4; // Skip BitangetX
			}
			else
			{
				buffers.vertex(i).x = (*(half_float::half *)vBegin); vBegin += 2;
				buffers.vertex(i).y = (*(half_float::half *)vBegin); vBegin += 2;
				buffers.vertex(i).z = (*(half_float::half *)vBegin); vBegin += 2;

				vBegin += 2; // Skip BitangetX
			}

			if ((vertexDesc & BSTriShape::kFlag_UVs) == BSTriShape::kFlag_UVs)
			{
				buffers.uv(i).x = (*(half_float::half *)vBegin); vBegin += 2;
				buffers.uv(i).y = (*(half_float::half *)vBegin); vBegin += 2;
			}
		}

		morph(buffers.vertices(), numVertices);

		Morpher::Triangle* triangles = reinterpret_cast<Morpher::Triangle*>(geomData->triangleData->triangles);
		RecalcNormals(buffers, geometry->numTriangles, triangles);
		CalcTangentSpace(buffers, geometry->numTriangles, triangles);

		vertexBlock = dstBlock ? dstBlock : geomData->vertexData->vertexBlock;
		for (UInt32 i = 0; i < numVertices; ++i)
		{
			UInt8* vBegin = &vertexBlock[i * vertexSize];
			const Morpher::Vector3& vertex = buffers.vertex(i);
			const Morpher::Vector3& normal = buffers.normal(i);
			const Morpher::Vector3& tangent = buffers.tangent(i);
			const Morpher::Vector3& bitangent = buffers.bitangent(i);

			if ((vertexDesc & BSTriShape::kFlag_FullPrecision) == BSTriShape::kFlag_FullPrecision)
			{
				(*(float *)vBegin) = vertex.x; vBegin += 4;
				(*(float *)vBegin) = vertex.y; vBegin += 4;
				(*(float *)vBegin) = vertex.z; vBegin += 4;
				(*(float *)vBegin) = bitangent.x; vBegin += 4;
			}
			else
			{
				(*(half_float::half *)vBegin) = vertex.x; vBegin += 2;
				(*(half_float::half *)vBegin) = vertex.y; vBegin += 2;
				(*(half_float::half *)vBegin) = vertex.z; vBegin += 2;
				(*(half_float::half *)vBegin) = bitangent.x; vBegin += 2;
			}

			// Skip UV write
			if ((vertexDesc & BSTriShape::kFlag_UVs) == BSTriShape::kFlag_UVs)
			{
				vBegin += 4;
			}

			if ((vertexDesc & BSTriShape::kFlag_Normals) == BSTriShape::kFlag_Normals)
			{
				*(SInt8*)vBegin = (UInt8)round_v((((normal.x + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				*(SInt8*)vBegin = (UInt8)round_v((((normal.y + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				*(SInt8*)vBegin = (UInt8)round_v((((normal.z + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				*(SInt8*)vBegin = (UInt8)round_v((((bitangent.y + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;

				if ((vertexDesc & BSTriShape::kFlag_Tangents) == BSTriShape::kFlag_Tangents)
				{
					*(SInt8*)vBegin = (UInt8)round_v((((tangent.x + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
					*(SInt8*)vBegin = (UInt8)round_v((((tangent.y + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
					*(SInt8*)vBegin = (UInt8)round_v((((tangent.z + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
					*(SInt8*)vBegin = (UInt8)round_v((((bitangent.z + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				}
			}
		}
	}
}
