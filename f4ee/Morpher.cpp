#include "Morpher.h"

#include <cmath>
#undef min
#undef max
#include "half.hpp"
#include "f4se/BSGeometry.h"

namespace {
	float round_v(float num) {
		return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
	}

	struct MorphBuffers {
		std::vector<Morpher::Vector3> vertices;
		std::vector<Morpher::Vector3> normals;
		std::vector<Morpher::Vector2> uvs;
		std::vector<Morpher::Vector3> tangents;
		std::vector<Morpher::Vector3> bitangents;

		explicit MorphBuffers(std::size_t numVertices) : vertices(numVertices), normals(numVertices), uvs(numVertices), tangents(numVertices), bitangents(numVertices) {}
	};

	void RecalcNormals(MorphBuffers& buffers, UInt32 numTriangles, Morpher::Triangle* triangles, bool smooth = true, float smoothThres = 60.0f) {
		const auto numVertices = buffers.vertices.size();

		std::vector<Morpher::Vector3> verts(numVertices);
		std::vector<Morpher::Vector3> norms(numVertices);

		for (std::size_t i = 0; i < numVertices; ++i) {
			verts[i].x = buffers.vertices[i].x * -0.1f;
			verts[i].z = buffers.vertices[i].y * 0.1f;
			verts[i].y = buffers.vertices[i].z * 0.1f;
		}

		// Face normals
		Morpher::Vector3 tn;
		for (UInt32 t = 0; t < numTriangles; ++t) {
			triangles[t].trinormal(verts, &tn);
			norms[triangles[t].p1] += tn;
			norms[triangles[t].p2] += tn;
			norms[triangles[t].p3] += tn;
		}

		for (auto& n : norms) {
			n.Normalize();
		}

		// Smooth normals
		if (smooth) {
			kd_matcher::for_each_match(verts, [&](std::size_t aIndex, std::size_t bIndex) {
				Morpher::Vector3& an = norms[aIndex];
				Morpher::Vector3& bn = norms[bIndex];

				if (an.angle(bn) < smoothThres * DEG2RAD) {
					const Morpher::Vector3 anTemp = an;
					an += bn;
					bn += anTemp;
				}
			});

			for (auto& n : norms) {
				n.Normalize();
			}
		}

		for (std::size_t i = 0; i < numVertices; ++i) {
			buffers.normals[i].x = -norms[i].x;
			buffers.normals[i].y = norms[i].z;
			buffers.normals[i].z = norms[i].y;
		}
	}

	void CalcTangentSpace(MorphBuffers& buffers, UInt32 numTriangles, Morpher::Triangle* triangles) {
		const auto numVertices = buffers.vertices.size();

		std::vector<Morpher::Vector3> tan1(numVertices);
		std::vector<Morpher::Vector3> tan2(numVertices);

		for (UInt32 i = 0; i < numTriangles; ++i) {
			const int i1 = triangles[i].p1;
			const int i2 = triangles[i].p2;
			const int i3 = triangles[i].p3;

			const Morpher::Vector3& v1 = buffers.vertices[i1];
			const Morpher::Vector3& v2 = buffers.vertices[i2];
			const Morpher::Vector3& v3 = buffers.vertices[i3];

			const Morpher::Vector2& w1 = buffers.uvs[i1];
			const Morpher::Vector2& w2 = buffers.uvs[i2];
			const Morpher::Vector2& w3 = buffers.uvs[i3];

			float x1 = v2.x - v1.x;
			float x2 = v3.x - v1.x;
			float y1 = v2.y - v1.y;
			float y2 = v3.y - v1.y;
			float z1 = v2.z - v1.z;
			float z2 = v3.z - v1.z;

			float s1 = w2.u - w1.u;
			float s2 = w3.u - w1.u;
			float t1 = w2.v - w1.v;
			float t2 = w3.v - w1.v;

			float r = (s1 * t2 - s2 * t1);
			r = (r >= 0.0f ? +1.0f : -1.0f);

			Morpher::Vector3 sdir = Morpher::Vector3((t2 * x1 - t1 * x2) * r, (t2 * y1 - t1 * y2) * r, (t2 * z1 - t1 * z2) * r);
			Morpher::Vector3 tdir = Morpher::Vector3((s1 * x2 - s2 * x1) * r, (s1 * y2 - s2 * y1) * r, (s1 * z2 - s2 * z1) * r);

			sdir.Normalize();
			tdir.Normalize();

			tan1[i1] += tdir;
			tan1[i2] += tdir;
			tan1[i3] += tdir;

			tan2[i1] += sdir;
			tan2[i2] += sdir;
			tan2[i3] += sdir;
		}

		for (std::size_t i = 0; i < numVertices; ++i) {
			buffers.tangents[i] = tan1[i];
			buffers.bitangents[i] = tan2[i];

			if (buffers.tangents[i].IsZero() || buffers.bitangents[i].IsZero()) {
				buffers.tangents[i].x = buffers.normals[i].y;
				buffers.tangents[i].y = buffers.normals[i].z;
				buffers.tangents[i].z = buffers.normals[i].x;
				buffers.bitangents[i] = buffers.normals[i].cross(buffers.tangents[i]);
			}
			else {
				buffers.tangents[i].Normalize();
				buffers.tangents[i] = (buffers.tangents[i] - buffers.normals[i] * buffers.normals[i].dot(buffers.tangents[i]));
				buffers.tangents[i].Normalize();

				buffers.bitangents[i].Normalize();
				buffers.bitangents[i] = (buffers.bitangents[i] - buffers.normals[i] * buffers.normals[i].dot(buffers.bitangents[i]));
				buffers.bitangents[i] = (buffers.bitangents[i] - buffers.tangents[i] * buffers.tangents[i].dot(buffers.bitangents[i]));
				buffers.bitangents[i].Normalize();
			}
		}
	}
}

namespace Morpher {
	bool HasRequiredMorphFlags(UInt64 vertexDesc) {
		constexpr UInt64 kRequiredMorphFlags = BSGeometry::kFlag_Vertex | BSTriShape::kFlag_UVs | BSTriShape::kFlag_Normals | BSTriShape::kFlag_Tangents;
		return (vertexDesc & kRequiredMorphFlags) == kRequiredMorphFlags;
	}

	void ApplyMorph(BSTriShape* geometry, UInt8* srcBlock, UInt8* dstBlock, const std::function<void(std::vector<Morpher::Vector3>&)>& morph) {
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
				buffers.vertices[i].x = (*(float *)vBegin); vBegin += 4;
				buffers.vertices[i].y = (*(float *)vBegin); vBegin += 4;
				buffers.vertices[i].z = (*(float *)vBegin); vBegin += 4;

				vBegin += 4; // Skip BitangetX
			}
			else
			{
				buffers.vertices[i].x = (*(half_float::half *)vBegin); vBegin += 2;
				buffers.vertices[i].y = (*(half_float::half *)vBegin); vBegin += 2;
				buffers.vertices[i].z = (*(half_float::half *)vBegin); vBegin += 2;

				vBegin += 2; // Skip BitangetX
			}

			if ((vertexDesc & BSTriShape::kFlag_UVs) == BSTriShape::kFlag_UVs)
			{
				buffers.uvs[i].u = (*(half_float::half *)vBegin); vBegin += 2;
				buffers.uvs[i].v = (*(half_float::half *)vBegin); vBegin += 2;
			}
		}

		morph(buffers.vertices);

		Morpher::Triangle* triangles = reinterpret_cast<Morpher::Triangle*>(geomData->triangleData->triangles);
		RecalcNormals(buffers, geometry->numTriangles, triangles);
		CalcTangentSpace(buffers, geometry->numTriangles, triangles);

		vertexBlock = dstBlock ? dstBlock : geomData->vertexData->vertexBlock;
		for (UInt32 i = 0; i < numVertices; ++i)
		{
			UInt8* vBegin = &vertexBlock[i * vertexSize];

			if ((vertexDesc & BSTriShape::kFlag_FullPrecision) == BSTriShape::kFlag_FullPrecision)
			{
				(*(float *)vBegin) = buffers.vertices[i].x; vBegin += 4;
				(*(float *)vBegin) = buffers.vertices[i].y; vBegin += 4;
				(*(float *)vBegin) = buffers.vertices[i].z; vBegin += 4;
				(*(float *)vBegin) = buffers.bitangents[i].x; vBegin += 4;
			}
			else
			{
				(*(half_float::half *)vBegin) = buffers.vertices[i].x; vBegin += 2;
				(*(half_float::half *)vBegin) = buffers.vertices[i].y; vBegin += 2;
				(*(half_float::half *)vBegin) = buffers.vertices[i].z; vBegin += 2;
				(*(half_float::half *)vBegin) = buffers.bitangents[i].x; vBegin += 2;
			}

			// Skip UV write
			if ((vertexDesc & BSTriShape::kFlag_UVs) == BSTriShape::kFlag_UVs)
			{
				vBegin += 4;
			}

			if ((vertexDesc & BSTriShape::kFlag_Normals) == BSTriShape::kFlag_Normals)
			{
				*(SInt8*)vBegin = (UInt8)round_v((((buffers.normals[i].x + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				*(SInt8*)vBegin = (UInt8)round_v((((buffers.normals[i].y + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				*(SInt8*)vBegin = (UInt8)round_v((((buffers.normals[i].z + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				*(SInt8*)vBegin = (UInt8)round_v((((buffers.bitangents[i].y + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;

				if ((vertexDesc & BSTriShape::kFlag_Tangents) == BSTriShape::kFlag_Tangents)
				{
					*(SInt8*)vBegin = (UInt8)round_v((((buffers.tangents[i].x + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
					*(SInt8*)vBegin = (UInt8)round_v((((buffers.tangents[i].y + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
					*(SInt8*)vBegin = (UInt8)round_v((((buffers.tangents[i].z + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
					*(SInt8*)vBegin = (UInt8)round_v((((buffers.bitangents[i].z + 1.0f) / 2.0f) * 255.0f)); vBegin += 1;
				}
			}
		}
	}
}
