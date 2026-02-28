#pragma once

#include <givr.h>
#include <optional>

namespace skinning {
	struct SkinnedModel {
		// Bone used in the model given by name (from file) and length
		struct Bone {
			std::string name	= "";
			float length		= 0.f;
		};

		// Face that indexes the vertices to connect
		struct Face {
			int va_id = -1;
			int vb_id = -1;
			int vc_id = -1;
		};

		// Vertex with a rest position and a series of weights for relevent bones
		struct Vertex {
			struct BoneWeight {
				int bone_id = -1;
				float w		= 0.f;
			};

			glm::vec3 rest_pos						= { 0.f, 0.f, 0.f };
			std::vector<BoneWeight> bone_weights	= {};
		};

		givr::geometry::TriangleSoup makeMesh() const;
		void updateMesh(givr::geometry::TriangleSoup& mesh) const;
		
		// Static Mesh loader. Doesnt use standard obj, but a 
		// custom format in a txt (a script for generating
		// this from blender will be provided)
		static std::optional<SkinnedModel> loadFromFile(std::string const& filepath);

		// Data
		std::vector<Bone> bones;
		std::vector<Face> faces;
		std::vector<Vertex> vertices;
	};
} // namespace skinning