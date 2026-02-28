#include "skinned_model.hpp"

namespace skinning {

	givr::geometry::TriangleSoup SkinnedModel::makeMesh() const {
		givr::geometry::TriangleSoup mesh;
		updateMesh(mesh); //Default rest geometry
		return mesh;
	}

	void SkinnedModel::updateMesh(givr::geometry::TriangleSoup& mesh) const {	
		mesh.triangles().clear();
		for (const Face& f : faces) { //Uses the rest pose for all vertices
			const Vertex& a = vertices[f.va_id];
			const Vertex& b = vertices[f.vb_id];
			const Vertex& c = vertices[f.vc_id];
			mesh.push_back(
				givr::geometry::Triangle(
					givr::geometry::Point1(a.rest_pos),
					givr::geometry::Point2(b.rest_pos),
					givr::geometry::Point3(c.rest_pos)
				)
			);
		};
	}

	std::optional<SkinnedModel> SkinnedModel::loadFromFile(std::string const& filepath) {
		std::ifstream file(filepath);
		if (!file) return {};

		SkinnedModel model;

		// load vertices
		size_t n_vertices;
		file >> n_vertices;
		model.vertices.resize(n_vertices);
		for (Vertex& v : model.vertices)
			file >> v.rest_pos.x >> v.rest_pos.y >> v.rest_pos.z;

		// load faces
		size_t n_faces;
		file >> n_faces;
		model.faces.resize(n_faces);
		for (Face& f : model.faces)
			file >> f.va_id >> f.vb_id >> f.vc_id;

		// load bones
		size_t n_bones;
		file >> n_bones;
		model.bones.resize(n_bones);
		for (Bone& bone : model.bones)
			file >> bone.name >> bone.length;

		// load weights
		for (size_t bone_id = 0; bone_id < model.bones.size(); bone_id++) {
			size_t n_bone_vert_weights;
			file >> n_bone_vert_weights;
			for (size_t index = 0; index < n_bone_vert_weights; index++) {
				size_t vertex_id;
				file >> vertex_id;

				float w;
				file >> w;

				std::string name;
				file >> name; // not used, need to be clear from the buffer though

				auto& vertex_weights = model.vertices[vertex_id].bone_weights;

				Vertex::BoneWeight bw;
				bw.w = w;
				bw.bone_id = bone_id;

				vertex_weights.push_back(bw);
			}
		}

		return model;
	}

} // namespace skinning