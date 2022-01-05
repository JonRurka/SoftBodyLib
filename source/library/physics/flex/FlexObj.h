#pragma once

#include "physics/Physics.h"

namespace SoftBodyLib {

	/// Texture coordinates for old-style actor body (the "cab")
	struct CabTexcoord {
		int node_id;
		float texcoord_u;
		float texcoord_v;
	};

	/// Submesh for old-style actor body (the "cab")
	struct CabSubmesh
	{
		enum BackmeshType
		{
			BACKMESH_NONE,
			BACKMESH_OPAQUE,
			BACKMESH_TRANSPARENT
		};

		BackmeshType  backmesh_type{ BACKMESH_NONE };
		size_t        texcoords_pos{ 0 };
		size_t        cabs_pos{ 0 };
	};

	/// A visual mesh, forming a chassis for softbody actor
	/// At most one instance is created per actor.
	class FlexObj
	{
	public:
		FlexObj(
			node_t* all_nodes,// For initial setup only, pointer is not stored
			std::vector<CabSubmesh>& texcoords,
			int numtriangles,
			int* triangles,
			std::vector<CabSubmesh>& submeshes,
			char* texname,
			const char* name,
			char* backtexname,
			char* transtexname
		);

		~FlexObj();

		glm::vec3	UpdateFlexObj();
		void		ScaleFlexObj(float factor);

	private:
		struct FlexObjVertex {
			glm::vec3 position;
			glm::vec3 normal;
			glm::vec2 texcoord;
		};

		/// Compute vertex position in the vertexbuffer (0-based offset) for node `v` of triangle `tidx`
		int			ComputeVertexPos(int tidx, int v, std::vector<CabSubmesh>& submeshes);
		glm::vec3	UpdateMesh();

		// Ogre::MeshPtr               m_mesh;
		// std::vector<Ogre::SubMesh*> m_submeshes;
		// RoR::GfxActor*              m_gfx_actor;
		float*						m_s_ref;

		size_t                      m_vertex_count;
		int*						m_vertex_nodes;
		// Ogre::VertexDeclaration*    m_vertex_format;
		// Ogre::HardwareVertexBufferSharedPtr m_hw_vbuf;

		union
		{
			float*			m_vertices_raw;
			FlexObjVertex*	m_vertices;
		};

		size_t                      m_index_count;
		unsigned short*				m_indices;
		int                         m_triangle_count;
	};
}