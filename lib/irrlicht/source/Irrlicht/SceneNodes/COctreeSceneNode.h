// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#ifndef __C_OCTREE_SCENE_NODE_H_INCLUDED__
#define __C_OCTREE_SCENE_NODE_H_INCLUDED__

#include <SceneNodes/IMeshSceneNode.h>
#include "../Octree.h"

namespace irr
{
namespace scene
{
	//! implementation of the IBspTreeSceneNode
	class COctreeSceneNode : public IMeshSceneNode
	{
	public:

		//! constructor
		COctreeSceneNode(ISceneNode* parent, ISceneManager* mgr, s32 id,
			s32 minimalPolysPerNode=512);

		//! destructor
		virtual ~COctreeSceneNode();

		virtual void OnRegisterSceneNode() OVERRIDE;

		//! renders the node.
		virtual void render() OVERRIDE;

		//! returns the axis aligned bounding box of this node
		virtual const core::aabbox3d<f32>& getBoundingBox() const OVERRIDE;

		//! creates the tree
		bool createTree(IMesh* mesh);

		//! returns the material based on the zero based index i. To get the amount
		//! of materials used by this scene node, use getMaterialCount().
		//! This function is needed for inserting the node into the scene hirachy on a
		//! optimal position for minimizing renderstate changes, but can also be used
		//! to directly modify the material of a scene node.
		virtual video::SMaterial& getMaterial(u32 i) OVERRIDE;

		//! returns amount of materials used by this scene node.
		virtual u32 getMaterialCount() const OVERRIDE;

		//! Returns type of the scene node
		virtual ESCENE_NODE_TYPE getType() const OVERRIDE { return ESNT_OCTREE; }

		//! Sets a new mesh to display
		virtual void setMesh(IMesh* mesh) OVERRIDE;

		//! Get the currently defined mesh for display.
		virtual IMesh* getMesh(void) OVERRIDE;

		//! Sets if the scene node should not copy the materials of the mesh but use them in a read only style.
		virtual void setReadOnlyMaterials(bool readonly) OVERRIDE;

		//! Check if the scene node should not copy the materials of the mesh but use them in a read only style
		virtual bool isReadOnlyMaterials() const OVERRIDE;

		//! Removes a child from this scene node.
		//! Implemented here, to be able to remove the shadow properly, if there is one,
		//! or to remove attached childs.
		virtual bool removeChild(ISceneNode* child) OVERRIDE;

	private:

		void deleteTree();

		core::aabbox3d<f32> Box;

		Octree<video::S3DVertex>* StdOctree;
		core::array< Octree<video::S3DVertex>::SMeshChunk > StdMeshes;

		Octree<video::S3DVertex2TCoords>* LightMapOctree;
		core::array< Octree<video::S3DVertex2TCoords>::SMeshChunk > LightMapMeshes;

		Octree<video::S3DVertexTangents>* TangentsOctree;
		core::array< Octree<video::S3DVertexTangents>::SMeshChunk > TangentsMeshes;

		video::E_VERTEX_TYPE VertexType;
		core::array< video::SMaterial > Materials;

		core::stringc MeshName;
		s32 MinimalPolysPerNode;
		s32 PassCount;

		IMesh * Mesh;
		//! use VBOs for rendering where possible
		bool UseVBOs;
		//! use visibility information together with VBOs
		bool UseVisibilityAndVBOs;
		//! use bounding box or frustum for calculate polys
		bool BoxBased;
	};

} // end namespace scene
} // end namespace irr

#endif

