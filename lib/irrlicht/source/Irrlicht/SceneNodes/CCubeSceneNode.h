// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#ifndef __C_CUBE_SCENE_NODE_H_INCLUDED__
#define __C_CUBE_SCENE_NODE_H_INCLUDED__

#include <SceneNodes/IMeshSceneNode.h>
#include "SMesh.h"

namespace irr
{
namespace scene
{
	class CCubeSceneNode : public IMeshSceneNode
	{
	public:

		//! constructor
		CCubeSceneNode(f32 size, ISceneNode* parent, ISceneManager* mgr, s32 id,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f));

		virtual ~CCubeSceneNode();

		virtual void OnRegisterSceneNode() OVERRIDE;

		//! renders the node.
		virtual void render() OVERRIDE;

		//! returns the axis aligned bounding box of this node
		virtual const core::aabbox3d<f32>& getBoundingBox() const OVERRIDE;

		//! returns the material based on the zero based index i. To get the amount
		//! of materials used by this scene node, use getMaterialCount().
		//! This function is needed for inserting the node into the scene hirachy on a
		//! optimal position for minimizing renderstate changes, but can also be used
		//! to directly modify the material of a scene node.
		virtual video::SMaterial& getMaterial(u32 i) OVERRIDE;

		//! returns amount of materials used by this scene node.
		virtual u32 getMaterialCount() const OVERRIDE;

		//! Returns type of the scene node
		virtual ESCENE_NODE_TYPE getType() const OVERRIDE { return ESNT_CUBE; }

		//! Creates a clone of this scene node and its children.
		virtual ISceneNode* clone(ISceneNode* newParent = 0, ISceneManager* newManager = 0) OVERRIDE;

		//! Sets a new mesh to display
		virtual void setMesh(IMesh* mesh) OVERRIDE {}

		//! Returns the current mesh
		virtual IMesh* getMesh(void) OVERRIDE { return Mesh; }

		//! Sets if the scene node should not copy the materials of the mesh but use them in a read only style.
		/* In this way it is possible to change the materials a mesh causing all mesh scene nodes 
		referencing this mesh to change too. */
		virtual void setReadOnlyMaterials(bool readonly) OVERRIDE {}

		//! Returns if the scene node should not copy the materials of the mesh but use them in a read only style
		virtual bool isReadOnlyMaterials() const OVERRIDE { return false; }

		//! Removes a child from this scene node.
		//! Implemented here, to be able to remove the shadow properly, if there is one,
		//! or to remove attached childs.
		virtual bool removeChild(ISceneNode* child) OVERRIDE;

	private:
		void setSize();

		IMesh* Mesh;
		f32 Size;
	};

} // end namespace scene
} // end namespace irr

#endif

