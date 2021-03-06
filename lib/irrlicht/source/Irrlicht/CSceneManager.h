// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#ifndef __C_SCENE_MANAGER_H_INCLUDED__
#define __C_SCENE_MANAGER_H_INCLUDED__

#include <Core/ISceneManager.h>
#include <Core/ISceneNode.h>
#include "ICursorControl.h"
#include "irrString.h"
#include "irrArray.h"
#include "IMeshLoader.h"
#include "CAttributes.h"
#include "ILightManager.h"

namespace irr
{
namespace io
{
	class IXMLWriter;
	class IFileSystem;
}
namespace scene
{
	class IMeshCache;
	class IGeometryCreator;

	/*!
		The Scene Manager manages scene nodes, mesh recources, cameras and all the other stuff.
	*/
	class CSceneManager : public ISceneManager, public ISceneNode
	{
	public:

		//! constructor
		CSceneManager(video::IVideoDriver* driver, io::IFileSystem* fs,
			gui::ICursorControl* cursorControl, IMeshCache* cache = 0,
			gui::IGUIEnvironment *guiEnvironment = 0);

		//! destructor
		virtual ~CSceneManager() OVERRIDE;

		//! gets an animateable mesh. loads it if needed. returned pointer must not be dropped.
		virtual IAnimatedMesh* getMesh(const io::path& filename) OVERRIDE;

		//! gets an animateable mesh. loads it if needed. returned pointer must not be dropped.
		virtual IAnimatedMesh* getMesh(io::IReadFile* file) OVERRIDE;

		//! Returns an interface to the mesh cache which is shared beween all existing scene managers.
		virtual IMeshCache* getMeshCache() OVERRIDE;

		//! returns the video driver
		virtual video::IVideoDriver* getVideoDriver() OVERRIDE;

		//! return the gui environment
		virtual gui::IGUIEnvironment* getGUIEnvironment() OVERRIDE;

		//! return the filesystem
		virtual io::IFileSystem* getFileSystem() OVERRIDE;

		//! adds a cube scene node to the scene. It is a simple cube of (1,1,1) size.
		//! the returned pointer must not be dropped.
		virtual IMeshSceneNode* addCubeSceneNode(f32 size=10.0f, ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0, 0, 0), const core::vector3df& rotation = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f)) OVERRIDE;

		//! Adds a sphere scene node to the scene.
		virtual IMeshSceneNode* addSphereSceneNode(f32 radius=5.0f, s32 polyCount=16, ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f)) OVERRIDE;

		//! adds a scene node for rendering an animated mesh model
		virtual IAnimatedMeshSceneNode* addAnimatedMeshSceneNode(IAnimatedMesh* mesh, ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f),
			bool alsoAddIfMeshPointerZero = false) OVERRIDE;

		//! adds a scene node for rendering a static mesh
		//! the returned pointer must not be dropped.
		virtual IMeshSceneNode* addMeshSceneNode(IMesh* mesh, ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f),
			bool alsoAddIfMeshPointerZero = false) OVERRIDE;

		//! Adds a scene node for rendering a animated water surface mesh.
		virtual ISceneNode* addWaterSurfaceSceneNode(IMesh* mesh, f32 waveHeight, f32 waveSpeed, f32 wlenght, ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f)) OVERRIDE;

		//! renders the node.
		virtual void render() OVERRIDE;

		//! returns the axis aligned bounding box of this node
		virtual const core::aabbox3d<f32>& getBoundingBox() const OVERRIDE;

		//! registers a node for rendering it at a specific time.
		virtual u32 registerNodeForRendering(ISceneNode* node, E_SCENE_NODE_RENDER_PASS pass = ESNRP_AUTOMATIC) OVERRIDE;

		//! draws all scene nodes
		virtual void drawAll(u32 flags = 0xFFFFFFFF) OVERRIDE;

		//! Adds a scene node for rendering using a octree to the scene graph. This a good method for rendering
		//! scenes with lots of geometry. The Octree is built on the fly from the mesh, much
		//! faster then a bsp tree.
		virtual IMeshSceneNode* addOctreeSceneNode(IAnimatedMesh* mesh, ISceneNode* parent=0,
			s32 id = -1, s32 minimalPolysPerNode = 512, bool alsoAddIfMeshPointerZero = false) OVERRIDE;

		//! Adss a scene node for rendering using a octree. This a good method for rendering
		//! scenes with lots of geometry. The Octree is built on the fly from the mesh, much
		//! faster then a bsp tree.
		virtual IMeshSceneNode* addOctreeSceneNode(IMesh* mesh, ISceneNode* parent=0,
			s32 id = -1, s32 minimalPolysPerNode = 128, bool alsoAddIfMeshPointerZero = false) OVERRIDE;

		//! Adds a camera scene node to the tree and sets it as active camera.
		//! \param position: Position of the space relative to its parent where the camera will be placed.
		//! \param lookat: Position where the camera will look at. Also known as target.
		//! \param parent: Parent scene node of the camera. Can be null. If the parent moves,
		//! the camera will move too.
		//! \return Pointer to interface to camera
		virtual ICameraSceneNode* addCameraSceneNode(ISceneNode* parent = 0,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& lookat = core::vector3df(0,0,100),
			s32 id = -1, bool makeActive = true) OVERRIDE;

		//! Adds a dynamic light scene node. The light will cast dynamic light on all
		//! other scene nodes in the scene, which have the material flag video::MTF_LIGHTING
		//! turned on. (This is the default setting in most scene nodes).
		virtual ILightSceneNode* addLightSceneNode(ISceneNode* parent = 0,
			const core::vector3df& position = core::vector3df(0,0,0),
			video::SColorf color = video::SColorf(1.0f, 1.0f, 1.0f),
			f32 range = 100.0f, s32 id = -1) OVERRIDE;

		//! Adds a billboard scene node to the scene. A billboard is like a 3d sprite: A 2d element,
		//! which always looks to the camera. It is usually used for things like explosions, fire,
		//! lensflares and things like that.
		virtual IBillboardSceneNode* addBillboardSceneNode(ISceneNode* parent = 0,
			const core::dimension2d<f32>& size = core::dimension2d<f32>(10.0f, 10.0f),
			const core::vector3df& position = core::vector3df(0,0,0), s32 id=-1,
			video::SColor shadeTop = 0xFFFFFFFF, video::SColor shadeBottom = 0xFFFFFFFF) OVERRIDE;

		//! Adds a skybox scene node. A skybox is a big cube with 6 textures on it and
		//! is drawn around the camera position.
		virtual ISceneNode* addSkyBoxSceneNode(video::ITexture* top, video::ITexture* bottom,
			video::ITexture* left, video::ITexture* right, video::ITexture* front,
			video::ITexture* back, ISceneNode* parent = 0, s32 id = -1) OVERRIDE;

		//! Adds a text scene node, which is able to display
		//! 2d text at a position in three dimensional space
		virtual ITextSceneNode* addTextSceneNode(gui::IGUIFont* font, const wchar_t* text,
			video::SColor color=video::SColor(100,255,255,255),
			ISceneNode* parent = 0,	const core::vector3df& position = core::vector3df(0,0,0),
			s32 id = -1) OVERRIDE;

		//! Adds a text scene node, which uses billboards
		virtual IBillboardTextSceneNode* addBillboardTextSceneNode(gui::IGUIFont* font, const wchar_t* text,
			ISceneNode* parent = 0,
			const core::dimension2d<f32>& size = core::dimension2d<f32>(10.0f, 10.0f),
			const core::vector3df& position = core::vector3df(0,0,0), s32 id=-1,
			video::SColor colorTop = 0xFFFFFFFF, video::SColor colorBottom = 0xFFFFFFFF) OVERRIDE;


		//! Adds a Hill Plane mesh to the mesh pool. The mesh is
		//! generated on the fly and looks like a plane with some hills
		//! on it. You can specify how many hills should be on the plane
		//! and how high they should be. Also you must specify a name
		//! for the mesh because the mesh is added to the mesh pool and
		//! can be retrieved back using ISceneManager::getMesh with the
		//! name as parameter.
		virtual IAnimatedMesh* addHillPlaneMesh(const io::path& name,
			const core::dimension2d<f32>& tileSize, const core::dimension2d<u32>& tileCount,
			video::SMaterial* material = 0,	f32 hillHeight = 0.0f,
			const core::dimension2d<f32>& countHills = core::dimension2d<f32>(1.0f, 1.0f),
			const core::dimension2d<f32>& textureRepeatCount = core::dimension2d<f32>(1.0f, 1.0f)) OVERRIDE;

		//! Adds a terrain mesh to the mesh pool.
		virtual IAnimatedMesh* addTerrainMesh(const io::path& meshname,	video::IImage* texture, video::IImage* heightmap,
			const core::dimension2d<f32>& stretchSize = core::dimension2d<f32>(10.0f,10.0f),
			f32 maxHeight=200.0f,
			const core::dimension2d<u32>& defaultVertexBlockSize = core::dimension2d<u32>(64, 64)) OVERRIDE;

		//! Add a arrow mesh to the mesh pool
		virtual IAnimatedMesh* addArrowMesh(const io::path& name,
				video::SColor vtxColor0, video::SColor vtxColor1,
				u32 tesselationCylinder, u32 tesselationCone,
				f32 height, f32 cylinderHeight, f32 width0,
				f32 width1) OVERRIDE;

		//! Adds a static sphere mesh to the mesh pool.
		IAnimatedMesh* addSphereMesh(const io::path& name,
			f32 radius = 5.f, u32 polyCountX = 16, u32 polyCountY = 16) OVERRIDE;

		//! Adds a static volume light mesh to the mesh pool.
		IAnimatedMesh* addVolumeLightMesh(const io::path& name,
			const u32 SubdivideU = 32, const u32 SubdivideV = 32,
			const video::SColor FootColor = video::SColor(51, 0, 230, 180),
			const video::SColor TailColor = video::SColor(0, 0, 0, 0)) OVERRIDE;

		//! Adds a particle system scene node.
		virtual IParticleSystemSceneNode* addParticleSystemSceneNode(
			bool withDefaultEmitter=true, ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0,0,0),
			const core::vector3df& rotation = core::vector3df(0,0,0),
			const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f)) OVERRIDE;

		//! Adds a terrain scene node to the scene graph.
		virtual ITerrainSceneNode* addTerrainSceneNode(
			const io::path& heightMapFileName,
			ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0.0f,0.0f,0.0f),
			const core::vector3df& rotation = core::vector3df(0.0f,0.0f,0.0f),
			const core::vector3df& scale = core::vector3df(1.0f,1.0f,1.0f),
			video::SColor vertexColor = video::SColor(255,255,255,255),
			s32 maxLOD=4, E_TERRAIN_PATCH_SIZE patchSize=ETPS_17,s32 smoothFactor=0,
			bool addAlsoIfHeightmapEmpty = false) OVERRIDE;

		//! Adds a terrain scene node to the scene graph.
		virtual ITerrainSceneNode* addTerrainSceneNode(
			io::IReadFile* heightMap,
			ISceneNode* parent=0, s32 id=-1,
			const core::vector3df& position = core::vector3df(0.0f,0.0f,0.0f),
			const core::vector3df& rotation = core::vector3df(0.0f,0.0f,0.0f),
			const core::vector3df& scale = core::vector3df(1.0f,1.0f,1.0f),
			video::SColor vertexColor = video::SColor(255,255,255,255),
			s32 maxLOD=4, E_TERRAIN_PATCH_SIZE patchSize=ETPS_17,s32 smoothFactor=0,
			bool addAlsoIfHeightmapEmpty = false) OVERRIDE;

		//! Adds a dummy transformation scene node to the scene graph.
		virtual IDummyTransformationSceneNode* addDummyTransformationSceneNode(
			ISceneNode* parent = 0, s32 id = -1) OVERRIDE;

		//! Adds an empty scene node.
		virtual ISceneNode* addEmptySceneNode(ISceneNode* parent, s32 id = -1) OVERRIDE;

		//! Returns the root scene node. This is the scene node wich is parent
		//! of all scene nodes. The root scene node is a special scene node which
		//! only exists to manage all scene nodes. It is not rendered and cannot
		//! be removed from the scene.
		//! \return Pointer to the root scene node.
		virtual ISceneNode* getRootSceneNode() OVERRIDE;

		//! Returns the current active camera.
		//! \return The active camera is returned. Note that this can be NULL, if there
		//! was no camera created yet.
		virtual ICameraSceneNode* getActiveCamera() const OVERRIDE;

		//! Sets the active camera. The previous active camera will be deactivated.
		//! \param camera: The new camera which should be active.
		virtual void setActiveCamera(ICameraSceneNode* camera) OVERRIDE;

		//! Adds an external mesh loader.
		virtual void addExternalMeshLoader(IMeshLoader* externalLoader) OVERRIDE;

		//! Returns the number of mesh loaders supported by Irrlicht at this time
		virtual u32 getMeshLoaderCount() const OVERRIDE;

		//! Retrieve the given mesh loader
		virtual IMeshLoader* getMeshLoader(u32 index) const OVERRIDE;

		//! Returns the number of scene loaders supported by Irrlicht at this time
		virtual u32 getSceneLoaderCount() const OVERRIDE;

		//! Retrieve the given scene loader
		virtual ISceneLoader* getSceneLoader(u32 index) const OVERRIDE;

		//! Returns a pointer to the mesh manipulator.
		virtual IMeshManipulator* getMeshManipulator() OVERRIDE;

		//! Sets the color of stencil buffers shadows drawn by the scene manager.
		virtual void setShadowColor(video::SColor color) OVERRIDE;

		//! Returns the current color of shadows.
		virtual video::SColor getShadowColor() const OVERRIDE;

		//! Adds a scene node to the deletion queue.
		virtual void addToDeletionQueue(ISceneNode* node) OVERRIDE;

		//! Returns the first scene node with the specified id.
		virtual ISceneNode* getSceneNodeFromId(s32 id, ISceneNode* start = 0) OVERRIDE;

		//! Returns the first scene node with the specified name.
		virtual ISceneNode* getSceneNodeFromName(const c8* name, ISceneNode* start = 0) OVERRIDE;

		//! Returns the first scene node with the specified type.
		virtual ISceneNode* getSceneNodeFromType(scene::ESCENE_NODE_TYPE type, ISceneNode* start = 0) OVERRIDE;

		//! returns scene nodes by type.
		virtual void getSceneNodesFromType(ESCENE_NODE_TYPE type, core::array<scene::ISceneNode*>& outNodes, ISceneNode* start = 0) OVERRIDE;

		//! Posts an input event to the environment. Usually you do not have to
		//! use this method, it is used by the internal engine.
		virtual bool postEventFromUser(const SEvent& event) OVERRIDE;

		//! Clears the whole scene. All scene nodes are removed.
		virtual void clear() OVERRIDE;

		//! Removes all children of this scene node
		virtual void removeAll() OVERRIDE;

		//! Returns interface to the parameters set in this scene.
		virtual io::IAttributes* getParameters() OVERRIDE;

		//! Returns current render pass.
		virtual E_SCENE_NODE_RENDER_PASS getSceneNodeRenderPass() const OVERRIDE;

		//! Creates a new scene manager.
		virtual ISceneManager* createNewSceneManager(bool cloneContent) OVERRIDE;

		//! Returns type of the scene node
		virtual ESCENE_NODE_TYPE getType() const OVERRIDE { return ESNT_SCENE_MANAGER; }

		//! Returns the default scene node factory which can create all built in scene nodes
		virtual ISceneNodeFactory* getDefaultSceneNodeFactory() OVERRIDE;

		//! Adds a scene node factory to the scene manager.
		/** Use this to extend the scene manager with new scene node types which it should be
		able to create automaticly, for example when loading data from xml files. */
		virtual void registerSceneNodeFactory(ISceneNodeFactory* factoryToAdd) OVERRIDE;

		//! Returns amount of registered scene node factories.
		virtual u32 getRegisteredSceneNodeFactoryCount() const OVERRIDE;

		//! Returns a scene node factory by index
		virtual ISceneNodeFactory* getSceneNodeFactory(u32 index) OVERRIDE;

		//! Returns a typename from a scene node type or null if not found
		virtual const c8* getSceneNodeTypeName(ESCENE_NODE_TYPE type) OVERRIDE;

		//! Adds a scene node to the scene by name
		virtual ISceneNode* addSceneNode(const char* sceneNodeTypeName, ISceneNode* parent = 0) OVERRIDE;

		//! Returns the default scene node animator factory which can create all built-in scene node animators
		virtual ISceneNodeAnimatorFactory* getDefaultSceneNodeAnimatorFactory() OVERRIDE;

		//! Returns amount of registered scene node animator factories.
		virtual u32 getRegisteredSceneNodeAnimatorFactoryCount() const OVERRIDE;

		//! Returns a scene node animator factory by index
		virtual ISceneNodeAnimatorFactory* getSceneNodeAnimatorFactory(u32 index) OVERRIDE;

		//! Returns a mesh writer implementation if available
		virtual IMeshWriter* createMeshWriter(EMESH_WRITER_TYPE type) OVERRIDE;

		//! Get a skinned mesh, which is not available as header-only code
		virtual ISkinnedMesh* createSkinnedMesh() OVERRIDE;

		//! Sets ambient color of the scene
		virtual void setAmbientLight(const video::SColorf &ambientColor) OVERRIDE;

		//! Returns ambient color of the scene
		virtual const video::SColorf& getAmbientLight() const OVERRIDE;

		//! Register a custom callbacks manager which gets callbacks during scene rendering.
		virtual void setLightManager(ILightManager* lightManager) OVERRIDE;

		//! Get current render time.
		virtual E_SCENE_NODE_RENDER_PASS getCurrentRendertime() const { return CurrentRendertime; }

		//! Set current render time.
		virtual void setCurrentRendertime(E_SCENE_NODE_RENDER_PASS currentRendertime) OVERRIDE { CurrentRendertime = currentRendertime; }

		//! Get an instance of a geometry creator.
		virtual const IGeometryCreator* getGeometryCreator(void) const OVERRIDE { return GeometryCreator; }

		//! returns if node is culled
		virtual bool isCulled(const ISceneNode* node) const OVERRIDE;

	private:

		//! clears the deletion list
		void clearDeletionList();

		struct DefaultNodeEntry
		{
			DefaultNodeEntry(ISceneNode* n) :
				Node(n), TextureValue(0), MaterialType(video::EMT_SOLID)
			{
				if (n->getMaterialCount())
				{
					TextureValue = (n->getMaterial(0).getTexture(0));
					MaterialType = n->getMaterial(0).MaterialType;
				}
			}

			bool operator < (const DefaultNodeEntry& other) const
			{
				if (MaterialType == other.MaterialType)
					return (TextureValue < other.TextureValue);
				else
					return (MaterialType < other.MaterialType);
			}

			ISceneNode* Node;
			private:
			void* TextureValue;
			video::E_MATERIAL_TYPE MaterialType;
		};

		//! sort on distance (center) to camera
		struct TransparentNodeEntry
		{
			TransparentNodeEntry(ISceneNode* n, const core::vector3df& camera)
				: Node(n)
			{
				Distance = Node->getAbsoluteTransformation().getTranslation().getDistanceFromSQ(camera);
			}

			bool operator < (const TransparentNodeEntry& other) const
			{
				return Distance > other.Distance;
			}

			ISceneNode* Node;
			private:
				f64 Distance;
		};

		//! sort on distance (sphere) to camera
		struct DistanceNodeEntry
		{
			DistanceNodeEntry(ISceneNode* n, const core::vector3df& cameraPos)
				: Node(n)
			{
				setNodeAndDistanceFromPosition(n, cameraPos);
			}

			bool operator < (const DistanceNodeEntry& other) const
			{
				return Distance < other.Distance;
			}

			void setNodeAndDistanceFromPosition(ISceneNode* n, const core::vector3df & fromPosition)
			{
				Node = n;
				Distance = Node->getAbsoluteTransformation().getTranslation().getDistanceFromSQ(fromPosition);
				Distance -= Node->getBoundingBox().getExtent().getLengthSQ() * 0.5;
			}

			ISceneNode* Node;
			private:
			f64 Distance;
		};

		//! video driver
		video::IVideoDriver* Driver;

		//! file system
		io::IFileSystem* FileSystem;

		//! GUI Enviroment ( Debug Purpose )
		gui::IGUIEnvironment* GUIEnvironment;

		//! cursor control
		gui::ICursorControl* CursorControl;

		//! render pass lists
		core::array<ISceneNode*> CameraList;
		core::array<ISceneNode*> LightList;
		core::array<ISceneNode*> ShadowNodeList;
		core::array<ISceneNode*> SkyBoxList;
		core::array<DefaultNodeEntry> SolidNodeList;
		core::array<TransparentNodeEntry> TransparentNodeList;
		core::array<TransparentNodeEntry> TransparentEffectNodeList;

		core::array<IMeshLoader*> MeshLoaderList;
		core::array<ISceneLoader*> SceneLoaderList;
		core::array<ISceneNode*> DeletionList;
		core::array<ISceneNodeFactory*> SceneNodeFactoryList;
		core::array<ISceneNodeAnimatorFactory*> SceneNodeAnimatorFactoryList;

		//! current active camera
		ICameraSceneNode* ActiveCamera;
		core::vector3df camWorldPos; // Position of camera for transparent nodes.

		video::SColor ShadowColor;
		video::SColorf AmbientLight;

		//! String parameters
		io::CAttributes Parameters;

		//! Mesh cache
		IMeshCache* MeshCache;

		E_SCENE_NODE_RENDER_PASS CurrentRendertime;

		//! An optional callbacks manager to allow the user app finer control
		//! over the scene lighting and rendering.
		ILightManager* LightManager;

		//! constants for reading and writing XML.
		//! Not made static due to portability problems.
		const core::stringw IRR_XML_FORMAT_SCENE;
		const core::stringw IRR_XML_FORMAT_NODE;
		const core::stringw IRR_XML_FORMAT_NODE_ATTR_TYPE;

		IGeometryCreator* GeometryCreator;
	};

} // end namespace video
} // end namespace scene

#endif

