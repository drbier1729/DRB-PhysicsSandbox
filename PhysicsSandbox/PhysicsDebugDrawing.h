#ifndef DRB_PHYSICSDEBUGDRAWING_H
#define DRB_PHYSICSDEBUGDRAWING_H

#include "Shader.h"
#include "Mesh.h"
#include "Camera.h"

namespace drb {

	class Mesh;

	// This should exist elsewhere
	struct ColorInfo
	{
		glm::vec3 specular, diffuse;
		float gloss, opacity;
	};

	namespace physics {

		struct Convex;
		class World;
		class RigidBody;
		struct ContactManifold;
		struct Plane;

		class DebugRenderer
		{
		private:
			// Used to store meshes that are not just the standard Cube, Sphere, and Cylinder.
			std::unordered_map<Convex const*, drb::Mesh> meshes;
			
			ShaderProgram shader;
			glm::vec3 lightColor;
			glm::vec3 lightDir;

			World* world;

		public:
			// Assumed to be called after all physics data is finalized. Relies on
			// things not moving in memory...
			void Init(World& world);

			// Calls BeginDraw, draws all rigidibodies in world with velocity vectors, 
			// draws contact manifolds, then calls EndDraw
			void Draw(Camera const& cam, float frameInterpolation) const;

			// Below are methods to allow more granular control over what is drawn. 
			// Call BeginDraw before calling these methods and FinishDraw after
			// them in order to use/unuse the debug shader.
			DebugRenderer const& BeginDraw(glm::mat4 const& proj, glm::mat4 const& view, glm::vec3 const& eyePos) const;
			DebugRenderer const& EndDraw() const;
			DebugRenderer const& DrawOneRigidBody(RigidBody const& rb, float frameInterpolation, ColorInfo const& ci) const;
			DebugRenderer const& HighlightOneRigidBody(RigidBody const& rb, float frameInterpolation, ColorInfo const& ci) const;
			DebugRenderer const& DrawRigidBodies(float frameInterpolation) const;
			DebugRenderer const& DrawStaticCollisionGeometry() const;
			DebugRenderer const& DrawOneContactManifold(ContactManifold const& m) const;
			DebugRenderer const& DrawContactManifolds() const;
			DebugRenderer const& DrawBVH() const;
			DebugRenderer const& EnableWireframeMode(Bool val = true) const;
		};

		// Some additional functions which might be of use...

		// Draws a vector (world space) from a given start point in world space
		void DrawVector(glm::vec3 const& vec, glm::vec3 const& startPt, ShaderProgram const& prg, ColorInfo const& colorInfo);
		
		// Draws a given mesh with additional info (transform and color)
		void DrawMesh(drb::Mesh const& m, ShaderProgram const& prg, glm::mat4 const& tr, ColorInfo const& colorInfo);

		// Draws the clipping planes generated from the edges of face
		void DrawFaceClipPlanes(Convex const& cvx, Uint8 faceIdx, ShaderProgram const& prg, glm::mat4 const& tr, ColorInfo const& colorInfo);
		
		// Creates a render mesh out of a physics::Convex object
		drb::Mesh MakeRenderMesh(Convex const& cvx);

		// Enables/disables wireframe mode by setting glPolygonMode
		void EnableWireframeMode(Bool val = true);
	}
}

#include "PhysicsDebugDrawing.inl"
#endif
