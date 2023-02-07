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
		Vec3 specular, diffuse;
		Float32 gloss, opacity;
	};

	namespace physics {

		struct Convex;
		class World;
		class RigidBody;


		class DebugRenderer
		{
		private:
			// Used to store meshes that are not just the standard Cube, Sphere, and Cylinder.
			std::unordered_map<Convex const*, drb::Mesh> meshes;
			
			ShaderProgram shader;
			Vec3 lightColor;
			Vec3 lightDir;

			World* world;

		public:
			// Assumed to be called after all physics data is finalized. Relies on
			// things not moving in memory...
			void Init(World& world);

			// Calls BeginDraw, draws all rigidibodies in world with velocity vectors, 
			// draws contact manifolds, then calls EndDraw
			void Draw(Camera const& cam, Float32 frameInterpolation) const;

			// Below are methods to allow more granular control over what is drawn. 
			// Call BeginDraw before calling these methods and FinishDraw after
			// them in order to use/unuse the debug shader.
			DebugRenderer const& BeginDraw(Mat4 const& proj, Mat4 const& view, Vec3 const& eyePos) const;
			DebugRenderer const& EndDraw() const;
			DebugRenderer const& DrawOneRigidBody(RigidBody& rb, Float32 frameInterpolation, ColorInfo const& ci) const;
			DebugRenderer const& HighlightOneRigidBody(RigidBody& rb, Float32 frameInterpolation, ColorInfo const& ci) const;
			DebugRenderer const& DrawRigidBodies(Float32 frameInterpolation) const;
			DebugRenderer const& DrawStaticCollisionGeometry() const;
			DebugRenderer const& EnableWireframeMode(Bool val = true) const;
		};

		// Some additional functions which might be of use...

		// Draws a vector (world space) from a given start point in world space
		void DrawVector(Vec3 const& vec, Vec3 const& startPt, ShaderProgram const& prg, ColorInfo const& colorInfo);
		
		// Draws a given mesh with additional info (transform and color)
		void DrawMesh(drb::Mesh const& m, ShaderProgram const& prg, Mat4 const& tr, ColorInfo const& colorInfo);
		
		// Creates a render mesh out of a physics::Convex object
		drb::Mesh MakeRenderMesh(Convex const& cvx);

		// Enables/disables wireframe mode by setting glPolygonMode
		void EnableWireframeMode(Bool val = true);
	}
}

#endif
