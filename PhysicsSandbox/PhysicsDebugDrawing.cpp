#include "pch.h"
#include "PhysicsDebugDrawing.h"

#include "PhysicsGeometry.h"
#include "PhysicsWorld.h"
#include "Mesh.h"
#include "Window.h"

namespace drb {
	namespace physics {

		// Helper fwd decls
		static inline Mat4 SphereTransform(Sphere const& sph, Mat4 const& tr);
		static inline Mat4 CubeTransform(Vec3 const& halfwidths, Mat4 const& tr);
		static void DrawCapsule(Capsule const& cap, ShaderProgram const& prg, Mat4 const& tr, ColorInfo const& colorInfo);
		static void DrawAABB(AABB const& b, ShaderProgram const& prg, ColorInfo const& colorInfo);

		// ---------------------------------------------------------------------
		// DEBUG RENDERER METHODS
		// ---------------------------------------------------------------------
		void DebugRenderer::Init(World& world_)
		{
			// Set shader and lighting data
			shader = ShaderProgram(
				{ 
					Shader("default.vert", GL_VERTEX_SHADER), 
					Shader("default.frag", GL_FRAGMENT_SHADER)
				});

			lightColor = { 3.0f, 3.0f, 3.0f };
			lightDir = { glm::normalize(Vec3(0.5f, 1.0f, 2.0f)) };

			// Build render meshes from world
			world = &world_;
			for (auto&& rb : world->bodies) 
			{
				for (auto&& cvx : rb.hulls) {
					meshes.emplace(&cvx.shape, std::move(MakeRenderMesh(cvx.shape)));
				}
			}
			for (auto&& cvx : world->cvxColliders)
			{
				meshes.emplace(&cvx.shape, std::move(MakeRenderMesh(cvx.shape)));
			}
		}

		void DebugRenderer::Draw(Camera const& cam, Float32 frameInterpolation) const
		{
			
			BeginDraw(cam.ProjMatrix(), cam.ViewMatrix(), cam.GetPosition());

			DrawRigidBodies(frameInterpolation);
			DrawStaticCollisionGeometry();
			// Draw contact manifolds

			EndDraw();
		}

		DebugRenderer const& DebugRenderer::BeginDraw(Mat4 const& proj, Mat4 const& view, Vec3 const& eyePos) const
		{
			shader.Use();

			// Sets View and Proj matrices
			shader.SetUniformMatrix(glm::value_ptr(proj), 4, "uProj");
			shader.SetUniformMatrix(glm::value_ptr(view), 4, "uView");
			shader.SetUniformVector(glm::value_ptr(eyePos), 3, "uViewPos");

			// Set lighting values
			shader.SetUniformVector(glm::value_ptr(lightColor), 3, "uLightColor");
			shader.SetUniformVector(glm::value_ptr(lightDir), 3, "uLightDir");

			return *this;
		}

		DebugRenderer const& DebugRenderer::EndDraw() const
		{
			shader.Unuse();
			return *this;
		}

		DebugRenderer const& DebugRenderer::DrawRigidBodies(Float32 frameInterpolation) const
		{
			static constexpr ColorInfo dynamicColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.5f, 0.0f, 0.0f },
					.gloss = 0.82f,
					.opacity = 0.7f
			};

			static constexpr ColorInfo kinematicColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.0f, 0.5f },
					.gloss = 0.82f,
					.opacity = 0.7f
			};
			
			static constexpr ColorInfo vectorColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.9f, 0.9f, 0.9f },
					.gloss = 0.82f,
					.opacity = 1.0f
			};

			for (auto&& rb : world->bodies)
			{
				ColorInfo const& ci = rb.GetType() == RigidBody::Type::Dynamic ? dynamicColor : kinematicColor;

				DrawOneRigidBody(rb, frameInterpolation, ci);
				DrawVector(rb.linearVelocity, rb.position, shader, vectorColor);
			}
			
			return *this;
		}
		
		DebugRenderer const& DebugRenderer::DrawOneRigidBody(RigidBody& rb, Float32 frameInterpolation, ColorInfo const& ci) const
		{
			Vec3 const interpolatedPos = frameInterpolation * rb.position + (1.0f - frameInterpolation) * rb.prevPosition;
			Quat const interpolatedRot = frameInterpolation * rb.orientation + (1.0f - frameInterpolation) * rb.prevOrientation;
			Mat4 const rbTr = glm::translate(Mat4(1), interpolatedPos) * glm::toMat4(interpolatedRot);

			Mat4 modelTr{};
			AABB b{};


			for (auto&& s : rb.spheres) {
				modelTr = rbTr * s.transform;
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };
				modelTr = SphereTransform(s.shape, modelTr);
				DrawMesh(use.mesh, shader, modelTr, ci);
				
				EnableWireframeMode();
				b = MakeAABB(s.shape, modelTr);
				DrawAABB(b, shader, ci);
				EnableWireframeMode(false);
			}

			for (auto&& c : rb.capsules) {
				modelTr = rbTr * c.transform;
				DrawCapsule(c.shape, shader, modelTr, ci);

				EnableWireframeMode();
				b = MakeAABB(c.shape, modelTr);
				DrawAABB(b, shader, ci);
				EnableWireframeMode(false);
			}

			for (auto&& h : rb.hulls) {
				modelTr = rbTr * h.transform;
				drb::Mesh::ScopedUse use{ meshes.at(&h.shape) };
				DrawMesh(use.mesh, shader, modelTr, ci);

				EnableWireframeMode();
				b = MakeAABB(h.shape, modelTr);
				DrawAABB(b, shader, ci);
				EnableWireframeMode(false);
			}

			return *this;
		}


		DebugRenderer const& DebugRenderer::HighlightOneRigidBody(RigidBody& rb, Float32 frameInterpolation, ColorInfo const& ci) const
		{
			Vec3 const interpolatedPos = frameInterpolation * rb.position + (1.0f - frameInterpolation) * rb.prevPosition;
			Quat const interpolatedRot = frameInterpolation * rb.orientation + (1.0f - frameInterpolation) * rb.prevOrientation;
			Mat4 const rbTr = glm::translate(Mat4(1), interpolatedPos) * glm::toMat4(interpolatedRot);

			Mat4 modelTr{};
			Mat4 const scale = glm::scale(Mat4(1), Vec3(1.03f));

			for (auto&& s : rb.spheres) {
				modelTr = rbTr * s.transform * scale;
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };
				modelTr = SphereTransform(s.shape, modelTr);
				DrawMesh(use.mesh, shader, modelTr, ci);
			}

			for (auto&& c : rb.capsules) {
				modelTr = rbTr * c.transform * scale;
				DrawCapsule(c.shape, shader, modelTr, ci);
			}

			for (auto&& h : rb.hulls) {
				modelTr = rbTr * h.transform * scale;
				drb::Mesh::ScopedUse use{ meshes.at(&h.shape) };
				DrawMesh(use.mesh, shader, modelTr, ci);
			}

			return *this;
		}

		DebugRenderer const& DebugRenderer::DrawStaticCollisionGeometry() const
		{
			static constexpr ColorInfo staticColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.5f, 0.0f },
					.gloss = 0.82f,
					.opacity = 0.7f
			};

			for (auto&& s : world->sphColliders) {
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };
				Mat4 const modelTr = SphereTransform(s.shape, s.transform);
				DrawMesh(use.mesh, shader, modelTr, staticColor);
			}
			
			for (auto&& c : world->capColliders) {
				DrawCapsule(c.shape, shader, c.transform, staticColor);
			}
			
			for (auto&& h : world->cvxColliders) {
				drb::Mesh::ScopedUse use{ meshes.at(&h.shape) };
				DrawMesh(use.mesh, shader, h.transform, staticColor);
			}

			return *this;
		}


		DebugRenderer const& DebugRenderer::EnableWireframeMode(Bool val) const
		{
			drb::physics::EnableWireframeMode(val);
			return *this;
		}


		// ---------------------------------------------------------------------
		// HELPERS
		// ---------------------------------------------------------------------

		void EnableWireframeMode(Bool val)
		{
			if (val) {
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			}
			else {
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			}
		}

		void DrawMesh(drb::Mesh const& m, ShaderProgram const& shader, Mat4 const& tr, ColorInfo const& colorInfo)
		{
			// assumes mesh is already in use
			Mat4 const modelInvTranspose = glm::transpose(glm::inverse(tr));

			shader.SetUniformMatrix(glm::value_ptr(tr), 4, "uModel");
			shader.SetUniformMatrix(glm::value_ptr(modelInvTranspose), 4, "uModelInvTr");

			shader.SetUniformVector(glm::value_ptr(colorInfo.specular), 3, "uSpecular");
			shader.SetUniformVector(glm::value_ptr(colorInfo.diffuse), 3, "uDiffuse");
			shader.SetUniform(colorInfo.gloss, "uGlossiness");
			shader.SetUniform(colorInfo.opacity, "uAlpha");

			m.Draw();
		}

		void DrawVector(Vec3 const& vec, Vec3 const& startPt, ShaderProgram const& prg, ColorInfo const& colorInfo)
		{
			Mat4 modelTr{};

			Float32 const mag = glm::length(vec);
			if (EpsilonEqual(mag, 0.0f)) {
				return;
			}
			Vec3 const dir = vec * (1.0f / mag);
			Mat4 const rot = glm::toMat4(glm::rotation(Vec3(0, 0, 1), dir));

			// Draw cylinder
			{
				modelTr = glm::translate(Mat4(1), startPt) *		   // translate s.t. bottom of cylinder is at startPt
					rot *											   // rotate to point in direction of vec
					glm::translate(Mat4(1), Vec3(0, 0, 0.5f * mag)) *  // shift to align "bottom" of cylinder with origin
					glm::scale(Mat4(1), Vec3(0.05f, 0.05f, 0.5f * mag)); // scale to make skinny and as long as magnitude of vec

				drb::Mesh::ScopedUse use{ drb::Mesh::Cylinder() };
				DrawMesh(use.mesh, prg, modelTr, colorInfo);
			}

			// Draw cone
			{
				modelTr = glm::translate(Mat4(1), startPt + vec) * rot * glm::scale(Mat4(1), Vec3(0.2f));
				drb::Mesh::ScopedUse use{ drb::Mesh::Cone()};
				DrawMesh(use.mesh, prg, modelTr, colorInfo);
			}
		}

		void DrawAABB(AABB const& b, ShaderProgram const& prg, ColorInfo const& colorInfo)
		{
			Mat4 const modelTr = glm::translate(Mat4(1), b.Center()) * glm::scale(Mat4(1), b.Halfwidths());

			drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
			DrawMesh(use.mesh, prg, modelTr, colorInfo);
		}

		drb::Mesh MakeRenderMesh(Convex const& cvx)
		{
			if (cvx.verts.size() == 0) {
				ASSERT(false, "Convex hull cannot be empty");
				return drb::Mesh{};
			}

			// Construct vertex buffer interleaved with vertex normals and uvs
			std::vector<float> verts;
			verts.reserve(cvx.verts.size() * 8u);

			std::vector<glm::uvec3> indices;
			indices.reserve(cvx.faces.size());

			// Triangulate faces via "fan triangulation" and compute vertex normals
			for (auto&& face : cvx.faces) {

				Uint32 vertCount = 0u;
				Uint32 faceIdx = static_cast<Uint32>(verts.size()) / 8u;

				auto const e0 = cvx.edges[face.edge];

				auto PushVert = [&verts, &face, &cvx, &vertCount](unsigned vertIdx) {
					// Insert world pos
					Vec3 const v = cvx.verts[vertIdx];
					verts.push_back(v.x);
					verts.push_back(v.y);
					verts.push_back(v.z);

					// Insert vertex normal
					verts.push_back(face.plane.n.x);
					verts.push_back(face.plane.n.y);
					verts.push_back(face.plane.n.z);

					// Insert uv
					verts.push_back(0.0f);
					verts.push_back(0.0f);

					++vertCount;
				};

				// Traverse the face while enumerating and pushing vertices
				auto eCurr = e0;
				do {

					PushVert(eCurr.origin);
					eCurr = cvx.edges[eCurr.next];

				} while (eCurr != e0);

				// Triangulate the face
				for (Uint32 i = 1; i < vertCount - 1; ++i)
				{
					indices.emplace_back(faceIdx, faceIdx + i, faceIdx + i + 1);
				}
			}

			return drb::Mesh{
				verts.data(),                    verts.size(),
				glm::value_ptr(*indices.data()), indices.size() * 3u
			};
		}

		static inline Mat4 SphereTransform(Sphere const& sph, Mat4 const& tr)
		{
			return tr * glm::scale(Mat4(1), Vec3(sph.r));
		}

		static inline Mat4 CubeTransform(Vec3 const& halfwidths, Mat4 const& tr)
		{
			return tr * glm::scale(Mat4(1), halfwidths);
		}

		static void DrawCapsule(Capsule const& cap, ShaderProgram const& prg, Mat4 const& tr, ColorInfo const& colorInfo)
		{
			Mat4 modelTr{};

			// Draw cylinder
			{
				modelTr = tr * glm::scale(Mat4(1), Vec3(cap.r - 0.01f, cap.h, cap.r - 0.01f)) * glm::rotate(Mat4(1), 0.5f * 3.142f, Vec3(1, 0, 0)); // first rotation is to correct z-up model

				drb::Mesh::ScopedUse use{ drb::Mesh::Cylinder() };
				DrawMesh(use.mesh, prg, modelTr, colorInfo);
			}

			// Draw spheres
			{
				Segment segment{ GetCentralSegment(cap, tr) };
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };

				// Draw bottom sphere
				modelTr = glm::translate(Mat4(1), segment.b) * glm::scale(Mat4(1), Vec3(cap.r));
				DrawMesh(use.mesh, prg, modelTr, colorInfo);

				// Draw top sphere
				modelTr = glm::translate(Mat4(1), segment.e) * glm::scale(Mat4(1), Vec3(cap.r));
				DrawMesh(use.mesh, prg, modelTr, colorInfo);

			}
		}
	}
}

