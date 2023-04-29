#include "pch.h"
#include "PhysicsDebugDrawing.h"

#include "CollisionGeometry.h"
#include "RigidBody.h"
#include "PhysicsWorld.h"
#include "Mesh.h"
#include "Window.h"
#include "DRBAssert.h"

namespace drb {
	namespace physics {

		// Helper fwd decls
		static inline glm::mat4 SphereTransform(Sphere const& sph, glm::mat4 const& tr);
		static inline glm::mat4 CubeTransform(Vec3 const& halfwidths, glm::mat4 const& tr);
		static void DrawCapsule(Capsule const& cap, ShaderProgram const& prg, glm::mat4 const& tr, ColorInfo const& colorInfo);
		static void DrawAABB(AABB const& b, ShaderProgram const& prg, ColorInfo const& colorInfo);
		static void DrawOBB(AABB const& localAABB, glm::mat4 const& rot, ShaderProgram const& prg, ColorInfo const& colorInfo);
		
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

			lightColor = { 3.0, 3.0, 3.0 };
			lightDir = { Normalize(Vec3(0.5, 1.0, 2.0)) };

			// Build render meshes from world
			world = &world_;
			for (auto&& rb : world->bodies) 
			{
				for (auto&& cvx : rb.geometry->hulls) {
					meshes.emplace(&cvx, std::move(MakeRenderMesh(cvx)));
				}
			}
			for (auto&& col : world->colliders) {
				for (auto&& cvx : col.hulls)
				{
					meshes.emplace(&cvx, std::move(MakeRenderMesh(cvx)));
				}
			}
		}

		void DebugRenderer::Draw(Camera const& cam, float frameInterpolation) const
		{
			
			BeginDraw(cam.ProjMatrix(), cam.ViewMatrix(), cam.GetPosition());

			DrawRigidBodies(frameInterpolation);
			DrawStaticCollisionGeometry();
			DrawContactManifolds();

			EndDraw();
		}

		DebugRenderer const& DebugRenderer::BeginDraw(glm::mat4 const& proj, glm::mat4 const& view, glm::vec3 const& eyePos) const
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

		DebugRenderer const& DebugRenderer::DrawRigidBodies(float frameInterpolation) const
		{
			static constexpr ColorInfo dynamicColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.5f, 0.0f, 0.0f },
					.gloss = 0.82f,
					.opacity = 1.0f
			};

			static constexpr ColorInfo kinematicColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.0f, 0.5f },
					.gloss = 0.82f,
					.opacity = 0.3f
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
		
		DebugRenderer const& DebugRenderer::DrawOneRigidBody(RigidBody const& rb, float frameInterpolation, ColorInfo const& ci) const
		{
			if (not rb.geometry) { return *this; }

			glm::vec3 const interpolatedPos = frameInterpolation * glm::vec3(rb.position) + (1.0f - frameInterpolation) * glm::vec3(rb.prevPosition);
			glm::quat const interpolatedRot = glm::slerp(glm::quat(rb.prevOrientation), glm::quat(rb.orientation), frameInterpolation);
			glm::mat4 const translation = glm::translate(glm::mat4(1), interpolatedPos);
			glm::mat4 const rotation = glm::toMat4(interpolatedRot);
			glm::mat4 const rbTr = translation * rotation;

			glm::mat4 modelTr{};
			AABB b{};

			{
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };
				for (auto&& s : rb.geometry->spheres) {
					modelTr = rbTr;
					{
						modelTr = SphereTransform(s, modelTr);
						DrawMesh(use.mesh, shader, modelTr, ci);
					}
				}
			}

			for (auto&& c : rb.geometry->capsules) {
				modelTr = rbTr;
				DrawCapsule(c, shader, modelTr, ci);

				//EnableWireframeMode();
				//b = c.Bounds(translation);
				//DrawOBB(b, interpolatedRot, shader, ci);
				//EnableWireframeMode(false);
			}
			
			for (auto&& s : rb.geometry->boxes) {
				modelTr = rbTr * glm::mat4(s.Transform()) * glm::scale(glm::mat4(1), glm::vec3(s.extents));
				{
					drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
					DrawMesh(use.mesh, shader, modelTr, ci);
				}
			}

			for (auto&& h : rb.geometry->hulls) {
				modelTr = rbTr * glm::mat4(h.Transform());
				drb::Mesh::ScopedUse use{ meshes.at(&h) };
				DrawMesh(use.mesh, shader, modelTr, ci);


				//EnableWireframeMode();
				//b = h.Bounds(rbTr);
				//DrawAABB(b, shader, ci);
				//EnableWireframeMode(false);
			}

			return *this;
		}

		DebugRenderer const& DebugRenderer::DrawBVH() const
		{
			static constexpr ColorInfo aabbColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.9f, 0.9f, 0.9f },
					.gloss = 0.82f,
					.opacity = 1.0f
			};



			EnableWireframeMode(true);
			{
				drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
				auto drawBV = [this, &use](BV const& bv) {

					Mat4 const modelTr = glm::translate(Mat4(1), bv.fatBounds.Center()) * glm::scale(Mat4(1), bv.fatBounds.Halfwidths());
					DrawMesh(use.mesh, shader, modelTr, aabbColor);
				};
				world->bvhTree.ForEach(drawBV);
			}
			EnableWireframeMode(false);

			return *this;
		}


		DebugRenderer const& DebugRenderer::HighlightOneRigidBody(RigidBody const& rb, float frameInterpolation, ColorInfo const& ci) const
		{
			if (not rb.geometry) { return *this; }

			glm::vec3 const interpolatedPos = frameInterpolation * glm::vec3(rb.position) + (1.0f - frameInterpolation) * glm::vec3(rb.prevPosition);
			glm::quat const interpolatedRot = glm::slerp(glm::quat(rb.prevOrientation), glm::quat(rb.orientation), frameInterpolation);
			glm::mat4 const rbTr = glm::translate(glm::mat4(1), interpolatedPos) * glm::toMat4(interpolatedRot);

			glm::mat4 modelTr{};
			glm::mat4 const scale = glm::scale(glm::mat4(1), glm::vec3(1.03f));
			
			for (auto&& s : rb.geometry->spheres) {
				modelTr = rbTr * scale;
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };
				modelTr = SphereTransform(s, modelTr);
				DrawMesh(use.mesh, shader, modelTr, ci);
			}

			for (auto&& c : rb.geometry->capsules) {
				modelTr = rbTr * scale;
				DrawCapsule(c, shader, modelTr, ci);
			}

			for (auto&& s : rb.geometry->boxes) {
				modelTr = rbTr * glm::mat4(s.Transform()) * glm::scale(glm::mat4(1), glm::vec3(s.extents)) * scale;
				drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
				DrawMesh(use.mesh, shader, modelTr, ci);
			}

			for (auto&& h : rb.geometry->hulls) {
				modelTr = rbTr * glm::mat4(h.Transform()) * scale;
				drb::Mesh::ScopedUse use{ meshes.at(&h) };
				DrawMesh(use.mesh, shader, modelTr, ci);
			}

			return *this;
		}

		DebugRenderer const& DebugRenderer::DrawContactManifolds() const
		{
			static constexpr ColorInfo planeColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.2f, 0.5f },
					.gloss = 0.82f,
					.opacity = 0.3f
			};

			for (auto&& [key, m] : world->contacts) {
				DrawOneContactManifold(m);

				// TODO : draw clip planes for the reference face and clipped verts of 
				// incident face

				/*if (key.aShape->type == Convex::type) {
					auto const* cvx = static_cast<CollisionShape<Convex> const*>(key.aShape);
					
					

						if (m.featureA.type == Feature::Type::Face) {
							Uint8 const f = m.featureA.index;

							Mat4 const tr = key.a->GetTransformMatrix() * cvx->transform;
							DrawFaceClipPlanes(cvx->shape, f, shader, tr, planeColor);
						}
					
					
				}
				
				if (key.bShape->type == Convex::type) {
					auto const* cvx = static_cast<CollisionShape<Convex> const*>(key.bShape);
					
						if (m.featureB.type == Feature::Type::Face) {
							Uint8 const f = m.featureB.index;

							Mat4 const tr = key.b->GetTransformMatrix() * cvx->transform;
							DrawFaceClipPlanes(cvx->shape, f, shader, tr, planeColor);
						}
					
					
				}*/
			}
			return *this;
		}

		DebugRenderer const& DebugRenderer::DrawOneContactManifold(ContactManifold const& m) const
		{
			static constexpr ColorInfo contactColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.9f, 0.9f, 0.0f },
					.gloss = 0.82f,
					.opacity = 0.7f
			};
			
			static constexpr ColorInfo planeColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.2f, 0.5f },
					.gloss = 0.82f,
					.opacity = 0.3f
			};

			if (m.numContacts == 0) { return *this; }

			// Draw contact points, and compute center  while we're at it
			Vec3 manifoldCenterA = Vec3(0);
			Vec3 manifoldCenterB = Vec3(0);

			Vec3 const xA = m.rbA->GetPosition();
			Vec3 const xB = m.rbB->GetPosition();
			Quat const qA = m.rbA->GetOrientation();
			Quat const qB = m.rbB->GetOrientation();

			{
				drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };

				for (Uint8 i = 0u; i < m.numContacts; ++i)
				{
					auto const [wA, wB] = m.contacts[i].WorldPoints(m.rbA, m.rbB);
					manifoldCenterA += wA;

					Mat4 const modelTrA = glm::translate(Mat4(1), wA) *
										 glm::scale(Mat4(1), Vec3(0.05_r));
					Mat4 const modelTrB = glm::translate(Mat4(1), wB) *
										 glm::scale(Mat4(1), Vec3(0.05_r));
					DrawMesh(use.mesh, shader, modelTrA, contactColor);
					DrawMesh(use.mesh, shader, modelTrB, contactColor);
				}
				manifoldCenterA *= 1.0 / m.numContacts;
			}

			// Draw normal of separating plane
			{
				DrawVector(glm::vec3(m.normal), glm::vec3(manifoldCenterA), shader, contactColor);
			}

			// Draw separating plane
			{
				drb::Mesh::ScopedUse use{ drb::Mesh::Quad() };
				Mat4 const modelTr = glm::translate(Mat4(1), manifoldCenterA)
					               * UnitVectorToBasis4(m.normal)
								   * glm::scale(Mat4(1), Vec3(8));
								   
				DrawMesh(use.mesh, shader, glm::mat4(modelTr), planeColor);
			}

			return *this;
		}

		DebugRenderer const& DebugRenderer::DrawStaticCollisionGeometry() const
		{
			static constexpr ColorInfo staticColor{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.5f, 0.0f },
					.gloss = 0.82f,
					.opacity = 0.3f
			};

			Mat4 modelTr{};

			for (auto&& col : world->colliders) {

				for (auto&& s : col.spheres) {
					modelTr = SphereTransform(s, Mat4(1));
					drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };
					DrawMesh(use.mesh, shader, modelTr, staticColor);
				}

				for (auto&& s : col.boxes) {
					modelTr = s.Transform() * glm::scale(Mat4(1), s.extents);
					drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
					DrawMesh(use.mesh, shader, modelTr, staticColor);
				}

				for (auto&& c : col.capsules) {
					DrawCapsule(c, shader, Mat4(1), staticColor);
				}

				for (auto&& h : col.hulls) {
					drb::Mesh::ScopedUse use{ meshes.at(&h) };
					DrawMesh(use.mesh, shader, Mat4(1), staticColor);
				}
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

		void DrawMesh(drb::Mesh const& m, ShaderProgram const& shader, glm::mat4 const& tr_, ColorInfo const& colorInfo)
		{
			// assumes mesh is already in use
			glm::mat4 const tr = tr_;
			glm::mat4 const modelInvTranspose = glm::transpose(glm::inverse(tr));

			shader.SetUniformMatrix(glm::value_ptr(tr), 4, "uModel");
			shader.SetUniformMatrix(glm::value_ptr(modelInvTranspose), 4, "uModelInvTr");

			shader.SetUniformVector(glm::value_ptr(colorInfo.specular), 3, "uSpecular");
			shader.SetUniformVector(glm::value_ptr(colorInfo.diffuse), 3, "uDiffuse");
			shader.SetUniform(colorInfo.gloss, "uGlossiness");
			shader.SetUniform(colorInfo.opacity, "uAlpha");

			m.Draw();
		}

		void DrawVector(glm::vec3 const& vec, glm::vec3 const& startPt, ShaderProgram const& prg, ColorInfo const& colorInfo)
		{
			glm::mat4 modelTr{};

			float const mag = glm::length(vec);
			if (EpsilonEqual(mag, 0.0f)) {
				return;
			}
			glm::vec3 const dir = vec * (1.0f / mag);
			glm::mat4 const rot = glm::toMat4(glm::rotation(glm::vec3(0, 0, 1), dir));

			// Draw cylinder
			{
				modelTr = glm::translate(glm::mat4(1), startPt) *		   // translate s.t. bottom of cylinder is at startPt
					rot *											   // rotate to point in direction of vec
					glm::translate(glm::mat4(1), glm::vec3(0, 0, 0.5f * mag)) *  // shift to align "bottom" of cylinder with origin
					glm::scale(glm::mat4(1), glm::vec3(0.05f, 0.05f, 0.5f * mag)); // scale to make skinny and as long as magnitude of vec

				drb::Mesh::ScopedUse use{ drb::Mesh::Cylinder() };
				DrawMesh(use.mesh, prg, modelTr, colorInfo);
			}

			// Draw cone
			{
				modelTr = glm::translate(glm::mat4(1), startPt + vec) * rot * glm::scale(glm::mat4(1), glm::vec3(0.2f));
				drb::Mesh::ScopedUse use{ drb::Mesh::Cone()};
				DrawMesh(use.mesh, prg, modelTr, colorInfo);
			}
		}

		void DrawFaceClipPlanes(Convex const& cvx, Uint8 faceIdx, ShaderProgram const& prg, Mat4 const& tr, ColorInfo const& colorInfo)
		{
			// Draw clip planes
			cvx.ForEachEdgeOfFace(faceIdx, [&](Convex::HalfEdge edge) {

					// Create a plane orthogonal to refFace and containing
					// endpoints of edge
					Convex::HalfEdge const twin = cvx.GetEdge(edge.twin);
					Vec3 const p0 = tr * Vec4(cvx.GetVert(edge.origin), 1);
					Vec3 const p1 = tr * Vec4(cvx.GetVert(twin.origin), 1);
					Vec3 const outwardNormal = Normalize(glm::cross(p1 - p0, cvx.GetFace(faceIdx).plane.n));
					Plane const edgePlane = Plane::Make(outwardNormal, p0); // normal pointing "outward" away from face center

					// Draw clip plane
					{
						drb::Mesh::ScopedUse use{ drb::Mesh::Quad() };
						Mat4 const clipPlaneTr = glm::translate(glm::mat4(1), 0.5f * glm::vec3(p1 + p0))
							* glm::mat4(UnitVectorToBasis4(outwardNormal))
							* glm::scale(glm::mat4(1), glm::vec3(8));

						DrawMesh(use.mesh, prg, clipPlaneTr, colorInfo);
					}
					// Draw normal of clip plane
					{
						DrawVector(glm::vec3(outwardNormal), 0.5f * glm::vec3(p1 + p0), prg, colorInfo);
					}
			});
		}


		void DrawAABB(AABB const& b, ShaderProgram const& prg, ColorInfo const& colorInfo)
		{
			glm::mat4 const modelTr = glm::translate(glm::mat4(1), glm::vec3(b.Center())) * glm::scale(glm::mat4(1), glm::vec3(b.Halfwidths()));

			drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
			DrawMesh(use.mesh, prg, modelTr, colorInfo);
		}
		void DrawOBB(AABB const& b, glm::mat4 const& rot, ShaderProgram const& prg, ColorInfo const& colorInfo)
		{
			glm::mat4 const modelTr = glm::translate(glm::mat4(1), glm::vec3(b.Center())) * rot * glm::scale(glm::mat4(1), glm::vec3(b.Halfwidths()));

			drb::Mesh::ScopedUse use{ drb::Mesh::Cube() };
			DrawMesh(use.mesh, prg, modelTr, colorInfo);
		}

		drb::Mesh MakeRenderMesh(Convex const& cvx)
		{
			if (cvx.NumVerts() == 0) {
				ASSERT(false, "Convex hull cannot be empty");
				return drb::Mesh{};
			}

			// Construct vertex buffer interleaved with vertex normals and uvs
			std::vector<float> verts;
			verts.reserve(static_cast<size_t>(cvx.NumVerts()) * 8u);

			std::vector<glm::uvec3> indices;
			indices.reserve(cvx.NumFaces());

			// Triangulate faces via "fan triangulation" and compute vertex normals
			auto const faces = cvx.GetFaces();
			for (auto&& face : faces) {

				Uint32 vertCount = 0u;
				Uint32 faceIdx = static_cast<Uint32>(verts.size()) / 8u;

				auto const e0 = cvx.GetEdge(face.edge);

				auto PushVert = [&verts, &face, &cvx, &vertCount](unsigned vertIdx) {
					// Insert world pos
					glm::vec3 const v = glm::vec3( cvx.GetVert(vertIdx) );
					verts.push_back(v.x);
					verts.push_back(v.y);
					verts.push_back(v.z);

					// Insert vertex normal
					glm::vec3 const n = glm::vec3(face.plane.n);
					verts.push_back(n.x);
					verts.push_back(n.y);
					verts.push_back(n.z);

					// Insert uv
					verts.push_back(0.0f);
					verts.push_back(0.0f);

					++vertCount;
				};

				// Traverse the face while enumerating and pushing vertices
				auto eCurr = e0;
				do {

					PushVert(eCurr.origin);
					eCurr = cvx.GetEdge(eCurr.next);

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

		static inline glm::mat4 SphereTransform(Sphere const& sph, glm::mat4 const& tr)
		{
			return tr * glm::mat4(sph.Transform()) * glm::scale(glm::mat4(1), glm::vec3(sph.r));
		}

		static inline glm::mat4 CubeTransform(glm::vec3 const& halfwidths, glm::mat4 const& tr)
		{
			return tr * glm::scale(glm::mat4(1), halfwidths);
		}

		static void DrawCapsule(Capsule const& cap, ShaderProgram const& prg, glm::mat4 const& tr, ColorInfo const& colorInfo)
		{
			glm::mat4 modelTr{};

			// Draw cylinder
			{
				modelTr = tr * glm::mat4(cap.Transform()) * glm::scale(glm::mat4(1), glm::vec3(Vec3(cap.r - 0.01, cap.SegmentHalfLength(), cap.r - 0.01))) * glm::rotate(glm::mat4(1), 0.5f * 3.142f, glm::vec3(1, 0, 0)); // first rotation is to correct z-up model

				drb::Mesh::ScopedUse use{ drb::Mesh::Cylinder() };
				DrawMesh(use.mesh, prg, modelTr, colorInfo);
			}

			// Draw spheres
			{
				Segment segment{ cap.CentralSegment().Transformed(tr) };
				drb::Mesh::ScopedUse use{ drb::Mesh::Sphere() };

				// Draw bottom sphere
				modelTr = glm::translate(glm::mat4(1), glm::vec3(segment.b)) * glm::scale(glm::mat4(1), glm::vec3(cap.r));
				DrawMesh(use.mesh, prg, modelTr, colorInfo);

				// Draw top sphere
				modelTr = glm::translate(glm::mat4(1), glm::vec3(segment.e)) * glm::scale(glm::mat4(1), glm::vec3(cap.r));
				DrawMesh(use.mesh, prg, modelTr, colorInfo);

			}
		}
	}
}

