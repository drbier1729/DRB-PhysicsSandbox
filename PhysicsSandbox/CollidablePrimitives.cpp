#include "stdafx.h"
#include "CollidablePrimitives.h"
//
//#include "RigidBody.h"
//#include "CollisionInfo.h"
//#include "CollisionProperties.h"
//#include "PhysicsManager.h"
//
//void CollidablePrimitive::ComputeTransform() noexcept {
//	transform = (owner != nullptr) ? 
//					owner->TransformMatrix() * offset :
//					offset;
//}
//
//void CollidablePrimitive::SetOwner(RigidBody* rb) noexcept {
//	SIK_ASSERT(owner == nullptr, "This primitive already has an owner.");
//	
//	owner = rb;
//	owner->collidable_primitive = this;
//	owner->ComputeInternals();
//}
//
//// Helpers
//inline static Float32 PenetrationOnAxis(Box* b0, Box* b1, Vec3 const& axis, Vec3 const& c0_to_c1) {
//	Float32 project_0 = b0->ProjectOntoAxis(axis);
//	Float32 project_1 = b1->ProjectOntoAxis(axis);
//
//	Float32 dist = glm::abs(glm::dot(axis, c0_to_c1));
//
//	return project_0 + project_1 - dist;
//}
//
//static Bool TryAxis(Box* b0, Box* b1, Vec3 const& axis, Vec3 const& c0_to_c1, Uint32 index,
//	Float32& out_smallest_penetration, Uint32& out_smallest_case) {
//
//	if (auto axis_is_near_zero = glm::equal(axis, Vec3(0.f), 0.001f); 
//		glm::all(axis_is_near_zero)) { 
//		return true; 
//	}
//
//	Vec3 normalized_axis = glm::normalize(axis);
//
//	Float32 penetration = PenetrationOnAxis(b0, b1, normalized_axis, c0_to_c1);
//
//	if (penetration < 0.0f) { return false; }
//	if (penetration < out_smallest_penetration) {
//		out_smallest_penetration = penetration;
//		out_smallest_case = index;
//	}
//	return true;
//}
//
//static Vec3 GenContactPt(Vec3 const& pt0, Vec3 const& axis0, Float32 size0,
//	Vec3 const& pt1, Vec3 const& axis1, Float32 size1,
//	Bool use0) {
//
//	// (Note: Dylan) I have no clue what the fuck this is doing. See Game Physics Engine Programming pg 326, 
//	// and https://github.com/idmillington/cyclone-physics/blob/master/src/collide_fine.cpp ... 
//	// good luck. Maybe Jolt, Box2D or Realtime Collision Detection will have better code samples!
//
//
//	Float32 dot_product_edges = glm::dot(axis0, axis1);
//	Float32 sqr_len0 = glm::length2(axis0);
//	Float32 sqr_len1 = glm::length2(axis1);
//	Float32 denom = sqr_len0 * sqr_len1 - dot_product_edges * dot_product_edges;
//
//	if (denom < 0.0001f) {
//		return use0 ? pt0 : pt1;
//	}
//
//	Vec3 diff = pt0 - pt1;
//	Float32 dot_product_diff0 = glm::dot(diff, axis0);
//	Float32 dot_product_diff1 = glm::dot(diff, axis1);
//
//	Float32 a = (dot_product_edges * dot_product_diff1 - sqr_len1 * dot_product_diff0) / denom;
//	Float32 b = (sqr_len0 * dot_product_diff1 - dot_product_edges * dot_product_diff0) / denom;
//
//	if (a > size0 || a < -size0 || b > size1 || b < -size1) {
//		return use0 ? pt0 : pt1;
//	}
//
//	return 0.5f * a * (pt0 + axis0) + 0.5f * b * (pt1 + axis1);
//}
//
//
//
//#define COLLISION_LOG_IMPLEMENTED \
////SIK_INFO("Narrow phase collision check called for shapes {} and {}", GetName(), first->GetName());
//
//#define COLLISION_LOG_NOT_IMPLEMENTED \
//SIK_ERROR("Narrow phase collision implemenation not defined for shapes {} and {}", GetName(), first->GetName()); \
//SIK_ASSERT(false, "Collision implementation not defined. See error log for more."); \
//return false;
//
//Bool Point::Collide(Point* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Point::Collide(Sphere* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Point::Collide(Capsule* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Point::Collide(Plane* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Point::Collide(Box* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Point::Collide(Assembly* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//
//Bool Sphere::Collide(Sphere* first) {
//	COLLISION_LOG_IMPLEMENTED
//
//	// The last column of the transform matrix will be the position vector
//	Vec3 pos_first = glm::column(first->transform, 3);
//	Vec3 pos_second = glm::column(transform, 3);
//
//	Vec3 midline = pos_first - pos_second;
//	Float32 size = glm::length(midline);
//
//	if (size <= 0.0f || size >= first->radius + radius) {
//		return false;
//	}
//
//	Vec3 normal = midline * 1.0f / size;
//
//	ContactBatch batch{ first->owner, owner };
//	batch.AddContact(
//		pos_first + 0.5f * midline, //point 
//		normal, //normal
//		first->radius + radius - size); //penetration
//	
//	p_physics_manager->AddContactBatch(batch);
//	return true;
//}
//Bool Sphere::Collide(Capsule* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Sphere::Collide(Plane* first) { 
//	COLLISION_LOG_IMPLEMENTED
//
//	Vec3 pos_second = glm::column(transform, 3);
//
//	Float32 dist = glm::dot(pos_second, first->normal_dir) - radius - first->signed_dist;
//	if (dist >= 0) {
//		return false;
//	}
//
//	ContactBatch batch{ first->owner, owner };
//	batch.AddContact(
//		pos_second - first->normal_dir * (dist + radius), //point
//		first->normal_dir, //normal
//		-dist); //penetration
//
//	p_physics_manager->AddContactBatch(batch);
//	return true;
//}
//Bool Sphere::Collide(Box* first) {
//	COLLISION_LOG_IMPLEMENTED
//
//	// Center of the sphere in the box's local coordinates
//	Vec3 pos_second = glm::column(transform, 3);
//	Vec3 rel_pos_second = Vec3( glm::inverse(first->transform) * Vec4(pos_second, 1) );
//	
//	// Early out if sphere is not intersecting with the box
//	if (auto no_intersect = glm::greaterThan(glm::abs(rel_pos_second) - Vec3(radius), first->halfwidth_extents); 
//		glm::any(no_intersect)) {
//		return false;
//	}
//
//	// Point on the box closest to the sphere, in box's local coordinates
//	Vec3 closest_pt = glm::clamp(rel_pos_second, -first->halfwidth_extents, first->halfwidth_extents);
//
//	// Check if we're intersecting at the closest point
//	Float32 dist = glm::length(closest_pt - rel_pos_second);
//	if (dist > radius) {
//		return false;
//	}
//
//	// Convert back to world space
//	closest_pt = Vec3( first->transform * Vec4(closest_pt, 1) );
//
//	// Generate contact
//	ContactBatch batch{ first->owner, owner };
//	batch.AddContact(
//		closest_pt, //point
//		glm::normalize(closest_pt - pos_second), //normal
//		radius - glm::sqrt(dist)); //penetration
//
//	p_physics_manager->AddContactBatch(batch);
//	return true;
//}
//Bool Sphere::Collide(Assembly* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//
//Bool Capsule::Collide(Capsule* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Capsule::Collide(Plane* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Capsule::Collide(Box* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Capsule::Collide(Assembly* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//
//Bool Plane::Collide(Plane* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//Bool Plane::Collide(Box* first) { 
//	COLLISION_LOG_IMPLEMENTED
//
//	// Early out
//	Float32 projected_radius = first->ProjectOntoAxis(normal_dir);
//	Float32 dist = glm::dot(normal_dir, Vec3(glm::column(first->transform, 3))) - projected_radius;
//	if (dist > signed_dist) {
//		return false;
//	}
//
//	// Generate vertices based on the box's halfwidth extents and transform
//	Box::Vertices vertices{};
//	first->GenerateVertices(vertices);
//
//	ContactBatch batch{ first->owner, owner };
//	for (auto i = 0; i < 8; ++i) {
//
//		Float32 vert_dist = glm::dot(vertices[i], normal_dir);
//		
//		if (vert_dist <= signed_dist) {
//			batch.AddContact(
//					normal_dir * (vert_dist - signed_dist) + vertices[i], //position = halfway between vert and plane
//					normal_dir, //normal
//					signed_dist - vert_dist); //penetration
//		}
//	}
//
//	p_physics_manager->AddContactBatch(batch);
//	return true;
//}
//Bool Plane::Collide(Assembly* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//
//Bool Box::Collide(Box* first) { 
//	COLLISION_LOG_IMPLEMENTED
//
//	Vec3 c0_to_c1 = glm::column(transform, 3) - glm::column(first->transform, 3);
//
//	Float32 penetration = std::numeric_limits<Float32>::max();
//	Uint32 best_index = std::numeric_limits<Uint32>::max();
//
//	// Check all overlaps
//	Vec3 axes_first[] = { 
//		glm::column(first->transform, 0),
//		glm::column(first->transform, 1),
//		glm::column(first->transform, 2),
//	};
//	Vec3 axes_second[] = { 
//		glm::column(transform, 0),
//		glm::column(transform, 1),
//		glm::column(transform, 2),
//	};
//
//	if (!TryAxis(first, this, axes_first[0], c0_to_c1, 0, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, axes_first[1], c0_to_c1, 1, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, axes_first[2], c0_to_c1, 2, penetration, best_index)) { return false; }
//	
//	if (!TryAxis(first, this, axes_second[0], c0_to_c1, 3, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, axes_second[1], c0_to_c1, 4, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, axes_second[2], c0_to_c1, 5, penetration, best_index)) { return false; }
//	
//	Uint32 best_single_axis = best_index;
//
//	if (!TryAxis(first, this, glm::cross(axes_first[0], axes_second[0]), c0_to_c1, 6, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[0], axes_second[1]), c0_to_c1, 7, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[0], axes_second[2]), c0_to_c1, 8, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[1], axes_second[0]), c0_to_c1, 9, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[1], axes_second[1]), c0_to_c1, 10, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[1], axes_second[2]), c0_to_c1, 11, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[2], axes_second[0]), c0_to_c1, 12, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[2], axes_second[1]), c0_to_c1, 13, penetration, best_index)) { return false; }
//	if (!TryAxis(first, this, glm::cross(axes_first[2], axes_second[2]), c0_to_c1, 14, penetration, best_index)) { return false; }
//
//	SIK_ASSERT(best_index != std::numeric_limits<Uint32>::max(), "No result found!");
//
//	// We know we have a collision so start a contact batch
//	ContactBatch batch{ first->owner, owner };
//
//	// Collision: Vertex of first with Face of second
//	if (best_index < 3) {
//
//		Vec3 normal = axes_first[best_index];
//		if (glm::dot(normal, c0_to_c1) > 0) {
//			normal *= -1.0f;
//		}
//
//		Vec3 vert = halfwidth_extents;
//		if (glm::dot(axes_second[0], normal) < 0) { vert.x = -vert.x; }
//		if (glm::dot(axes_second[1], normal) < 0) { vert.y = -vert.y; }
//		if (glm::dot(axes_second[2], normal) < 0) { vert.z = -vert.z; }
//
//		batch.AddContact(
//			Vec3(transform * Vec4(vert, 1)),
//			normal,
//			penetration);
//	}
//	// Collision: Vertex of second with Face of first
//	else if (best_index < 6) {
//
//		Vec3 normal = axes_second[best_index-3];
//		if (glm::dot(normal, c0_to_c1) > 0) {
//			normal *= -1.0f;
//		}
//
//		Vec3 vert = halfwidth_extents;
//		if (glm::dot(axes_first[0], normal) < 0) { vert.x = -vert.x; }
//		if (glm::dot(axes_first[1], normal) < 0) { vert.y = -vert.y; }
//		if (glm::dot(axes_first[2], normal) < 0) { vert.z = -vert.z; }
//
//		batch.AddContact(
//			Vec3(first->transform * Vec4(vert, 1)),
//			normal,
//			penetration);
//	}
//	// Collision: Edge-Edge
//	else {
//
//		best_index -= 6;
//		Uint32 idx_first = best_index / 3;
//		Uint32 idx_second = best_index % 3;
//		Vec3 axis_first = axes_first[idx_first];
//		Vec3 axis_second = axes_first[idx_second];
//
//		Vec3 axis = glm::normalize( glm::cross(axis_first, axis_second) );
//		if (glm::dot(axis, c0_to_c1) > 0) { axis *= -1.0f; }
//
//		// Compute which edges are colliding
//		Vec3 pt_on_edge_first = first->halfwidth_extents;
//		Vec3 pt_on_edge_second = halfwidth_extents;
//		for (Uint32 i = 0; i < 3; ++i) {
//			if (i == idx_first) { 
//				pt_on_edge_first[i] = 0; 
//			}
//			else if (glm::dot(axes_first[i], axis) < 0) { 
//				pt_on_edge_first[i] *= -1.0f; 
//			}
//			
//			if (i == idx_second) { 
//				pt_on_edge_second[i] = 0; 
//			}
//			else if (glm::dot(axes_second[i], axis) < 0) { 
//				pt_on_edge_second[i] *= -1.0f; 
//			}
//		}
//
//		// Move points on edges into world coords
//		pt_on_edge_first = Vec3( first->transform * Vec4(pt_on_edge_first, 1));
//		pt_on_edge_second = Vec3( transform * Vec4(pt_on_edge_second, 1));
//		
//		Vec3 vert = GenContactPt(
//						pt_on_edge_first, axis_first, first->halfwidth_extents[idx_first],
//						pt_on_edge_second, axis_second, first->halfwidth_extents[idx_second],
//						best_single_axis > 2);
//
//		batch.AddContact(vert, axis, penetration);
//	}
//
//	p_physics_manager->AddContactBatch(batch);
//	return true;
//}
//Bool Box::Collide(Assembly* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//
//Bool Assembly::Collide(Assembly* first) { COLLISION_LOG_NOT_IMPLEMENTED }
//
//
//// Order-swapped implementations
//Bool Sphere::Collide(Point* first) { return first->Collide(this); }
//
//Bool Capsule::Collide(Point* first) { return first->Collide(this); }
//Bool Capsule::Collide(Sphere* first) { return first->Collide(this); }
//
//Bool Plane::Collide(Point* first) { return first->Collide(this); }
//Bool Plane::Collide(Sphere* first) { return first->Collide(this); }
//Bool Plane::Collide(Capsule* first) { return first->Collide(this); }
//
//Bool Box::Collide(Point* first) { return first->Collide(this); }
//Bool Box::Collide(Sphere* first) { return first->Collide(this); }
//Bool Box::Collide(Capsule* first) { return first->Collide(this); }
//Bool Box::Collide(Plane* first) { return first->Collide(this); }
//
//Bool Assembly::Collide(Point* first) { return first->Collide(this); }
//Bool Assembly::Collide(Sphere* first) { return first->Collide(this); }
//Bool Assembly::Collide(Capsule* first) { return first->Collide(this); }
//Bool Assembly::Collide(Plane* first) { return first->Collide(this); }
//Bool Assembly::Collide(Box* first) { return first->Collide(this); }
//
//#undef COLLISION_LOG_NOT_IMPLEMENTED
//
//
//#include "Mesh.h"
//
//void Point::DebugDraw(Uint32) const noexcept {}
//
//void Sphere::DebugDraw(Uint32 shader_id) const noexcept {
//	auto const& sphere_mesh = Mesh::Sphere();
//	sphere_mesh.Use();
//	sphere_mesh.Draw();
//	sphere_mesh.Unuse();
//}
//
//void Capsule::DebugDraw(Uint32) const noexcept {}
//
//void Plane::DebugDraw(Uint32 shader_id) const noexcept {
//	auto const & plane_mesh = Mesh::Plane();
//	plane_mesh.Use();
//	plane_mesh.Draw();
//	plane_mesh.Unuse();
//}
//
//void Box::DebugDraw(Uint32 shader_id) const noexcept {
//	auto const& cube_mesh = Mesh::Cube();
//	cube_mesh.Use();
//	cube_mesh.Draw();
//	cube_mesh.Unuse();
//}
//
//void Assembly::DebugDraw(Uint32) const noexcept {}
//
//
//
//#include "Reflector.h"
//
//BEGIN_ATTRIBUTES_FOR(Point)
//END_ATTRIBUTES
//
//BEGIN_ATTRIBUTES_FOR(Sphere)
//DEFINE_MEMBER(Float32, radius)
//END_ATTRIBUTES
//
//BEGIN_ATTRIBUTES_FOR(Capsule)
//DEFINE_MEMBER(Vec3, p0)
//DEFINE_MEMBER(Vec3, p1)
//DEFINE_MEMBER(Float32, radius);
//END_ATTRIBUTES
//
//BEGIN_ATTRIBUTES_FOR(Plane)
//DEFINE_MEMBER(Vec3, normal_dir)
//DEFINE_MEMBER(Float32, signed_dist)
//END_ATTRIBUTES
//
//BEGIN_ATTRIBUTES_FOR(Box)
//DEFINE_MEMBER(Vec3, halfwidth_extents)
//END_ATTRIBUTES
//
//BEGIN_ATTRIBUTES_FOR(Assembly)
//END_ATTRIBUTES
