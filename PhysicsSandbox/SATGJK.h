#ifndef DRB_SATGJK_H
#define DRB_SATGJK_H

namespace drb::physics {
		
		struct Convex;
		struct Plane;
		struct Segment;
		
		namespace util {
		
			// -----------------------------------------------------------------
			// Separating Axis Test
			// -----------------------------------------------------------------
			struct FaceQuery {
				Float32 separation = std::numeric_limits<Float32>::lowest();
				Vec3    normal = Vec3(NAN);
				Int16   index = -1;
			};

			struct EdgeQuery {
				Float32 separation = std::numeric_limits<Float32>::lowest();
				Vec3	normal = Vec3(NAN);
				Int16   indexA = -1;
				Int16   indexB = -1;
			};

			// Note that these need to be called sequentially for the SAT: 
			//	QueryFace(A, B)
			//	QueryFace(B, A)
			//	QueryEdge(A, B)
			FaceQuery SATQueryFaceDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
			EdgeQuery SATQueryEdgeDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
			Float32	  SeparationOnAxis(Vec3 const& axis, Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);


			// -----------------------------------------------------------------
			// GJK
			// -----------------------------------------------------------------
			struct Simplex
			{
				enum class Type : Uint8 {
					NONE = 0,
					Point = 1,
					Line = 2,
					Triangle = 3,
					Tet = 4
				};

				Vec3 verts[4] = {};
				Uint8 size = 0;
				Bool containsOrigin = false;

				inline Type GetType() const;
			};

			// GJK: Convex-Convex
			Simplex GJK(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB, Vec3 const& dir, Simplex const& initialSimplex = {});
			
			// GJK: Point-Convex
			Simplex GJK(Vec3 const& A, Convex const& B, Mat4 const& trB,
				Vec3 const& dir, Simplex const& initialSimplex = {});
			
			// GJK: Segment-Convex
			Simplex GJK(Segment const& A, Convex const& B, Mat4 const& trB,
				Vec3 const& dir, Simplex const& initialSimplex = {});
		}
}

#include "SATGJK.inl"

#endif