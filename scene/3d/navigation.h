/*************************************************************************/
/*  navigation.h                                                         */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "core/math/a_star.h"
#include "scene/3d/navigation_mesh.h"
#include "scene/3d/spatial.h"

class Navigation : public Spatial {

	GDCLASS(Navigation, Spatial);

	struct Edge {

		Vector3 left;
		Vector3 right;

		_FORCE_INLINE_ Edge() {}

		_FORCE_INLINE_ Edge(const Vector3 &p_left, const Vector3 &p_right) :
				left(p_left), right(p_right) {}

		_FORCE_INLINE_ Vector3 center() const { return (left + right) * 0.5; }

		_FORCE_INLINE_ bool operator==(const Edge &p_s) const {
			return (left == p_s.left && right == p_s.right) || (left == p_s.right && right == p_s.left);
		}

		_FORCE_INLINE_ float triarea2(const Vector3 &p_point, const Vector3 &p_up) const {
			// If < 0 then the point is to the right, if > 0 the point is to the left, if equal to 0 the point is on the edge
			return ((p_point - left ).cross(p_point - right)).dot(p_up);
		}

		_FORCE_INLINE_ void relative_to(const Edge &p_s, const Vector3 &p_up) { // p_s must cross the edge
			if (p_s.triarea2(left, p_up) < 0)
				SWAP(left, right);
		}
	};

	struct EdgeKey : public Edge { // Unique key for each possible edge

		_FORCE_INLINE_ bool operator<(const EdgeKey &p_eKey) const { // Used by the binary search tree of Map
			return (left == p_eKey.left) ? (right < p_eKey.right) : (left < p_eKey.left);
		};

		_FORCE_INLINE_ EdgeKey() {}

		_FORCE_INLINE_ EdgeKey(const Vector3 &p_left, const Vector3 &p_right) : Edge(p_left, p_right) {
			if (right < left) // left is always less than right
				SWAP(left, right);
		}
	};

	struct NavMesh {

		Object *owner;
		Transform xform;
		bool linked;
		Ref<NavigationMesh> navmesh;
		List<int> polygon_ids;
	};

	struct Polygon {

		Vector<Vector3> points;

		int id;
		Vector3 center;
		NavMesh *owner;

		Vector3 get_closest_point(const Vector3 &p_point) const;
		Edge get_connection_to(const Polygon *p_poly) const;
	};

	Map<EdgeKey, List<int> > connections;
	Map<int, Polygon> polygons;

	AStar *aStar;

	void _navmesh_link(int p_id);
	void _navmesh_unlink(int p_id);

	Map<int, NavMesh> navmesh_map;

	Vector3 up;

	int last_mesh_id;
	int last_polygon_id;

	void _clip_path(Vector<Vector3> &r_path, Vector3 &p_from, const Vector3 &p_to, const Vector<Edge> &portals, int &p_from_index, const int p_to_index);
	Navigation::Polygon *_get_closest_polygon(const Vector3 &p_point) const;

protected:
	static void _bind_methods();

public:
	void set_up_vector(const Vector3 &p_up);
	Vector3 get_up_vector() const;

	//API should be as dynamic as possible
	int navmesh_add(const Ref<NavigationMesh> &p_mesh, const Transform &p_xform, Object *p_owner = NULL);
	void navmesh_set_transform(int p_id, const Transform &p_xform);
	void navmesh_remove(int p_id);

	Vector<Vector3> get_simple_path(const Vector3 &p_start, const Vector3 &p_end, bool p_optimize = true);
	Vector3 get_closest_point_to_segment(const Vector3 &p_from, const Vector3 &p_to, const bool &p_use_collision = false);
	Vector3 get_closest_point(const Vector3 &p_point);
	Vector3 get_closest_point_normal(const Vector3 &p_point);
	Object *get_closest_point_owner(const Vector3 &p_point);

	Navigation();
	~Navigation();
};

#endif // NAVIGATION_H
