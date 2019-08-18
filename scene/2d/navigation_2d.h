/*************************************************************************/
/*  navigation_2d.h                                                      */
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

#ifndef NAVIGATION_2D_H
#define NAVIGATION_2D_H

#include "core/math/a_star.h"
#include "scene/2d/navigation_polygon.h"
#include "scene/2d/node_2d.h"

#include <iostream>

class Navigation2D : public Node2D {

	GDCLASS(Navigation2D, Node2D);

	struct Edge {

		Vector2 left;
		Vector2 right;

		_FORCE_INLINE_ Edge() {}

		_FORCE_INLINE_ Edge(const Vector2 &p_left, const Vector2 &p_right) :
				left(p_left), right(p_right) {}

		_FORCE_INLINE_ Vector2 center() const { return (left + right) * 0.5; }

		_FORCE_INLINE_ bool operator==(const Edge &p_s) const {
			return (left == p_s.left && right == p_s.right) || (left == p_s.right && right == p_s.left);
		}

		_FORCE_INLINE_ float triarea2(const Vector2 &p_point) const { // Returns two times the area of the triangle created with p_point
			// If < 0 then the point is to the right, if > 0 the point is to the left, if equal to 0 the point is on the edge
			return (p_point.x - left.x) * (right.y - left.y) - (p_point.y - left.y) * (right.x - left.x);
		}

		_FORCE_INLINE_ Vector2 closest_point(const Vector2 &p_point) const { // Returns the point inside this edge that's closest to p_point
			Vector2 segment[2] = { left, right };
			return Geometry::get_closest_point_to_segment_2d(p_point, segment);
		}

		_FORCE_INLINE_ void relative_to(const Edge &p_s) { // p_s must cross the edge
			if (p_s.triarea2(left) < 0)
				SWAP(left, right);
		}
	};

	struct EdgeKey : public Edge { // Unique key for each possible edge

		_FORCE_INLINE_ bool operator<(const EdgeKey &p_eKey) const { // Used by the binary search tree of Map
			return (left == p_eKey.left) ? (right < p_eKey.right) : (left < p_eKey.left);
		};

		_FORCE_INLINE_ EdgeKey() {}

		_FORCE_INLINE_ EdgeKey(const Vector2 &p_left, const Vector2 &p_right) : Edge(p_left, p_right) {
			if (right < left) // left is always less than right
				SWAP(left, right);
		}
	};

	struct NavMesh {

		Object *owner;
		Transform2D xform;
		bool linked;
		Ref<NavigationPolygon> navpoly;
		List<int> polygon_ids;
	};

	struct Polygon {

		Vector<Vector2> points;

		int id;
		Vector2 center;
		NavMesh *owner;
		float radius;

		bool contains(const Vector2 &p_point) const;
		Vector2 get_closest_point(const Vector2 &p_point) const;
		Edge get_connection_to(const Polygon *p_poly) const;
	};

	Map<EdgeKey, List<int> > connections;
	Map<int, Polygon> polygons;

	AStar *aStar;

	void _navpoly_link(int p_id);
	void _navpoly_unlink(int p_id);

	Map<int, NavMesh> navpoly_map;

	int last_mesh_id;
	int last_polygon_id;

protected:
	static void _bind_methods();

public:
	//API should be as dynamic as possible
	int navpoly_add(const Ref<NavigationPolygon> &p_mesh, const Transform2D &p_xform, Object *p_owner = NULL);
	void navpoly_set_transform(int p_id, const Transform2D &p_xform);
	void navpoly_remove(int p_id);

	Vector<Vector2> get_simple_path(const Vector2 &p_start, const Vector2 &p_end, bool p_optimize = true);
	Navigation2D::Polygon *get_closest_polygon(const Vector2 &p_point) const;
	Vector2 get_closest_point(const Vector2 &p_point) const;
	Object *get_closest_point_owner(const Vector2 &p_point) const;

	Navigation2D();
	~Navigation2D();
};

#endif // NAVIGATION_2D_H
