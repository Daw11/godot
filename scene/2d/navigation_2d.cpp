/*************************************************************************/
/*  navigation_2d.cpp                                                    */
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

#include "navigation_2d.h"

#define USE_ENTRY_POINT

#include <valgrind/callgrind.h>
#include <ctime>
#include <iostream>

_FORCE_INLINE_ bool Navigation2D::Polygon::contains(const Vector2 &p_point) const {

	if (p_point.distance_squared_to(center) > radius) // The point can't be inside this polygon
		return false;

	for (int i = 2; i < points.size(); i++) { // All the polygons are convex

		if (Geometry::is_point_in_triangle(p_point, points[0], points[i - 1], points[i]))
			return true;
	}

	return false;
}

Vector2 Navigation2D::Polygon::get_closest_point(const Vector2 &p_point) const {

	if (contains(p_point))
		return p_point;

	const int points_s = points.size();
	float closest_d = Math_INF;
	Vector2 p = Vector2();

	for (int i = 0; i < points_s; i++) {

		Edge s(points[i], points[(i + 1) % points_s]);
		Vector2 closest_p = s.closest_point(p_point);
		float d = closest_p.distance_squared_to(p_point);

		if (d < closest_d) {

			p = closest_p;
			closest_d = d;
		}
	}

	return p;
}

// Returns the edge that connects the two polygons
Navigation2D::Edge Navigation2D::Polygon::get_connection_to(const Polygon *p_poly) const {

	const int size1 = points.size();
	const int size2 = p_poly->points.size();

	for (int i = 0; i < size1; i++) {

		Navigation2D::Edge e1 = Edge(points[i], points[(i + 1) % size1]);

		for (int j = 0; j < size2; j++) {

			Navigation2D::Edge e2 = Edge(p_poly->points[j], p_poly->points[(j + 1) % size2]);
			if (e1 == e2)
				return e1;
		}
	}

	ERR_EXPLAIN("The polygon " + itos(id) + " isn't connected with polygon " + itos(p_poly->id));
	ERR_FAIL_V(Navigation2D::Edge());
}

void Navigation2D::_navpoly_link(int p_id) {

	ERR_FAIL_COND(!navpoly_map.has(p_id));
	NavMesh &nm = navpoly_map[p_id];
	ERR_FAIL_COND(nm.linked);

	PoolVector<Vector2> vertices = nm.navpoly->get_vertices();
	int len = vertices.size();
	if (len == 0)
		return;

	PoolVector<Vector2>::Read r = vertices.read();

	for (int i = 0; i < nm.navpoly->get_polygon_count(); i++) {

		//build
		last_polygon_id++;
		Map<int, Polygon>::Element *P = polygons.insert(last_polygon_id, Polygon());
		Polygon &p = P->get();
		p.id = last_polygon_id;
		p.owner = &nm;

		Vector<int> poly = nm.navpoly->get_polygon(i);
		const int *indices = poly.ptr();
		int plen = poly.size();

		bool valid = true;
		Vector2 center;

		p.points.resize(plen);

		for (int j = 0; j < plen; j++) {

			int idx = indices[j];
			if (idx < 0 || idx >= len) {
				valid = false;
				break;
			}

			Vector2 point = nm.xform.xform(r[idx]);
			p.points.write[j] = point.floor();
			center += point;
		}

		if (!valid) {
			polygons.erase(p.id);
			ERR_CONTINUE(!valid);
		}

		p.center = center / plen;
		nm.polygon_ids.push_back(p.id);

		p.radius = 0; // Get the squared radius of the circle enclosing the polygon
		for (int j = 0; j < plen; j++)
			p.radius = MAX(p.radius, p.center.distance_squared_to(p.points[j]));

		aStar->add_point(p.id, Vector3(p.center.x, p.center.y, 0));

		//connect
		for (int j = 0; j < plen; j++) {

			EdgeKey ek(p.points[j], p.points[(j + 1) % plen]);
			Map<EdgeKey, List<int> >::Element *C = connections.find(ek);
			if (!C)
				C = connections.insert(ek, List<int>());

			for (List<int>::Element *I = C->get().front(); I; I = I->next())
				aStar->connect_points(p.id, I->get());

			C->get().push_back(p.id);
		}
	}

	nm.linked = true;
}

void Navigation2D::_navpoly_unlink(int p_id) {

	ERR_FAIL_COND(!navpoly_map.has(p_id));
	NavMesh &nm = navpoly_map[p_id];
	ERR_FAIL_COND(!nm.linked);

	for (List<int>::Element *E = nm.polygon_ids.front(); E; E = E->next()) {

		Polygon &p = polygons[E->get()];

		aStar->remove_point(p.id);

		int points_s = p.points.size();

		for (int i = 0; i < points_s; i++) {

			EdgeKey ek(p.points[i], p.points[(i + 1) % points_s]);
			Map<EdgeKey, List<int> >::Element *C = connections.find(ek);
			ERR_FAIL_COND(!C);

			C->get().erase(p.id);
		}

		polygons.erase(p.id);
	}

	nm.polygon_ids.clear();

	nm.linked = false;
}

int Navigation2D::navpoly_add(const Ref<NavigationPolygon> &p_mesh, const Transform2D &p_xform, Object *p_owner) {

	NavMesh nm;
	nm.linked = false;
	nm.navpoly = p_mesh;
	nm.xform = p_xform;
	nm.owner = p_owner;

	last_mesh_id++;

	navpoly_map[last_mesh_id] = nm;

	_navpoly_link(last_mesh_id);

	return last_mesh_id;
}

void Navigation2D::navpoly_set_transform(int p_id, const Transform2D &p_xform) {

	ERR_FAIL_COND(!navpoly_map.has(p_id));
	NavMesh &nm = navpoly_map[p_id];
	if (nm.xform == p_xform)
		return; //bleh
	_navpoly_unlink(p_id);
	nm.xform = p_xform;
	_navpoly_link(p_id);
}

void Navigation2D::navpoly_remove(int p_id) {

	ERR_FAIL_COND(!navpoly_map.has(p_id));
	_navpoly_unlink(p_id);
	navpoly_map.erase(p_id);
}

Vector<Vector2> Navigation2D::get_simple_path(const Vector2 &p_start, const Vector2 &p_end, bool p_optimize) {

	using namespace std;
	clock_t begin = clock();

	CALLGRIND_START_INSTRUMENTATION;

	if (navpoly_map.size() == 0)
		return Vector<Vector2>();

	Vector2 begin_point = p_start;
	Vector2 end_point = p_end;

	Polygon *begin_poly = get_closest_polygon(begin_point);
	Polygon *end_poly = get_closest_polygon(end_point);

	begin_point = begin_poly->get_closest_point(begin_point);
	end_point = end_poly->get_closest_point(end_point);

	if (begin_poly == end_poly) {

		Vector<Vector2> path;
		path.resize(2);
		path.write[0] = begin_point;
		path.write[1] = end_point;
		return path;
	}

	PoolVector<int> idPath = aStar->get_id_path(begin_poly->id, end_poly->id);
	const int idPathSize = idPath.size();

	if (idPathSize == 0)
		return Vector<Vector2>(); // No path found

	Vector<Vector2> path;

	if (p_optimize) {
		//Simple Stupid Funnel Algorithm

		Polygon *curr_poly = begin_poly;
		Polygon *next_poly;

		Vector<Edge> portals;
		portals.resize(idPathSize + 1);

		portals.write[0] = Edge(begin_point, begin_point);

		for (int i = 1; i < idPathSize; i++) {
			next_poly = &polygons[idPath[i]];
			Edge portal = curr_poly->get_connection_to(next_poly);
			Edge direction = Edge(curr_poly->center, portal.center());
			portal.relative_to(direction);
			portals.write[i] = portal;
			curr_poly = next_poly;
		}

		portals.write[idPathSize] = Edge(end_point, end_point);

		int rightIndex = 0;
		int leftIndex = 0;

		Vector2 apex = begin_point;
		Edge funnel = portals[0];

		path.push_back(begin_point);

		for (int i = 1; i < portals.size(); i++) {

			// Check right side
			Edge rightSide = Edge(apex, portals[i].right);

			if (rightSide.triarea2(funnel.right) <= 0) {

				if (apex == funnel.right || rightSide.triarea2(funnel.left) > 0) { // Inside funnel

					funnel.right = portals[i].right;
					rightIndex = i;
				} else { // Crossing

					if (apex != funnel.left)
						path.push_back(funnel.left);

					apex = funnel.left;
					funnel.right = apex;
					i = leftIndex;
					rightIndex = leftIndex;
					continue;
				}
			}

			// Check left side
			Edge leftSide = Edge(apex, portals[i].left);

			if (leftSide.triarea2(funnel.left) >= 0) {

				if (apex == funnel.left || leftSide.triarea2(funnel.right) < 0) { // Inside funnel

					funnel.left = portals[i].left;
					leftIndex = i;
				} else {

					if (apex != funnel.right)
						path.push_back(funnel.right);

					apex = funnel.right;
					funnel.left = apex;
					i = rightIndex;
					leftIndex = rightIndex;
					continue;
				}
			}
		}

		if (apex != end_point)
			path.push_back(end_point);

	} else {
		//Use the midpoints of the polygons connections

		path.resize(idPathSize + 1);

		path.write[0] = begin_point;

		Polygon *current_p = begin_poly;

		for (int i = 1; i < idPathSize; i++) {
			Polygon *next_p = &polygons[idPath[i]];

			Edge portal = current_p->get_connection_to(next_p);
			path.write[i] = portal.center();

			current_p = next_p;
		}

		path.write[idPathSize] = end_point;
	}

	CALLGRIND_STOP_INSTRUMENTATION;

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

	std::cout << "Time: " << elapsed_secs << '\n';
	std::cout << "Path size: " << path.size() << '\n';

	return path;
}

Navigation2D::Polygon *Navigation2D::get_closest_polygon(const Vector2 &p_point) const {

	for (Map<int, Polygon>::Element *P = polygons.front(); P; P = P->next()) {

		Polygon &p = P->get();

		if (p.contains(p_point))
			return &p;
	}

	Polygon *closest_poly = NULL;
	float min_d = Math_INF;

	for (Map<int, Polygon>::Element *P = polygons.front(); P; P = P->next()) {

		Polygon &p = P->get();
		int points_s = p.points.size();

		for (int i = 0; i < points_s; i++) {

			Edge s(p.points[i], p.points[(i + 1) % points_s]);
			Vector2 closest_point = s.closest_point(p_point);
			float d = closest_point.distance_squared_to(p_point);

			if (d < min_d) {

				closest_poly = &p;
				min_d = d;
			}
		}
	}

	return closest_poly;
}

Vector2 Navigation2D::get_closest_point(const Vector2 &p_point) const {

	Polygon *p = get_closest_polygon(p_point);
	return p->get_closest_point(p_point);
}

Object *Navigation2D::get_closest_point_owner(const Vector2 &p_point) const {

	Polygon *p = get_closest_polygon(p_point);
	NavMesh *nm = p->owner;

	return nm->owner;
}

void Navigation2D::_bind_methods() {

	ClassDB::bind_method(D_METHOD("navpoly_add", "mesh", "xform", "owner"), &Navigation2D::navpoly_add, DEFVAL(Variant()));
	ClassDB::bind_method(D_METHOD("navpoly_set_transform", "id", "xform"), &Navigation2D::navpoly_set_transform);
	ClassDB::bind_method(D_METHOD("navpoly_remove", "id"), &Navigation2D::navpoly_remove);

	ClassDB::bind_method(D_METHOD("get_simple_path", "start", "end", "optimize"), &Navigation2D::get_simple_path, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("get_closest_point", "to_point"), &Navigation2D::get_closest_point);
	ClassDB::bind_method(D_METHOD("get_closest_point_owner", "to_point"), &Navigation2D::get_closest_point_owner);
}

Navigation2D::Navigation2D() {

	last_mesh_id = 0;
	last_polygon_id = 0;
	aStar = memnew(AStar);
}

Navigation2D::~Navigation2D() {

	memdelete(aStar);
}
