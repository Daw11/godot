/*************************************************************************/
/*  navigation.cpp                                                       */
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

#include "navigation.h"

#include <valgrind/callgrind.h>
#include <ctime>
#include <iostream>


Vector3 Navigation::Polygon::get_closest_point(const Vector3 &p_point) const {

	float closest_d = Math_INF;
	Vector3 p = Vector3();

	for (int i = 2; i < points.size(); i++) {

		Face3 f(points[0], points[i - 1], points[i]);
		Vector3 closest_p = f.get_closest_point_to(p_point);
		float d = closest_p.distance_squared_to(p_point);

		if (d < closest_d) {

			p = closest_p;
			closest_d = d;
		}
	}

	return p;
}

// Returns the edge that connects the two polygons
Navigation::Edge Navigation::Polygon::get_connection_to(const Polygon *p_poly) const {

	const int size1 = points.size();
	const int size2 = p_poly->points.size();

	for (int i = 0; i < size1; i++) {

		Navigation::Edge e1 = Edge(points[i], points[(i + 1) % size1]);

		for (int j = 0; j < size2; j++) {

			Navigation::Edge e2 = Edge(p_poly->points[j], p_poly->points[(j + 1) % size2]);
			if (e1 == e2)
				return e1;
		}
	}

	ERR_EXPLAIN("The polygon " + itos(id) + " isn't connected with polygon " + itos(p_poly->id));
	ERR_FAIL_V(Navigation::Edge());
}

void Navigation::_navmesh_link(int p_id) {

	ERR_FAIL_COND(!navmesh_map.has(p_id));
	NavMesh &nm = navmesh_map[p_id];
	ERR_FAIL_COND(nm.linked);
	ERR_FAIL_COND(nm.navmesh.is_null());

	PoolVector<Vector3> vertices = nm.navmesh->get_vertices();
	int len = vertices.size();
	if (len == 0)
		return;

	PoolVector<Vector3>::Read r = vertices.read();

	for (int i = 0; i < nm.navmesh->get_polygon_count(); i++) {

		//build
		last_polygon_id++;
		Map<int, Polygon>::Element *P = polygons.insert(last_polygon_id, Polygon());
		Polygon &p = P->get();
		p.id = last_polygon_id;
		p.owner = &nm;

		Vector<int> poly = nm.navmesh->get_polygon(i);
		const int *indices = poly.ptr();
		int plen = poly.size();

		bool valid = true;
		Vector3 center;

		p.points.resize(plen);

		for (int j = 0; j < plen; j++) {

			int idx = indices[j];
			if (idx < 0 || idx >= len) {
				valid = false;
				break;
			}

			Vector3 point = nm.xform.xform(r[idx]);
			p.points.write[j] = point;
			center += point;
		}

		if (!valid) {
			polygons.erase(p.id);
			ERR_CONTINUE(!valid);
		}

		p.center = center / plen;
		nm.polygon_ids.push_back(p.id);

		aStar->add_point(p.id, p.center);

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

void Navigation::_navmesh_unlink(int p_id) {

	ERR_FAIL_COND(!navmesh_map.has(p_id));
	NavMesh &nm = navmesh_map[p_id];
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

int Navigation::navmesh_add(const Ref<NavigationMesh> &p_mesh, const Transform &p_xform, Object *p_owner) {

	NavMesh nm;
	nm.linked = false;
	nm.navmesh = p_mesh;
	nm.xform = p_xform;
	nm.owner = p_owner;

	last_mesh_id++;

	navmesh_map[last_mesh_id] = nm;

	_navmesh_link(last_mesh_id);

	return last_mesh_id;
}

void Navigation::navmesh_set_transform(int p_id, const Transform &p_xform) {

	ERR_FAIL_COND(!navmesh_map.has(p_id));
	NavMesh &nm = navmesh_map[p_id];
	if (nm.xform == p_xform)
		return; //bleh
	_navmesh_unlink(p_id);
	nm.xform = p_xform;
	_navmesh_link(p_id);
}

void Navigation::navmesh_remove(int p_id) {

	ERR_FAIL_COND(!navmesh_map.has(p_id));
	_navmesh_unlink(p_id);
	navmesh_map.erase(p_id);
}

// Create the path that connects p_from to p_to, the path is a straight line but some of the portals might be at different heights
void Navigation::_clip_path(Vector<Vector3> &r_path, Vector3 &p_from, const Vector3 &p_to, const Vector<Edge> &portals, int &p_from_index, const int p_to_index) {

	if (p_from == p_to) {
		p_from_index = p_to_index;
		return;
	}

	Vector3 plane_normal = ((p_to - p_from).cross(up)).normalized();
	float d = plane_normal.dot(p_from);
	Plane cut_plane = Plane(plane_normal, d); // Vertical plane that cuts the navmesh

	bool first_point = false;

	for (int i = p_from_index + 1; i < p_to_index; i++) {

		Vector3 inters;
		if(!cut_plane.intersects_segment(portals[i].left, portals[i].right, &inters))
			continue;

		if ((p_from * up) == (inters * up)) // The point is at the same height of the previous
			continue;

		if (first_point) {
			r_path.push_back(p_from);
			first_point = true;
		}

		if (inters != p_from)
			r_path.push_back(inters);

		p_from = inters;
	}

	if (p_from != p_to)
		r_path.push_back(p_to);

	p_from_index = p_to_index;
	p_from = p_to;
}

Vector<Vector3> Navigation::get_simple_path(const Vector3 &p_start, const Vector3 &p_end, bool p_optimize) {

	if (navmesh_map.size() == 0)
		return Vector<Vector3>();

	Vector3 begin_point = p_start;
	Vector3 end_point = p_end;

	Polygon *begin_poly = _get_closest_polygon(begin_point);
	Polygon *end_poly = _get_closest_polygon(end_point);

	begin_point = begin_poly->get_closest_point(begin_point);
	end_point = end_poly->get_closest_point(end_point);

	if (begin_poly == end_poly) {

		Vector<Vector3> path;
		path.resize(2);
		path.write[0] = begin_point;
		path.write[1] = end_point;
		return path;
	}

	PoolVector<int> idPath = aStar->get_id_path(begin_poly->id, end_poly->id);
	const int idPathSize = idPath.size();

	if (idPathSize == 0)
		return Vector<Vector3>(); // No path found

	Vector<Vector3> path;

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
			portal.relative_to(direction, up);
			portals.write[i] = portal;
			curr_poly = next_poly;
		}

		portals.write[idPathSize] = Edge(end_point, end_point);

		int rightIndex = 0;
		int leftIndex = 0;
		int apexIndex = 0;

		Vector3 apex = begin_point;
		Edge funnel = portals[0];

		path.push_back(begin_point);

		for (int i = 1; i < portals.size(); i++) {

			// Check right side
			Edge rightSide = Edge(apex, portals[i].right);

			if (rightSide.triarea2(funnel.right, up) <= 0) {

				if (apex == funnel.right || rightSide.triarea2(funnel.left, up) > 0) { // Inside funnel

					funnel.right = portals[i].right;
					rightIndex = i;
				} else { // Crossing

					_clip_path(path, apex, funnel.left, portals, apexIndex, leftIndex);

					// if (apex != funnel.left)
					// 	path.push_back(funnel.left);
					//
					// apex = funnel.left;
					funnel.right = apex;
					i = rightIndex = leftIndex;
					continue;
				}
			}

			// Check left side
			Edge leftSide = Edge(apex, portals[i].left);

			if (leftSide.triarea2(funnel.left, up) >= 0) {

				if (apex == funnel.left || leftSide.triarea2(funnel.right, up) < 0) { // Inside funnel

					funnel.left = portals[i].left;
					leftIndex = i;
				} else {

					_clip_path(path, apex, funnel.right, portals, apexIndex, rightIndex);

					// if (apex != funnel.right)
					// 	path.push_back(funnel.right);
					//
					// apex = funnel.right;
					funnel.left = apex;
					i = leftIndex = rightIndex;
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

	return path;
}

Vector3 Navigation::get_closest_point_to_segment(const Vector3 &p_from, const Vector3 &p_to, const bool &p_use_collision) {

	Vector3 closest_point;
	float closest_d = Math_INF;

	for (Map<int, Polygon>::Element *P = polygons.front(); P; P = P->next()) {

		Polygon &p = P->get();
		int points_s = p.points.size();

		for (int i = 2; i < points_s; i++) {

			Face3 f(p.points[0], p.points[i - 1], p.points[i]);
			Vector3 inters;
			if (f.intersects_segment(p_from, p_to, &inters)) {

				float d = p_from.distance_squared_to(inters);
				if(d < closest_d) {

					closest_point = inters;
					closest_d = d;
				}
			}
		}
	}

	if (p_use_collision || closest_d != Math_INF)
		return closest_point;

	for (Map<int, Polygon>::Element *P = polygons.front(); P; P = P->next()) {

		Polygon &p = P->get();
		int points_s = p.points.size();

		for (int i = 0; i < points_s; i++) {

			Vector3 a, b;
			Geometry::get_closest_points_between_segments(p_from, p_to, p.points[i], p.points[(i + 1) % points_s], a, b);

			float d = a.distance_squared_to(b);
			if (d < closest_d) {

				closest_point = b;
				closest_d = d;
			}
		}
	}

	return closest_point;
}

Navigation::Polygon *Navigation::_get_closest_polygon(const Vector3 &p_point) const {

	Polygon *closest_poly = NULL;
	float closest_d = Math_INF;

	for (Map<int, Polygon>::Element *P = polygons.front(); P; P = P->next()) {

		Polygon &p = P->get();

		for (int i = 2; i < p.points.size(); i++) {

			Face3 f(p.points[0], p.points[i - 1], p.points[i]);
			Vector3 closest_p = f.get_closest_point_to(p_point);
			float d = closest_p.distance_squared_to(p_point);

			if (d < closest_d) {

				closest_poly = &p;
				closest_d = d;
			}
		}
	}

	return closest_poly;
}

Vector3 Navigation::get_closest_point(const Vector3 &p_point) {

	Polygon *p = _get_closest_polygon(p_point);
	return p->get_closest_point(p_point);
}

Vector3 Navigation::get_closest_point_normal(const Vector3 &p_point) {

	Polygon *p = _get_closest_polygon(p_point);
	Face3 f(p->points[0], p->points[1], p->points[2]);
	return f.get_plane().normal;
}

Object *Navigation::get_closest_point_owner(const Vector3 &p_point) {

	Polygon *p = _get_closest_polygon(p_point);
	NavMesh *nm = p->owner;

	return nm->owner;
}

void Navigation::set_up_vector(const Vector3 &p_up) {

	up = p_up;
}

Vector3 Navigation::get_up_vector() const {

	return up;
}

void Navigation::_bind_methods() {

	ClassDB::bind_method(D_METHOD("navmesh_add", "mesh", "xform", "owner"), &Navigation::navmesh_add, DEFVAL(Variant()));
	ClassDB::bind_method(D_METHOD("navmesh_set_transform", "id", "xform"), &Navigation::navmesh_set_transform);
	ClassDB::bind_method(D_METHOD("navmesh_remove", "id"), &Navigation::navmesh_remove);

	ClassDB::bind_method(D_METHOD("get_simple_path", "start", "end", "optimize"), &Navigation::get_simple_path, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("get_closest_point_to_segment", "start", "end", "use_collision"), &Navigation::get_closest_point_to_segment, DEFVAL(false));
	ClassDB::bind_method(D_METHOD("get_closest_point", "to_point"), &Navigation::get_closest_point);
	ClassDB::bind_method(D_METHOD("get_closest_point_normal", "to_point"), &Navigation::get_closest_point_normal);
	ClassDB::bind_method(D_METHOD("get_closest_point_owner", "to_point"), &Navigation::get_closest_point_owner);

	ClassDB::bind_method(D_METHOD("set_up_vector", "up"), &Navigation::set_up_vector);
	ClassDB::bind_method(D_METHOD("get_up_vector"), &Navigation::get_up_vector);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "up_vector"), "set_up_vector", "get_up_vector");
}

Navigation::Navigation() {

	up = Vector3(0, 1, 0);
	last_mesh_id = 0;
	last_polygon_id = 0;
	aStar = memnew(AStar);
}

Navigation::~Navigation() {

	memdelete(aStar);
}
