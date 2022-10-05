#pragma once
#include "include/gfx/vec3.h"
#include "include/gfx/vec2.h"
#include "Contact.h"
#include "System.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <set>

using namespace std;

class CollisionDetector
{
public:
	void find_contacts(System::State& state, vector<Contact>& contacts);

	Vec3 pt_velocity(IObject* obj, Vec3 p);
	bool colliding(Contact c);
	bool collision(Contact c);
private:
	// Broad contact detection
	void find_contacts_broad(set<IObject*>& objs, vector<Contact>& contacts, unordered_map<int, vector<double>> bounds);
	map<double, pair<IObject*, bool>> buildSweepSort(set<IObject*> objs, unordered_map<int, vector<double>> bounds, uint8_t axis);
	void extractObjectsFromCollisions(set<IObject*>& objs, unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> colls);
	unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> sweepLine(map<double, pair<IObject*, bool>> plot);
	bool sat_theorem(Contact c);
	// Narrow contact detection
	void find_contacts_narrow(vector<Contact>& contacts);
	uint8_t check_point_interpenetration_in_aabb(Vec3 pos, Vec3 half_size);
	double get_intersection_plane_point(Vec3 ray_origin, Vec3 ray_dir, Vec3 plane_normal, Vec3 plane_point);
	void check_body_particles_collision(Contact& contact, unordered_map<uint8_t, Vec3>& particles, uint8_t& coll_type, Mat3& Rinv, Vec3& half_size);
	bool check_edge_to_edge_collision(vector<Contact>& edge_contacts, RigidBody* body1, RigidBody* body2, Vec3 half_size, Mat3 Rinv);
	bool check_particle_edge_to_edge_collision(vector<Contact>& edge_contacts, unordered_map<uint8_t, Vec3>& particles, RigidBody* body1, RigidBody* body2, Vec3 half_size, Mat3 Rinv);
	// Misc
	bool within_margin(double value, double target, double tollerance);
};

