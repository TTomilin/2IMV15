#include "stdafx.h"
#include "collisionDetector.h"
#include <cstdlib>
#include <ctime>

//#define DEBUG
#define TOLLERANCE	0.001
#define THRESHOLD	0.01
#define EPSILON		0.5
#define MULTIPLIER  1.25

void CollisionDetector::find_contacts(System::State& state, vector<Contact>& contacts)
{
	// Push bodies onto array
	set<IObject*> objects(state.bodies.begin(), state.bodies.end());
	// Push all particles not belonging to a body onto array
	int last_particle = state.bodies.size() == 0 ? state.particles.size() : state.bodies.at(0)->getID();
	for (int i = 0; i < last_particle; i++) objects.insert(state.particles.at(i));
	// Cache AABB bounds as they require computation
	unordered_map<int, vector<double>> bounds;
	for (IObject* obj : objects) bounds[obj->getID()] = obj->getBoundingBox();

	// Broad collision detection
	find_contacts_broad(objects, contacts, bounds);
#ifdef DEBUG
	if (contacts.size() > 0) {
		for (int i = 0; i < contacts.size(); i++) {
			printf("Broad contact=(%d, %d)\n", contacts[i].a->getID(), contacts[i].b->getID());
		}
	}
	else {
		printf("No broad contacts\n");
	}
#endif // DEBUG

	// Narrow collision detection
	find_contacts_narrow(contacts);
#ifdef DEBUG
	if (contacts.size() > 0) {
		for (int i = 0; i < contacts.size(); i++) {
			printf("Narrow contact=(%d, %d)\n", contacts[i].a->getID(), contacts[i].b->getID());
		}
	}
	else {
		printf("No narrow contacts\n");
	}
	printf("\n");
#endif // DEBUG
}

Vec3 CollisionDetector::pt_velocity(IObject* obj, Vec3 p)
{
	if (obj->getType() == IObject::Type::particle) {
		return (dynamic_cast<Particle*>(obj))->m_Velocity;
	}
	else {
		RigidBody* body = dynamic_cast<RigidBody*>(obj);
		return body->v + (body->omega ^ (p - body->x));
	}
}

bool CollisionDetector::colliding(Contact c)
{
	Vec3 padot = pt_velocity(c.a, c.p),	//p_a(t0)
		pbdot = pt_velocity(c.b, c.p);	//p_b(t0)
	double	vrel = c.n * (padot - pbdot);

	return vrel < THRESHOLD;
}

bool CollisionDetector::collision(Contact c)
{
	Vec3 padot = pt_velocity(c.a, c.p),	//p_a(t0)
		pbdot = pt_velocity(c.b, c.p),	//p_b(t0)
		ra = c.p - c.a->getPosition(),	//r_a
		rb = c.p - c.b->getPosition();	//r_b
	double	vrel = c.n * (padot - pbdot),	// v_(rel)
		numerator = -(1 + EPSILON) * vrel;

	// Calculate denominator in four parts
	double	term1 = c.a->getType() == IObject::Type::rigidBody ? 1 / c.a->getMassSum() : c.a->getMassSum(),
		    term2 = c.b->getType() == IObject::Type::rigidBody ? 1 / c.b->getMassSum() : c.b->getMassSum(),
		    term3 = c.n * ((c.a->getIinv() * (ra ^ c.n)) ^ ra),
		    term4 = c.n * ((c.b->getIinv() * (rb ^ c.n)) ^ rb);

	// Compute impulse magnitude
	double j = numerator / (term1 + term2 + term3 + term4);
	Vec3 force = j * c.n * MULTIPLIER;

	// Apply the impulse
	c.a->addLinearMomentum(force);
	if (c.a->getType() == IObject::Type::rigidBody) {
		c.a->setVelocity(c.a->getLinearMomentum() / c.a->getMassSum());
		c.a->addAngularMomentum(ra ^ force);
		c.a->setOmega(c.a->getIinv() * c.a->getAngularMomentum());
	}
	else {
		c.a->setVelocity(c.a->getVelocity() + force * c.a->getMassSum());
	}

	c.b->addLinearMomentum(-force);
	if (c.b->getType() == IObject::Type::rigidBody) {
		c.b->setVelocity(c.b->getLinearMomentum() / c.b->getMassSum());
		c.b->addAngularMomentum((rb ^ -force));
		c.b->setOmega(c.b->getIinv() * c.b->getAngularMomentum());
	}
	else {
		c.b->setVelocity(c.b->getVelocity() - force * c.b->getMassSum());
	}

	return false;
}

///////////////////////////////////////////////
////// Broad contact detection functions //////
///////////////////////////////////////////////

void CollisionDetector::find_contacts_broad(set<IObject*>& objs, vector<Contact>& contacts, unordered_map<int, vector<double>> bounds)
{
	// Sweep sort
	map<double, pair<IObject*, bool>> m = buildSweepSort(objs, bounds, 0);
	// Find Particles/Bodies potentially colliding
	unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> colls = sweepLine(m);
	for (int axis = 1; axis < 3; axis++) {
		// Build map only with objects that could intersect
		extractObjectsFromCollisions(objs, colls);
		m = buildSweepSort(objs, bounds, axis);
		// Find collisions on axis
		unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> curColls = sweepLine(m);
		// Remove objects from overall collisions list that do not collide on current axis
		vector<pair<int, int>> toRemove;
		for (pair<int, pair<IObject*, unordered_map<int, IObject*>>> c1 : colls) {
			for (pair<int, IObject*> c2 : c1.second.second) {
				if (curColls.find(c1.first) != curColls.end()) {
					if (curColls[c1.first].second.find(c2.first) == curColls[c1.first].second.end()) {
						toRemove.push_back(make_pair(c1.first, c2.first));
					}
				}
				else {
					toRemove.push_back(make_pair(c1.first, -1));
				}
			}
		}
		for (pair<int, int> entry : toRemove) {
			if (entry.second == -1) colls.erase(entry.first);
			else colls[entry.first].second.erase(entry.second);
		}
	}
	// Build contacts list
	for (pair<int, pair<IObject*, unordered_map<int, IObject*>>> c1 : colls) {
		for (pair<int, IObject*> c2 : c1.second.second) {
			Contact contact;
			contact.a = c1.second.first; contact.b = c2.second;
			// Perform SAT check on body to body collisions
			if (contact.a->getType() == IObject::Type::rigidBody && contact.b->getType() == IObject::Type::rigidBody) {
				if (!sat_theorem(contact)) continue;
			}
			contact.vf = false;
			contacts.push_back(contact);
		}
	}
}

map<double, pair<IObject*, bool>> CollisionDetector::buildSweepSort(set<IObject*> objs, unordered_map<int, vector<double>> bounds, uint8_t axis)
{
	map<double, pair<IObject*, bool>> m;
	for (IObject* obj : objs) {
		srand(obj->getID() + time(0));
		double r1 = ((double)rand() / (double)(RAND_MAX)) / 100000.0 - 0.000005;
		double r2 = ((double)rand() / (double)(RAND_MAX)) / 100000.0 - 0.000005;
		m[bounds[obj->getID()][axis * 2] + r1 - TOLLERANCE] = make_pair(obj, true);		// Start
		m[bounds[obj->getID()][axis * 2 + 1] + r2 + TOLLERANCE] = make_pair(obj, false);	// End
	}
	return m;
}

void CollisionDetector::extractObjectsFromCollisions(set<IObject*>& objs, unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> colls)
{
	objs.clear();
	for (pair<int, pair<IObject*, unordered_map<int, IObject*>>> c1 : colls) {
		objs.insert(c1.second.first);
		for (pair<int, IObject*> c2 : c1.second.second) {
			objs.insert(c2.second);
		}
	}
}

unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> CollisionDetector::sweepLine(map<double, pair<IObject*, bool>> plot)
{
	unordered_map<int, pair<IObject*, unordered_map<int, IObject*>>> colls;
	unordered_map<int, IObject*> active;
	for (pair<double, pair<IObject*, bool>> x : plot) {
		if (x.second.second) {
			// Register collisions
			if (active.size() > 0) {
				for (pair<int, IObject*> obj : active) {
					IObject* id1; IObject* id2;
					// Ensure id1 > id2 at all times
					if (x.second.first->getID() > obj.first) {
						id1 = obj.second;
						id2 = x.second.first;
					}
					else {
						id1 = x.second.first;
						id2 = obj.second;
					}
					// Populate collisions list
					if (colls.count(id1->getID()) == 0) colls[id1->getID()] = make_pair(id1, unordered_map<int, IObject*>());
					colls[id1->getID()].second[id2->getID()] = id2;
				}
			}
			// Add to active list
			active[x.second.first->getID()] = x.second.first;
		}
		else {
			active.erase(x.second.first->getID());
		}
	}
	return colls;
}

bool CollisionDetector::sat_theorem(Contact c)
{
	RigidBody* body1;
	RigidBody* body2;
	bool intersect = true;
	for (uint8_t s = 0; s < 2; s++) {
		body1 = dynamic_cast<RigidBody*>(s == 0 ? c.a : c.b);
		body2 = dynamic_cast<RigidBody*>(s == 0 ? c.b : c.a);

		Mat3 Rinv = body1->R.inverse();
		Vec3 half_size = body1->size / 2.0;
		vector<Vec2> bounds2{ Vec2(DBL_MAX,-DBL_MAX),Vec2(DBL_MAX,-DBL_MAX),Vec2(DBL_MAX,-DBL_MAX) };

		// Compute body2 bounding box axis alligned with body1
		for (uint8_t i = 0; i < 8; i++) {
			Vec3 pos = Rinv * (body2->get_particles().at(i)->getPosition() - body1->getPosition());
			// Keep track of bounding box
			for (uint8_t axis = 0; axis < 3; axis++) {
				if (pos[axis] < bounds2[axis][0]) bounds2[axis][0] = pos[axis];
				else if (pos[axis] > bounds2[axis][1]) bounds2[axis][1] = pos[axis];
			}
		}
		// Check for intersection
		for (uint8_t axis = 0; axis < 3; axis++) {
			if (!(
				bounds2[axis][1] >= -half_size[axis] && bounds2[axis][0] <= half_size[axis]
				)) {
				intersect = false;
			}
		}
		if (!intersect) {
			break;
		}
	}
	return intersect;
}

///////////////////////////////////////////////
///// Narrow contact detection functions //////
///////////////////////////////////////////////

void CollisionDetector::find_contacts_narrow(vector<Contact>& contacts)
{
	vector<Contact> narrow_contacts;
	for (Contact contact : contacts) {
		if (contact.a->getType() == IObject::particle && contact.b->getType() == IObject::particle) {
			// Particle to particle
			Particle* sphere1 = dynamic_cast<Particle*>(contact.a);
			Particle* sphere2 = dynamic_cast<Particle*>(contact.b);

			Vec3 d = (contact.a->getPosition() - contact.b->getPosition());
			double d_magnitude = d.magnitude();
			double dist = sphere1->radius + sphere2->radius;
			if (d_magnitude <= dist + TOLLERANCE) {
				if (within_margin(d_magnitude, dist, TOLLERANCE)) {
					// Contact point
					contact.p = contact.b->getPosition() + (d / 2.0);
					// Face normal
					contact.n = d / d_magnitude;
					// Contact vertex to face
					contact.vf = true;
				}
				else {
					contact.vf = false;
				}
				// Add to list of valid contacts
				narrow_contacts.push_back(contact);
			}
		}
		else if (contact.a->getType() == IObject::particle && contact.b->getType() == IObject::rigidBody) {
			// Particle to body (body to particle cannot happen due to id ordering of contacts list)
			Particle* sphere = dynamic_cast<Particle*>(contact.a);
			RigidBody* body = dynamic_cast<RigidBody*>(contact.b);

			Vec3 half_size = body->size / 2.0;
			// Get point on sphere closest to the body
			Vec3 p_relativeTo_b = body->R.inverse() * (contact.a->getPosition() - contact.b->getPosition());
			Vec3 p_onSphere = Vec3(0.0, 0.0, 0.0);
			for (uint8_t axis = 0; axis < 3; axis++) {
				if (p_relativeTo_b[axis] > 0) p_onSphere[axis] = fmin(p_relativeTo_b[axis], half_size[axis]);
				else p_onSphere[axis] = fmax(p_relativeTo_b[axis], -half_size[axis]);
			}
			p_onSphere = p_onSphere - p_relativeTo_b;
			p_relativeTo_b = p_relativeTo_b + p_onSphere / p_onSphere.magnitude() * sphere->radius;
			// Check for collision
			uint8_t coll_type = check_point_interpenetration_in_aabb(p_relativeTo_b, half_size);
			if (coll_type == 1) {
				contact.p = (body->R * p_relativeTo_b) + body->getPosition();
				contact.n = (body->R * (-p_onSphere / p_onSphere.magnitude()));
				contact.vf = true;
				narrow_contacts.push_back(contact);
			}
			else if (coll_type == 2) {
				contact.vf = false;
				narrow_contacts.push_back(contact);
			}
		}
		else if (contact.a->getType() == IObject::rigidBody && contact.b->getType() == IObject::rigidBody) {
			// Body to body
			// Check if any corner is against the other body
			Mat3 Rinv;
			Vec3 half_size;
			unordered_map<uint8_t, Vec3> particles;
			uint8_t coll_type;
			check_body_particles_collision(contact, particles, coll_type, Rinv, half_size);
			//// STEP 2
			if (coll_type < 2) {
				RigidBody* body1 = dynamic_cast<RigidBody*>(contact.a);
				RigidBody* body2 = dynamic_cast<RigidBody*>(contact.b);

				vector<Contact> edge_contacts;
				if (particles.size() == 0) {
					coll_type = check_edge_to_edge_collision(edge_contacts, body1, body2, half_size, Rinv) ? 2 : coll_type;
				}
				else {
					coll_type = check_particle_edge_to_edge_collision(edge_contacts, particles, body2, body1, half_size, Rinv) ? 2 : coll_type;
				}
				// Add contacts if no inter penetration
				if (coll_type != 2) {
					// Add valid 
					for (pair<uint8_t, Vec3> a : particles) {
						// Add contact
						Contact c;
						c.a = contact.a;
						c.b = contact.b;
						c.p = body1->get_particles().at(a.first)->getPosition();
						c.n = body2->R * a.second;
						c.vf = true;
						narrow_contacts.push_back(c);
					}
					for (Contact edge_contact : edge_contacts) narrow_contacts.push_back(edge_contact);
				}
			}
			// Inter penetration
			if (coll_type == 2) {
				contact.vf = false;
				narrow_contacts.push_back(contact);
				continue;
			}
		}
	}
	contacts.swap(narrow_contacts);
}

uint8_t CollisionDetector::check_point_interpenetration_in_aabb(Vec3 pos, Vec3 half_size)
{
	bool surface_contact = false;
	uint8_t inter_penetration = 0;
	for (uint8_t axis = 0; axis < 3; axis++) {
		double abs_pos = abs(pos[axis]);
		if (within_margin(abs_pos, half_size[axis], TOLLERANCE)) {
			surface_contact = true;
		}
		else if (abs_pos < half_size[axis] - TOLLERANCE) {
			inter_penetration += 1;
		}
		else {
			surface_contact = false;
		}
	}
	// Surface contact
	if (surface_contact && inter_penetration == 2) return 1;
	// Inter penetration
	if (inter_penetration == 3) return 2;
	// No contact
	return 0;
}

double CollisionDetector::get_intersection_plane_point(Vec3 ray_origin, Vec3 ray_dir, Vec3 plane_normal, Vec3 plane_point)
{
	Vec3 diff = ray_origin - plane_point;
	return dotProduct(diff, plane_normal) / dotProduct(ray_dir, plane_normal);
}

void CollisionDetector::check_body_particles_collision(
	Contact& contact, unordered_map<uint8_t, Vec3>& particles, uint8_t& coll_type,
	Mat3& Rinv, Vec3& half_size
) {
	for (uint8_t s = 0; s < 2; s++) {
		RigidBody* body1 = dynamic_cast<RigidBody*>(contact.b);
		RigidBody* body2 = dynamic_cast<RigidBody*>(contact.a);
		Rinv = body1->R.inverse();
		half_size = body1->size / 2.0;
		// Check particles
		for (uint8_t i = 0; i < 8; i++) {
			Vec3 pos = Rinv * (body2->get_particles().at(i)->getPosition() - body1->getPosition());
			// Check for body particle contact
			coll_type = check_point_interpenetration_in_aabb(pos, half_size);

			if (coll_type == 1) {
				// Face Normal
				double gap = DBL_MAX;
				double closestAxis = -1;
				for (uint8_t axis = 0; axis < 3; axis++) {
					double d = fabs(fabs(pos[axis]) - half_size[axis]);
					if (d < gap) {
						closestAxis = axis;
						gap = d;
					}
				}
				Vec3 normal = Vec3(0, 0, 0);
				normal[closestAxis] = pos[closestAxis] > 0 ? 1 : -1;
				particles.insert(make_pair(i, normal));
			}
			else if (coll_type == 2) {
				break;
			}
		}
		if (particles.size() == 0 && coll_type != 2) {
			// Swap contacts
			IObject* swap = contact.a;
			contact.a = contact.b;
			contact.b = swap;
		}
		else {
			break;
		}
	}
}

bool CollisionDetector::check_edge_to_edge_collision(vector<Contact>& edge_contacts, RigidBody* body1, RigidBody* body2, Vec3 half_size, Mat3 Rinv)
{
	// Get possible collision planes
	vector<Vec3> plane_normals;
	Vec3 b_relativeTo_a = Rinv * (body2->getPosition() - body1->getPosition());
	for (uint8_t axis = 0; axis < 3; axis++) {
		Vec3 n = Vec3(0.0, 0.0, 0.0);
		n[axis] = b_relativeTo_a[axis] >= 0.0 ? 1.0 : 1.0;
		plane_normals.push_back(n);
	}
	// Check each plane
	vector<pair<Vec3, Vec3>> edges = body2->get_edges();
	uint8_t inter_penetration = 0;
	for (Vec3 plane_normal : plane_normals) {
		Vec3 plane_point = plane_normal * half_size;
		// Check each edge
		for (pair<Vec3, Vec3> edge : edges) {
			Vec3 ray_origin = (Rinv * (edge.first - body1->getPosition()));
			Vec3 ray_dir = Rinv * (edge.second - edge.first);
			Vec3 ray_end = ray_origin + ray_dir;

			double t = get_intersection_plane_point(ray_origin, ray_dir, plane_normal, plane_point);
			if (t >= -1.0 && t <= 0.0) {
				Vec3 contact_point = ray_origin - ray_dir * t;
				// Check contact validity
				uint8_t inter_penetration = 0;
				bool is_edge_contact = false;
				for (uint8_t axis = 0; axis < 3; axis++) {
					if (plane_normal[axis] == 0) {
						double penetration = contact_point[axis] < 0 ?
							contact_point[axis] + half_size[axis] :
							half_size[axis] - contact_point[axis];
						if (penetration >= -TOLLERANCE && penetration <= TOLLERANCE) {
							is_edge_contact = true;
						}
						else if (penetration > TOLLERANCE) {
							inter_penetration++;
						}
					}
				}
				if (is_edge_contact && inter_penetration == 1) {
					Contact c;
					c.a = body1;
					c.b = body2;
					c.p = (body1->R * contact_point) + body1->getPosition();
					c.n = body1->R * plane_normal;
					c.vf = true;
					edge_contacts.push_back(c);
				}
				else if (inter_penetration == 2) {
					return true;
				}
			}
		}
	}
	return false;
}

bool CollisionDetector::check_particle_edge_to_edge_collision(vector<Contact>& edge_contacts, unordered_map<uint8_t, Vec3>& particles, RigidBody* body1, RigidBody* body2, Vec3 half_size, Mat3 Rinv)
{
	// Check that all particles are on the same face and that their edges do not go towards the body
	Vec3 face_normal = Vec3(DBL_MAX, DBL_MAX, DBL_MAX);
	for (pair<uint8_t, Vec3> a : particles) {
		if (face_normal[0] == DBL_MAX) {
			face_normal = a.second;
		}
		else {
			for (uint8_t axis = 0; axis < 3; axis++) {
				if (face_normal[axis] != a.second[axis]) {
					return true;
				}
			}
		}

		// Check that the edges that include this particle do not go towards the body unless it's another
		// surface contact particle
		vector<uint8_t> near_particles = body2->get_edges_of_particle(a.first);
		for (uint8_t b : near_particles) {
			if (particles.count(b) == 0) {
				Vec3 ray_origin = (Rinv * (body2->get_particles().at(a.first)->getPosition() - body1->getPosition()));
				Vec3 ray_dir = Rinv * (body2->get_particles().at(b)->getPosition() - body2->get_particles().at(a.first)->getPosition());
				Vec3 ray_end = ray_origin + ray_dir;
				Vec3 plane_point = face_normal * half_size;

				double edge_contact_t = -DBL_MAX;
				uint8_t fixed_axis = face_normal[0] != 0 ? 0 : (face_normal[1] != 0 ? 1 : 2);
				for (uint8_t axis = 0; axis < 3; axis++) {
					// Check collision against the planes of the of the other faces to determine if the edge
					// touches the edge of the face and if it does if it's a surface contact or inter penetration
					if (face_normal[axis] == 0) {
						// Plane normal
						Vec3 plane_normal = Vec3(0.0, 0.0, 0.0);
						plane_normal[axis] = ray_end[axis] > 0 ? -1 : 1;
						// Plane Point
						Vec3 plane_p = -half_size * plane_normal;
						// Check for intersection
						double t_2 = get_intersection_plane_point(ray_origin, ray_dir, plane_normal, plane_p);
						// Ensure there's no inter_penetration
						if (t_2 >= -1.0 && t_2 <= 0.0) {
							Vec3 new_pos = ray_origin + ray_dir * t_2;
							double penetration = plane_point[fixed_axis] > 0 ?
								new_pos[fixed_axis] - plane_point[fixed_axis] :
								plane_point[fixed_axis] - new_pos[fixed_axis];
							// Check penetration
							if (penetration < -TOLLERANCE)
								edge_contact_t = DBL_MAX;	// No contact
							else if (penetration > TOLLERANCE)
								return true;				// Inter penetration
							else
								edge_contact_t = t_2;		// Surface contact
							break;
						}
					}
				}
				if (edge_contact_t >= -1.0 && edge_contact_t < 0.0) {
					// Add valid contact
					Contact c;
					c.a = body1;
					c.b = body2;
					ray_end = ray_origin - ray_dir * edge_contact_t;
					c.p = (body1->R * ray_end) + body1->getPosition();
					c.n = body1->R * a.second;
					c.vf = true;
					edge_contacts.push_back(c);
				}
			}
		}
	}
	return false;
}

bool CollisionDetector::within_margin(double value, double target, double tollerance)
{
	if (value >= target - tollerance && value <= target + tollerance) return true;
	return false;
}
