
#include <vector>
#include <iostream>

#include "Mouse.h"

#include "Particle.h"
#include "SpringForce.h"
#include "GravityForce.h"
#include "AngularSpring.h"
#include "DragForce.h"
#include "RigidBody.h"

#include "JacobianMatrix.h"

#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "Contact.h"
#include "windows.h"

using namespace std;

enum class AvailableSystems {
	BASIC,
	ROD,
	ROD_COLLISIONS,
	CLOTH,
	HAIR,
	NET,
	BODIES_COLLISIONS,
};

#pragma once
class System
{

public:
	struct State {
		State(vector<Particle*> _particles, vector<RigidBody*> _bodies) {
			particles = _particles;
			bodies = _bodies;
		}
		vector<Particle*> particles;
		vector<RigidBody*> bodies;
	};

	void initialization(AvailableSystems type);

	State get_state();
	void set_state(State state);

	void reset();
	void reset_data();
	void clear_data();
	void free_data();

	void draw(Mouse mouse);
	void draw_particles();
	void draw_bodies();
	void draw_forces(Mouse mouse);
	void draw_constraints();
	void draw_center_axis();
	void draw_contact_points();

	void init_basic();
	void init_bodies_collisions();
	void init_rod();
	void init_rod_collisions();
	void init_cloth();
	void init_hair();
	void init_net();

	std::vector<Particle*>* get_particles();
	std::vector<RigidBody*>* get_bodies();
	std::vector<Force*>* get_forces();
	std::vector<Force*>* get_constraints();

	const double START_DT = 0.06;
	double dt = START_DT;
	bool drawForcesOnParticles = false;

	void show_contact_points(vector<Contact> c);
	void hide_contact_points();

private:
	AvailableSystems system_type = AvailableSystems::BASIC;

	int particle_counter = 0;
	int constraint_counter = 0;

	vector<Contact> contacts;
	bool display_contact_points;

	std::vector<Particle*> particles;
	std::vector<RigidBody*> bodies;
	std::vector<Force*> forces;
	std::vector<Force*> constraints;

	void construct_hair(int internal_particles, Vec3 start, Vec3 end, bool angular);
};

