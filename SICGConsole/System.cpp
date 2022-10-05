#include "stdafx.h"
#include "System.h"
#include "gl\GLUT.h"
#include "RigidBody.h"

void System::initialization(AvailableSystems type)
{
	this->system_type = type;

	switch (type) {
	case AvailableSystems::BASIC:
		this->init_basic();
		break;

	case AvailableSystems::ROD:
		this->init_rod();
		break;

	case AvailableSystems::ROD_COLLISIONS:
		this->init_rod_collisions();
		break;

	case AvailableSystems::CLOTH:
		this->init_cloth();
		break;

	case AvailableSystems::HAIR:
		this->init_hair();
		break;

	case AvailableSystems::NET:
		this->init_net();
		break;

	case AvailableSystems::BODIES_COLLISIONS:
		this->init_bodies_collisions();
		break;

	default:
		this->initialization(AvailableSystems::BASIC);
	}
}


// CONTROL SYSTEM STATES
System::State System::get_state()
{
	vector<Particle*> particlesCopy;
	vector<RigidBody*> bodiesCopy;

	// Loop through particles
	for (int i = 0; i < particle_counter; i++) {
		Particle* particle = this->get_particles()->at(i);
		Particle* newParticle = new Particle(*particle);
		particlesCopy.push_back(newParticle);
	}
	// Loop through bodies
	for (int i = 0; i < bodies.size(); i++) {
		RigidBody* body = this->get_bodies()->at(i);
		vector<Particle*> bodyParticles = body->get_particles();
		// Create deep copy of body
		RigidBody* newBody = new RigidBody(*body);
		newBody->particles.clear();
		for (int p = 0; p < bodyParticles.size(); p++) {
			Particle* newParticle = new Particle(*bodyParticles.at(p));
			newBody->particles.push_back(newParticle);
			particlesCopy.push_back(newParticle);
		}
		bodiesCopy.push_back(newBody);
	}

	return State(particlesCopy, bodiesCopy);
}

void System::set_state(State state)
{
	assert(state.particles.size() == particles.size());
	assert(state.bodies.size() == bodies.size());
	// Update Particles
	for (int i = 0; i < state.particles.size(); i++) {
		Particle* newParticleState = state.particles.at(i);
		Particle* particle = particles.at(i);
		particle->m_Position = newParticleState->m_Position;
		particle->m_Velocity = newParticleState->m_Velocity;
	}
	// Update Bodies
	for (int i = 0; i < state.bodies.size(); i++) {
		RigidBody* newBodyState = state.bodies.at(i);
		RigidBody* body = bodies.at(i);
		body->setState(RigidBody::State(newBodyState->x, newBodyState->q, newBodyState->P, newBodyState->L));
	}
}


// CLEAN SYSTEM
void System::reset()
{
	this->initialization(this->system_type);
}

void System::free_data()
{
	particles.clear();
	bodies.clear();

	for (Force* force : forces) {
		delete force;
		force = NULL;
	}

	for (Force* constraint : constraints) {
		delete constraint;
		constraint = NULL;
	}
}

void System::reset_data()
{
	for (int i = 0; i < particles.size(); i++) {
		particles[i]->reset();
	}

	for (int i = 0; i < bodies.size(); i++) {
		bodies[i]->reset();
	}

	dt = START_DT;
}

void System::clear_data()
{
	printf("\nClearing data");
	particles.clear();
	bodies.clear();
	forces.clear();
	constraints.clear();
}

// DRAW SYSTEM
void System::draw(Mouse mouse)
{
	this->draw_particles();
	this->draw_bodies();
	this->draw_forces(mouse);
	this->draw_constraints();
	this->draw_center_axis();
	if (display_contact_points) {
		this->draw_contact_points();
	}
}

void System::draw_particles()
{
	for (int ii = 0; ii < particle_counter; ii++)
	{
		particles[ii]->draw(drawForcesOnParticles);
	}
}

void System::draw_bodies()
{
	int size = bodies.size();

	for (int ii = 0; ii < size; ii++)
	{
		bodies[ii]->draw();
	}
}

void System::draw_forces(Mouse mouse)
{
	for (Force* force : forces) {
		force->draw();
	}

	// Mouse spring
	if (mouse.getMouseSpring() != nullptr) {
		mouse.getMouseSpring()->draw();
	}
}

void System::draw_constraints()
{
	for (Force* constraint : constraints) {
		constraint->draw();
	}
}

void System::draw_center_axis()
{
	glBegin(GL_LINES);
	glColor3f(1., 0., 0.);
	glVertex3f(0., 0., 0.);
	glColor3f(1., 0., 0.);
	glVertex3f(1., 0., 0.);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0., 1., 0.);
	glVertex3f(0., 0., 0.);
	glColor3f(0., 1., 0.);
	glVertex3f(0., 1., 0.);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0., 0., 1.);
	glVertex3f(0., 0., 0.);
	glColor3f(0., 0., 1.);
	glVertex3f(0., 0., 1.);
	glEnd();
}

void System::draw_contact_points()
{
	for (Contact contact : contacts) {
		if (contact.vf) {
			glPushMatrix();
			glColor3f(0.5, 0.2, 0.0);
			GLUquadric* quad = gluNewQuadric();
			glTranslatef(contact.p[0], contact.p[1], contact.p[2]);
			gluSphere(quad, 0.005, 50, 50);
			glPopMatrix();

			Vec3 a = contact.p;
			Vec3 b = contact.p + contact.n;
			glBegin(GL_LINES);
			glColor3f(1., 0., 0.);
			glVertex3f(a[0], a[1], a[2]);
			glColor3f(1., 0., 0.);
			glVertex3f(b[0], b[1], b[2]);
			glEnd();
		}
	}
}


// SYSTEM INITALIZATIONS
void System::init_basic()
{

	printf("\n\nInitializing BASIC particle system... ");
	/*
		Generate the circle
	*/

	// CONSTANTS
	double dist = 0.2;
	Vec3 center(-1.5, 0.5, 0.0);
	Vec3 offset_x(dist, 0.0, 0.0);
	Vec3 offset_y(0.0, dist, 0.0);

	// INIT PARTICLES
	double mass = 1.;
	particle_counter = 0;
	particles.push_back(new Particle(center + offset_x, particle_counter++, mass));
	particles.push_back(new Particle(center + 2.0 * offset_x, particle_counter++, mass));
	particles.push_back(new Particle(center + 3.0 * offset_x, particle_counter++, mass));
	particles.push_back(new Particle(center + 4.0 * offset_x, particle_counter++, mass));

	particles.push_back(new Particle(center + 2.5 * offset_x - offset_y, particle_counter++, mass));
	particles.push_back(new Particle(center + 2.5 * offset_x - 2.0 * offset_y, particle_counter++, mass));

	// INIT FORCES
	forces.push_back(new GravityForce());
	forces.push_back(new DragForce());
	forces.push_back(new SpringForce(particles[0], particles[1], dist, 0.1, 0.01));
	forces.push_back(new SpringForce(particles[1], particles[2], dist, 0.1, 0.01));
	forces.push_back(new SpringForce(particles[1], particles[4], dist, 0.1, 0.01));
	forces.push_back(new SpringForce(particles[2], particles[3], dist, 0.1, 0.01));
	forces.push_back(new SpringForce(particles[2], particles[4], dist, 0.1, 0.01));
	forces.push_back(new SpringForce(particles[4], particles[5], dist, 0.1, 0.01));

	// INIT CONSTRAINTS
	constraints.push_back(new CircularWireConstraint(particles[0], center, dist, constraint_counter++));

	// Two lone particles elsewhere that just fall down
	particles.push_back(new Particle(Vec3(1., .5, 0.), particle_counter++, mass));
	particles.push_back(new Particle(Vec3(1.2, .5, 0.), particle_counter++, mass));

	printf("DONE");

	//for (Particle* particle : particles) {
	//	Particle::print_data(*particle);
	//}
}

void System::init_bodies_collisions()
{
	particle_counter = 0;
	// INIT PARTICLES
	particles.push_back(new Particle(Vec3(-0.2, 0.5, 0.2), particle_counter++, 1.0 / 100.0));
	// INIT BODIES
	int bodyId = particle_counter;
	bodies.push_back(new RigidBody(Vec3(0.0, 0.1, 0.2), 20, Vec3(1.0, 0.5, 0.2), Vec3(10.0, 15.0, 5.0), Vec3(100, 100, 0), bodyId));
	bodyId += 28;
	bodies.push_back(new RigidBody(Vec3(0.0, 0.1, 0.6), 100, Vec3(0.2, 0.3, 0.1), Vec3(90.0, 0.0, 5.0), Vec3(0, 100, 50), bodyId));
	bodyId += 28;
	for (RigidBody* body : bodies) {
		//printf("Body id: %d\n", body->getID());
		vector<Particle*> body_particles = body->get_particles();
		for (Particle* p : body_particles) {
			particles.push_back(p);
			//printf("Particle id: %d\n", p->getID());
		}
	}
}

void System::init_rod()
{
	/*
		Generate the rod
	*/

	// CONSTANTS
	const double ks = 0.1;
	const double kd = 0.05;
	const double mass = 1.;
	const double dist_unit = 0.1;
	const Vec3 center_1 = Vec3(-0.5, 0.5, 0.0);
	const Vec3 center_2 = Vec3(0.5, 0.5, 0.0);
	const Vec3 center_3 = Vec3(0., 0.5, 0.);
	const Vec3 offset_x = Vec3(dist_unit, 0.0, 0.0);
	const Vec3 offset_y = Vec3(0.0, dist_unit, 0.0);
	const Vec3 offset_z = Vec3(0.0, 0.0, dist_unit);

	// GENERAL FORCES
	forces.push_back(new GravityForce());
	forces.push_back(new DragForce());

	// ROD SYSTEM 1

	// PARTICLES
	particle_counter = 0;
	Particle* p1 = new Particle(center_1, particle_counter++, mass);
	Particle* p2 = new Particle(center_1 - 3.0 * offset_x, particle_counter++, mass);
	Particle* p3 = new Particle(center_1 - 8.0 * offset_x, particle_counter++, mass);
	particles.push_back(p1);
	particles.push_back(p2);
	particles.push_back(p3);

	// CONSTRAINTS
	constraints.push_back(new RodConstraint(p1, p2, dist_unit * 3.0, constraint_counter++));
	constraints.push_back(new RodConstraint(p2, p3, dist_unit * 5.0, constraint_counter++));
	constraints.push_back(new CircularWireConstraint(p1, center_1 + offset_x, dist_unit, constraint_counter++));


	// ROD SYSTEM 2

	// PARTICLES
	Particle* p4 = new Particle(center_2, particle_counter++, mass);
	Particle* p5 = new Particle(center_2 - 1.0 * offset_x, particle_counter++, mass);
	Particle* p6 = new Particle(center_2 - 3.0 * offset_x, particle_counter++, mass);
	Particle* p7 = new Particle(center_2 - 6.0 * offset_x, particle_counter++, mass);
	particles.push_back(p4);
	particles.push_back(p5);
	particles.push_back(p6);
	particles.push_back(p7);

	// FORCES
	forces.push_back(new SpringForce(p5, p6, dist_unit * 2.0, ks, kd));

	// CONSTRAINTS
	constraints.push_back(new RodConstraint(p4, p5, dist_unit * 1.0, constraint_counter++));
	constraints.push_back(new RodConstraint(p6, p7, dist_unit * 3.0, constraint_counter++));
	constraints.push_back(new CircularWireConstraint(p4, center_2 + offset_x, dist_unit, constraint_counter++));

	// ROD SYSTEM 3 (Pendulum)

	// PARTICLES
	Particle* p8 = new Particle(center_3, particle_counter++, 0.);
	Particle* p9 = new Particle(center_3 - 1.0 * offset_y, particle_counter++, mass);
	Particle* p10 = new Particle(center_3 - 2.0 * offset_y, particle_counter++, mass);
	Particle* p11 = new Particle(center_3 - 3.0 * offset_y, particle_counter++, mass);
	particles.push_back(p8);
	particles.push_back(p9);
	particles.push_back(p10);
	particles.push_back(p11);

	// CONSTRAINTS
	constraints.push_back(new RodConstraint(p8, p9, dist_unit, constraint_counter++));
	constraints.push_back(new RodConstraint(p9, p10, dist_unit, constraint_counter++));
	constraints.push_back(new RodConstraint(p10, p11, dist_unit, constraint_counter++));

	printf("DONE");
}

void System::init_rod_collisions()
{

	const double mass = 1.;
	const double dist_unit = 0.1;
	const Vec3 center_1 = Vec3(-0.5, 0.5, 0.0);
	const Vec3 center_2 = Vec3(0.5, 0.5, 0.0);
	const Vec3 center_3 = Vec3(0., 0.5, 0.);
	const Vec3 center_4 = Vec3(0., 0., 0.2);
	const Vec3 center_5 = Vec3(0., 0., 0.6);
	const Vec3 offset_x = Vec3(dist_unit, 0.0, 0.0);
	const Vec3 offset_y = Vec3(0.0, dist_unit, 0.0);
	const Vec3 offset_z = Vec3(0.0, 0.0, dist_unit);
	particle_counter = 0;

	// GENERAL FORCES
	forces.push_back(new GravityForce());
	forces.push_back(new DragForce());

	// ROD SYSTEM 4 (Colliding balls)

	// PARTICLES
	Vec3 offset_pend_z = Vec3(0., 0., 0.042);
	int num_balls = 2;
	for (int c = 0; c < num_balls; c++) {
		Vec3 pos = center_4 + (double)c * offset_pend_z;
		if (c == 0) {
			pos += offset_y - offset_z;
		}
		else if (c == num_balls - 1) {
			pos += offset_y + offset_z;
		}
		Particle* pn1 = new Particle(pos, particle_counter++, mass / 20.);
		Particle* pn2 = new Particle(center_4 + (double)c * offset_pend_z - offset_x + offset_y, particle_counter++, 0.);
		Particle* pn3 = new Particle(center_4 + (double)c * offset_pend_z + offset_x + offset_y, particle_counter++, 0.);
		particles.push_back(pn1);
		particles.push_back(pn2);
		particles.push_back(pn3);

		// CONSTRAINTS
		double pend_rod_dist = norm(offset_x + offset_y);
		constraints.push_back(new RodConstraint(pn1, pn2, pend_rod_dist, constraint_counter++));
		constraints.push_back(new RodConstraint(pn1, pn3, pend_rod_dist, constraint_counter++));
	}

	// ROD SYSTEM 5 (Newton's Cradle)

	// PARTICLES
	num_balls = 5;
	for (int c = 0; c < num_balls; c++) {
		Vec3 pos = center_5 + (double)c * offset_pend_z;
		Particle* pn1 = new Particle(pos, particle_counter++, mass / 20.);
		Particle* pn2 = new Particle(center_5 + (double)c * offset_pend_z - offset_x + offset_y, particle_counter++, 0.);
		Particle* pn3 = new Particle(center_5 + (double)c * offset_pend_z + offset_x + offset_y, particle_counter++, 0.);
		particles.push_back(pn1);
		particles.push_back(pn2);
		particles.push_back(pn3);

		// CONSTRAINTS
		double pend_rod_dist = norm(offset_x + offset_y);
		constraints.push_back(new RodConstraint(pn1, pn2, pend_rod_dist, constraint_counter++));
		constraints.push_back(new RodConstraint(pn1, pn3, pend_rod_dist, constraint_counter++));
	}
}


void System::init_cloth()
{
	printf("\n\nInitializing CLOTH particle system... ");
	bool cross_fibers = true;
	double ks = 1.;
	double kd = 0.;

	const int particles_in_line = 5;

	const Vec3 bottom_left(-0.35, -0.35, 0);
	const Vec3 top_left(-0.35, 0.35, 0);
	const Vec3 bottom_right(0.35, -0.35, 0);
	const Vec3 offset_X = (bottom_right - bottom_left) / double(particles_in_line);
	const Vec3 offset_Y = (top_left - bottom_left) / double(particles_in_line);
	const Vec3 top_right = bottom_left + (offset_X + offset_Y) * double(particles_in_line);
	const double dist = offset_X[0];
	Vec3 mass = Vec3(5, 5, 5);
	Vec3 sliderMass = Vec3(5, 0, 0);

	int i, j;
	particle_counter = 0;
	for (i = 0; i <= particles_in_line; i++) {
		for (j = 0; j <= particles_in_line; j++) {
			particles.push_back(new Particle(bottom_left + offset_X * double(i) + offset_Y * double(j), particle_counter++, 
				j == particles_in_line ? sliderMass : mass));
		}
	}

	// Two stationary particles at the ends to stop the rail
	Particle* limit_left = new Particle(bottom_left + offset_X * double(-1) + offset_Y * double(particles_in_line), particle_counter++, Vec3(0., 0., 0.));
	Particle* limit_right = new Particle(bottom_left + offset_X * double(particles_in_line + 1) + offset_Y * double(particles_in_line), particle_counter++, Vec3(0., 0., 0.));
	particles.push_back(limit_left);
	particles.push_back(limit_right);
	// With a spring inbetween for visuals
	double sliderLen = norm(offset_X * double(particles_in_line + 2));
	forces.push_back(new SpringForce(limit_left, limit_right, sliderLen, 0., 0.));

	double rest = dist / 1.05;
	for (i = 0; i < particles_in_line; i++) {
		for (j = 0; j < particles_in_line; j++) {
			int current = j * (particles_in_line + 1) + i;
			int right = current + particles_in_line + 1;
			int below = current + 1;
			forces.push_back(new SpringForce(particles[current], particles[right], rest, ks, kd));
			forces.push_back(new SpringForce(particles[current], particles[below], rest, ks, kd));
		}
	}
	for (i = 0; i < particles_in_line; i++) {
		int current_1 = (i + 1) * (particles_in_line + 1) - 1;
		int right = current_1 + particles_in_line + 1;
		forces.push_back(new SpringForce(particles[current_1], particles[right], rest, ks, kd));

		int current_2 = i + particles_in_line * (particles_in_line + 1);
		int below = current_2 + 1;
		forces.push_back(new SpringForce(particles[current_2], particles[below], rest, ks, kd));
	}
	if (cross_fibers) {
		double d_rest = rest * sqrt(2);
		for (i = 0; i < particles_in_line; i++) {
			for (j = 0; j < particles_in_line; j++) {
				int current = j * (particles_in_line + 1) + i;
				int right_below = current + particles_in_line + 2;
				forces.push_back(new SpringForce(particles[current], particles[right_below], d_rest, ks, kd));
			}
		}
		for (i = 1; i <= particles_in_line; i++) {
			for (j = 0; j < particles_in_line; j++) {
				int current = j * (particles_in_line + 1) + i;
				int right_above = current + particles_in_line;
				forces.push_back(new SpringForce(particles[current], particles[right_above], d_rest, ks, kd));
			}
		}
	}
	forces.push_back(new GravityForce());
	forces.push_back(new DragForce(-0.018));

	double radius = 0.05;

	// INIT BODIES (Add this after particles)
	int bodyId = particle_counter;
	bodies.push_back(new RigidBody(Vec3(0.0, 0.1, 0.6), Vec3(20., 20., 20.), Vec3(0.15, 0.22, 0.1), Vec3(0.0, 20.0, 0.0), Vec3(0, 100, 50), bodyId));
	bodyId += 28;
	for (RigidBody* body : bodies) {
		printf("Body id: %d\n", body->getID());
		vector<Particle*> body_particles = body->get_particles();
		for (Particle* p : body_particles) {
			particles.push_back(p);
		}
	}

	// Stick the top corners of the cloth
	//particles[particles_in_line]->m_MassInv = Vec3(0., 0., 0.);
	//particles[(particles_in_line + 1) * (particles_in_line + 1) - 1]->m_MassInv = Vec3(0., 0., 0.);

	printf("DONE");
}

void System::init_hair()
{
	printf("\n\nInitializing HAIR particle system... ");
	
	const int internal_particles = 47; // Amount of particles in hair + 2

	particle_counter = 0;
	construct_hair(10, Vec3(-0.1, 0.9, 0.0), Vec3(-0.1, 0., 0.0), true);
	construct_hair(15, Vec3(0.0, 0.9, 0.0), Vec3(0.0, 0.0, 0.0), true);
	construct_hair(20, Vec3( 0.1, 0.9, 0.0), Vec3( 0.1, 0., 0.0), true);

	
	forces.push_back(new GravityForce());
	forces.push_back(new DragForce(-0.07));

	printf("DONE");
}

void System::construct_hair(int internal_particles, Vec3 start, Vec3 end, bool angular) {
	int i = 0;
	Vec3 step = (end - start) / (double) (internal_particles + 1);
	for (i = 0; i <= internal_particles + 1; i++) {
		Vec3 mass = i == 0 ? Vec3(0., 0., 0.) : Vec3(1.0, 1.0, 1.0);
		particles.push_back(new Particle(start + step * (double) i, particle_counter++, mass));
	}

	double rest = norm(step);
	double ks = 5;
	double kd = 0.8;
	double ks_ang = 1.5;
	double kd_ang = 0.25;
	double internal_angle = 180.0 * internal_particles;
	double angle_degrees = internal_angle / (internal_particles + 2.0);
	double angle_radians = M_PI * angle_degrees / 180.0;
	forces.push_back(new SpringForce(particles[particle_counter - internal_particles - 2], particles[particle_counter - internal_particles - 1], rest, ks, kd));
	for (i = particle_counter - internal_particles - 1; i < particle_counter - 1; i++) {
		forces.push_back(new SpringForce(particles[i], particles[i + 1], rest, ks, kd));
		if (angular) {
			forces.push_back(new AngularSpring(particles[i], particles[i - 1], particles[i + 1], angle_radians, ks_ang, kd_ang));
		}
	} 
}


void System::init_net()
{
	printf("\n\nInitializing NET particle system... ");

	bool cross_fibers = true;
	double ks = 50;
	double kd = 1;

	const int particles_in_line = 6;

	const Vec3 bottom_left(-0.35, 0, -0.35);
	const Vec3 top_left(-0.35, 0, 0.35);
	const Vec3 bottom_right(0.35, 0, -0.35);
	const Vec3 offset_X = (bottom_right - bottom_left) / double(particles_in_line);
	const Vec3 offset_Y = (top_left - bottom_left) / double(particles_in_line);
	const Vec3 top_right = bottom_left + (offset_X + offset_Y) * double(particles_in_line);
	const double dist = offset_X[0];
	Vec3 mass = Vec3(1, 1, 1);


	int i, j;
	particle_counter = 0;
	for (i = 0; i <= particles_in_line; i++) {
		for (j = 0; j <= particles_in_line; j++) {
			particles.push_back(new Particle(bottom_left + offset_X * double(i) + offset_Y * double(j), particle_counter++, mass));
		}
	}
	double rest = dist / 1.05;
	for (i = 0; i < particles_in_line; i++) {
		for (j = 0; j < particles_in_line; j++) {
			int current = j * (particles_in_line + 1) + i;
			int right = current + particles_in_line + 1;
			int below = current + 1;
			forces.push_back(new SpringForce(particles[current], particles[right], rest, ks, kd));
			forces.push_back(new SpringForce(particles[current], particles[below], rest, ks, kd));
		}
	}
	for (i = 0; i < particles_in_line; i++) {
		int current_1 = (i + 1) * (particles_in_line + 1) - 1;
		int right = current_1 + particles_in_line + 1;
		forces.push_back(new SpringForce(particles[current_1], particles[right], rest, ks, kd));

		int current_2 = i + particles_in_line * (particles_in_line + 1);
		int below = current_2 + 1;
		forces.push_back(new SpringForce(particles[current_2], particles[below], rest, ks, kd));
	}
	if (cross_fibers) {
		double d_rest = rest * sqrt(2);
		for (i = 0; i < particles_in_line; i++) {
			for (j = 0; j < particles_in_line; j++) {
				int current = j * (particles_in_line + 1) + i;
				int right_below = current + particles_in_line + 2;
				forces.push_back(new SpringForce(particles[current], particles[right_below], d_rest, ks, kd));
			}
		}
		for (i = 1; i <= particles_in_line; i++) {
			for (j = 0; j < particles_in_line; j++) {
				int current = j * (particles_in_line + 1) + i;
				int right_above = current + particles_in_line;
				forces.push_back(new SpringForce(particles[current], particles[right_above], d_rest, ks, kd));
			}
		}
	}

	forces.push_back(new GravityForce());
	forces.push_back(new DragForce(-0.08));

	double radius = 0.05;

	// INIT BODIES
	int bodyId = particle_counter;
	bodies.push_back(new RigidBody(Vec3(0.0, 1, 0.1), Vec3(5., 5., 5.), Vec3(0.30, 0.11, 0.2), Vec3(0.0, 0.0, 0.0), Vec3(0, 100, 50), bodyId));
	bodyId += 28;
	for (RigidBody* body : bodies) {
		vector<Particle*> body_particles = body->get_particles();
		for (Particle* p : body_particles) {
			particles.push_back(p);
		}
	}

	// Stick the top corners of the cloth
	particles[0]->m_MassInv = Vec3(0., 0., 0.);
	particles[particles_in_line]->m_MassInv = Vec3(0., 0., 0.);
	particles[particles_in_line * (particles_in_line+1)]->m_MassInv = Vec3(0., 0., 0.);
	particles[(particles_in_line + 1) * (particles_in_line + 1) - 1]->m_MassInv = Vec3(0., 0., 0.);


	printf("Done");
}


// RETURN PRIVATES
std::vector<Particle*>* System::get_particles()
{
	return &particles;
}

std::vector<RigidBody*>* System::get_bodies()
{
	return &bodies;
}

std::vector<Force*>* System::get_forces()
{
	return &forces;
}

std::vector<Force*>* System::get_constraints()
{
	return &constraints;
}

void System::show_contact_points(vector<Contact> c)
{
	contacts.swap(c);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	display_contact_points = true;
}

void System::hide_contact_points()
{
	contacts.clear();
	glPolygonMode(GL_FRONT, GL_FILL);
	display_contact_points = false;
}
