// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "Solver.h"
#include "System.h"
#include "JacobianMatrix.h"
#include "Integrator.h"
//#include "imageio.h"


#include <stdlib.h>
#include <stdio.h>
#include "GL/glut.h"
#include "include/gfx/vec3.h"
#include "include/gfx/mat3.h"
#include "include/gfx/vec4.h"
#include "include/gfx/mat4.h"
#include "Mouse.h"
#include "Camera.h"
#include "RigidBody.h"
#include "collisionDetector.h"

/* macros */

/* external definitions */

/* global variables */

static int N;
static double d;
static int dsim;
static bool adaptive;
static int iteration;
static int ortographic;
static int dump_frames;
static int frame_number;
static int particle_counter;
static int constraint_counter;

static System* particle_system = new System();
static Solver* solver = nullptr;

static int win_id;
static int win_x, win_y;


// Integrator
static Integrator integrator = nullptr;
int integratorType = 3; // 1 = Euler, 2 = MidPoint, 3 = Runge Kutta 4, 4 = Implicit, 5 = Verlet, otherwise error

// System initalizations
AvailableSystems system_setup = AvailableSystems::NET; // BASIC, ROD, CLOTH, HAIR, NET

// Camera and mouse
Camera camera = Camera();
Mouse mouse = Mouse(*particle_system->get_particles(), *particle_system->get_bodies(), camera);
int16_t mouse_last_x = 0, mouse_last_y = 0;
bool updateMouse = false;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/


static void init_system(void) {

	// Reseting jacobian matrix and particle forces and contraints.
	particle_system->clear_data();
	particle_system->initialization(system_setup);

	double ks = 10;
	double kd = 1;
	double epsilon = 1e-8;

	// CLOTH
	if (system_setup == AvailableSystems::CLOTH) 
	{
		ks = 50;
		kd = 15;
		epsilon = 1e-4;
	}

	std::vector<Particle*>* particles = particle_system->get_particles();
	std::vector<RigidBody*>* bodies = particle_system->get_bodies();
	std::vector<Force*>* forces = particle_system->get_forces();
	std::vector<Force*>* constraints = particle_system->get_constraints();

	solver = new Solver(*particles, *bodies, *forces, *constraints, ks, kd, epsilon, mouse);
	integrator = Integrator(solver);
  
	particle_system->reset_data();
	integrator.reset();
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	GLfloat aspect = (GLfloat)win_x / (GLfloat)win_y;
	if (ortographic) {
		glOrtho ( -1.0, 1.0, -1.0, 1.0, -1.0, 100.0 );
	}
	else {
		gluPerspective(camera.getFOV(), aspect, camera.getNearPlane(), camera.getFarPlane());
	}
	Vec3 camera_dir = camera.getDirection();
	Vec3 camera_pos = camera.getPosition();
	Vec3 look_at_pos = camera_pos + camera_dir;
	Vec3 camera_up = camera.getUp();

	gluLookAt(camera_pos[0], camera_pos[1], camera_pos[2],
		look_at_pos[0], look_at_pos[1], look_at_pos[2],
		camera_up[0], camera_up[1], camera_up[2]);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer) {
				exit(-1);
			}

			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			//sprintf(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			//saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	camera.buttonPressed(key, x, y);
	switch (key)
	{
		// Integrators key
	case '1':
		integratorType = 1;
		printf("\n\n\nNew intergrator type: Euler");
		break;

	case '2':
		integratorType = 2;
		printf("\n\n\nNew intergrator type: MidPoint");
		break;

	case '3':
		integratorType = 3;
		printf("\n\n\nNew intergrator type: Runge Kutta 4");
		break;

	case '4':
		integratorType = 4;
		printf("\n\n\nNew intergrator type: Implicit");
		break;

		// Scene selection
	case '5':
		system_setup = AvailableSystems::BASIC;
		init_system();
		break;

	case '6':
		system_setup = AvailableSystems::ROD;
		init_system();
		break;

	case '7':
		system_setup = AvailableSystems::ROD_COLLISIONS;
		init_system();
		break;

	case '8':
		system_setup = AvailableSystems::CLOTH;
		init_system();
		break;

	case '9':
		system_setup = AvailableSystems::HAIR;
		init_system();
		break;

	case '0':
		system_setup = AvailableSystems::BODIES_COLLISIONS;
		init_system();
		break;

	case 'l':
	case 'L':
		system_setup = AvailableSystems::NET;
		init_system();
		break;

		// Control keys
	case 'r':
	case 'R':
		particle_system->reset_data();
		if (&integrator != nullptr) {
			integrator.reset();
		}
		break;

	case 'p':
	case 'P':
		dump_frames = !dump_frames;
		break;

	case 'o':
	case 'O':
		ortographic = !ortographic;
		break;

	case 't':
	case 'T':
		printf("\n\n\nTerminating program\n\n");
		particle_system->free_data();
		exit(0);
		break;

	case 'y':
	case 'Y':
		adaptive = !adaptive;
		printf("\n\tTurned %s adaptive step size", adaptive ? "on" : "off");
		break;

	case 'f':
	case 'F':
		particle_system->drawForcesOnParticles = !particle_system->drawForcesOnParticles;
		break;

	case ' ':
		dsim = !dsim;

		if (dsim) {
			printf("\n\nStarted simulating\n");
			integrator.reset();
		}
		else {
			printf("\n\tStopped simulating");
		}

		break;
	case 'b':
		if (integrator.is_collisions_enabled()) {
			printf("\n\tDisabled collisions\n");
		}
		else {
			printf("\n\tEnabled collisions\n");
		}
		integrator.set_collisions(!integrator.is_collisions_enabled());
		break;
	case 'c':
	{
		vector<Contact> contacts;
		CollisionDetector collisionDetector = CollisionDetector();
		collisionDetector.find_contacts(particle_system->get_state(), contacts);
		for (Contact contact : contacts) {
			if (contact.vf) {
				Vec3 a = contact.p;
				Vec3 b = contact.p + contact.n;
				//printf("Normal: (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)\n", a[0], a[1], a[2], b[0], b[1], b[2]);
			}
		}
		particle_system->show_contact_points(contacts);
		break;
	}
	}
	updateMouse = true;
}

static void mouse_func ( int button, int state, int x, int y )
{
	mouse_last_x = x; mouse_last_y = y;
	mouse.updateButton(button, state, x, y);
}

static void motion_func ( int x, int y )
{
	mouse_last_x = x; mouse_last_y = y;
	mouse.updateCoords(x, y);
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if (dsim) {
		if (DEBUG) printf("\n\nIteration %d", ++iteration);
		integrator.simulation_step(particle_system, integratorType, adaptive);
	}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();
	if (updateMouse) {
		mouse.updateCoords(mouse_last_x, mouse_last_y);
		updateMouse = false;
	}
	particle_system->draw(mouse);
	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(
		glutGet(GLUT_SCREEN_WIDTH) / 2 - win_x / 2, 
		glutGet(GLUT_SCREEN_HEIGHT) / 2 - win_y / 2
	);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Particletoys!");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n", N, particle_system->dt, d );
	} else {
		N = atoi(argv[1]);
		particle_system->dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n");
	printf("\n\t SPACE: Toggle construction/simulation display");
	printf("\n\t R: Restart simulation");
	printf("\n\t P: Dump frames");
	printf("\n\t O: Toggle Ortographic/3D view");
	printf("\n\t T: Exit");
	printf("\n\t WASDQE: Move");
	printf("\n\t b: Enable/Disable collisions");
	printf("\n\t UHJK or RightClick: Look around");
	printf("\n\t +-: Adjust FOV");
	printf("\n\t Y: Toggle adaptive step size");
	printf("\n\t F: Toggle drawing forces on particles");
	printf("\n\t 1234: Use integrator: Euler, MidPoint, RangeKutta4, Implicit");
	printf("\n\t 567890L: Pick scene: Basic, Rod, Rod Collisions, Cloth, Hair, Body collisions and Net");

	dsim = 0;
	adaptive = true;
	dump_frames = 0;
	frame_number = 0;
	
	init_system();
	
	win_x = 1920;
	win_y = 1080;
	open_glut_window ();
 
	glutMainLoop ();

	exit ( 0 );
}

