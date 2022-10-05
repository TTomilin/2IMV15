#include "stdafx.h"
#include "Camera.h"

Camera::Camera()
{
	reset();
}

Vec3 Camera::getDirection()
{
	return Vec3(
		sin(tilt) * cos(pan),
		cos(tilt),
		sin(tilt) * sin(pan)
	);
}

Vec3 Camera::getRight(void)
{
	Vec3 right = cross(getDirection(), GLOBAL_UP);
	unitize(right);
	return right;
}

Vec2 Camera::getTiltAndPan()
{
	return Vec2(tilt, pan);
}

Vec3 Camera::getUp(void)
{
	Vec3 cameraDirection = getDirection();
	Vec3 right = getRight();
	Vec3 cameraUp = cross(right, cameraDirection);
	unitize(cameraUp);
	return cameraUp;
}

Vec3 Camera::getPosition()
{
	return position;
}

double Camera::getFOV()
{
	return fov;
}

double Camera::getNearPlane()
{
	return NEAR_PLANE;
}

double Camera::getFarPlane()
{
	return FAR_PLANE;
}

void Camera::reset()
{
	position = Vec3(START_POS[0], START_POS[1], START_POS[2]);
	tilt = START_TILT;
	pan = START_PAN;
	fov = START_FOV;
}

void Camera::setTiltAndPan(Vec2 data)
{
	tilt = data[0];
	if (tilt < 0.0001) {
		tilt = 0.0001;
	}
	if (tilt > M_PI - 0.0001) {
		tilt = M_PI - 0.0001;
	}

	pan = data[1];
	while (pan < 0) {
		pan += 2 * M_PI;
	}
	while (pan > 2 * M_PI) {
		pan -= 2 * M_PI;
	}
}

void Camera::buttonPressed(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'g':
	case 'G':
		reset();
		break;

	case 'w':
	case 'W':
		position += MOVE_STEPSIZE * getDirection();
		break;

	case 's':
	case 'S':
		position -= MOVE_STEPSIZE * getDirection();
		break;
	case 'a':
	case 'A':
		position -= MOVE_STEPSIZE * getRight();
		break;

	case 'd':
	case 'D':
		position += MOVE_STEPSIZE * getRight();
		break;

	case 'q':
	case 'Q':
		position -= MOVE_STEPSIZE * GLOBAL_UP;
		break;

	case 'e':
	case 'E':
		position += MOVE_STEPSIZE * GLOBAL_UP;
		break;

	case 'u':
	case 'U':
		tilt -= TILT_STEPSIZE;
		if (tilt < 0.0001) tilt = 0.0001;
		break;

	case 'j':
	case 'J':
		tilt += TILT_STEPSIZE;
		if (tilt > M_PI - 0.0001) tilt = M_PI - 0.0001;
		break;

	case 'h':
	case 'H':
		pan -= PAN_STEPSIZE;
		if (pan < 0.0) {
			pan += M_PI * 2.0;
		}
		break;

	case 'k':
	case 'K':
		pan += PAN_STEPSIZE;
		if (pan > M_PI * 2.0) {
			pan -= M_PI * 2.0;
		}
		break;

	case '+':
	case '=':
		fov += FOV_STEPSIZE;
		printf("\n\t Changed FOV to %i", (int) fov);
		break;

	case '_':
	case '-':
		fov -= FOV_STEPSIZE;
		printf("\n\t Changed FOV to %i", (int) fov);
		break;
	}
}