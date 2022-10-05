#pragma once
#include "IObject.h"
#include "include/gfx/vec3.h"

struct Contact
{
	IObject* a;		// Body containing vertex
	IObject* b;		// Body containing face
	Vec3	 p,		// World-space vertex location
		n,		// Outwards pointing normal of face
		ea,		// Edge direction for A
		eb;		// Edge direction for B
	bool    vf;		// True if vertex/face contact
};