// SICGConsole.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


/* Removed SICGConsole's main function in order for ParticleToy to function.
 * 
 * Added freeglut.dll in the project, does not need to be added anymore. 
 * 
 * Moved import files from the include/gfx/ folder to the project's main folder
 * 
 * Removed imageio.h and imageio.cpp, its dependencies and references to png.h
 * since png.h was outdated. They can be added back in, using the originals. 
 * Since sprintf() has become unsafe, it has been commented out in imageio
 * together with saveImageRGBA(). 
 * 
 * Removed "const bool true/false" since newer versions does not support the
 * assignment of a value to a variable named "true" or "false". 
 * 
 * Removed OS check, since only the windows config file can be found. 
 * 
 * Minor syntax changes (double-float multiplication, changed #include<> to 
 * #include""). 
 * 
 * This project can be run by Visual Studio (2015) using C++ Development, 
 * freely available from Microsoft. 
 * 
 * OpenGL should already be contained in Visual Studio, so there is no need
 * to install it. 
 * 
 * glut.h has been included in the project, but newer versions can be gotten
 * from https://www.opengl.org/resources/libraries/glut/glut_downloads.php
 * The version currently used is 3.7.6 alpha
 * 
 * To use this project, one still has to install (free)glut's glut32.lib, which
 * can be downloaded from http://freeglut.sourceforge.net/
 * Download and extract freeglut, and move glut32.lib to 
 * %PROGRAMS DIRECTORY%\Microsoft Visual Studio 14.0\VC\lib
 * 
 * Put back together by Merlijn Busink. 
*/