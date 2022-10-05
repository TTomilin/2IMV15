========================================================================
    CONSOLE APPLICATION : SICGConsole Project Overview
========================================================================

/* This project can be run by Visual Studio (2015) using C++ Development, 
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
 * This folder contains freeglut 3.0. 
 * Download and extract freeglut, and move glut32.lib to 
 * %PROGRAMS DIRECTORY%\Microsoft Visual Studio 14.0\VC\lib
 * For VS2017, it's: %PROGRAMS DIRECTORY%\Microsoft Visual Studio\2017\
 * Community\VC\Auxiliary\VS\lib\x64
 * 
 * Put back together by Merlijn Busink. 
 * 
 * 
 * Changes: 
 * 
 * Removed SICGConsole's main function in order for ParticleToy to function.
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
 * Minor syntax changes (a.o. double-float multiplication, changed #include<> 
 * to #include""). 
*/

AppWizard has created this SICGConsole application for you.

This file contains a summary of what you will find in each of the files that
make up your SICGConsole application.


SICGConsole.vcxproj
    This is the main project file for VC++ projects generated using an Application Wizard.
    It contains information about the version of Visual C++ that generated the file, and
    information about the platforms, configurations, and project features selected with the
    Application Wizard.

SICGConsole.vcxproj.filters
    This is the filters file for VC++ projects generated using an Application Wizard. 
    It contains information about the association between the files in your project 
    and the filters. This association is used in the IDE to show grouping of files with
    similar extensions under a specific node (for e.g. ".cpp" files are associated with the
    "Source Files" filter).

SICGConsole.cpp
    This is the main application source file.

/////////////////////////////////////////////////////////////////////////////
Other standard files:

StdAfx.h, StdAfx.cpp
    These files are used to build a precompiled header (PCH) file
    named SICGConsole.pch and a precompiled types file named StdAfx.obj.

/////////////////////////////////////////////////////////////////////////////
Other notes:

AppWizard uses "TODO:" comments to indicate parts of the source code you
should add to or customize.

/////////////////////////////////////////////////////////////////////////////
