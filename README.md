

[TOC]

# Group 11


## Videos
- Presentation: https://youtu.be/98hKwCLSUoo
- Demo: https://youtu.be/hfL2dRX3G_k

## Installation
- Clone the repository
- Copy `./.vs/SICGConsole/v16/.suo.copy` to `./.vs/SICGConsole/v16/.suo`
- Open Visual Studio
- Import the project
- Link the Eigen3 library with the following:
        1) Install globally using vcpkg package manager, tutorial: https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019
        2) Install locally to desired location and link at Project -> Properties -> Linker -> General -> Additional Library Directories


## Definitions

Screen orientation, relative from the person sitting behind the computer watching the screen. 

- +x is to the right of the screen
- +y is upwards
- +z is outwards (toward the user) (+)



| Var  | Means                                             |
| ---- | ------------------------------------------------- |
| Q    | accumulated forces                                |
| M    | Mass of the particles                             |
| W    | 1/M                                               |
| C    | Shift in position that would happen (F1, page 98) |
| J    |                                                   |
|      |                                                   |







## Planning
 - Implicit + force constraints for 2D cloth
    - (Sliding or fixing points should  be  regarded  as  constraints  and  implemented  using  the  method  of  Lagrange mutipliers.)
- Horizontal force on a Cloth (disable 3th axis)
- Collision with solid objects (one-way coupling)
- Two-way coupling



### Required

- Video			   - Show what you have done (show off)

- Presentation    - Explain what you have done and how

- Report             - The same old, same old.

  




## Links



### Camera

- [OpenGL camera](https://learnopengl.com/Getting-started/Camera)
- [Camera from GitHub](https://github.com/deiss/camera)
- [In Glut using stackoverflow](https://stackoverflow.com/questions/27488230/rotating-camera-in-opengl-using-glut-libraries-and-glulookat)

### GLUT

- [Specification](https://www.opengl.org/resources/libraries/glut/spec3/node1.html)

