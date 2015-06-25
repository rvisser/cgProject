# Computer Graphics Project

## Project members
+ Ruben Visser (4081641)
+ Jens Voortman (4081005)
+ Ruben Bes (4227492)
+ Diony Tadema (4219198)
+ Aurél Bánsági (4251342)
+ Tom Brunner (4217160)
+ Rob van den Berg (4319427)


## Implemented features
+ Ray intersection with planes, and triangles
+ Illumination of the impact point which is drawn in the resulting color
+ An option to move the light sources in the scene
+ A shaded display in OpenGL of the 3D scene
+ A ray-tracing acceleration structure (KdTree)
+ Reflections
+ Supersampling
+ Perform an actual full-image ray-tracing instead of a single ray
+ Designed our own 3D scene in Blender
+ Multithreading with OpenMP (Dynamic scheduler)
+ Live reloading of (some values of) the configuration file


## Controls
+ l: Add Light
+ m: Move last added light to the current Camera Position.
+ s: Shoot a single ray.
+ r: Perform the raytrace.
+ (Numpad) 4: Select previous light.
+ (Numpad) 6: Select next light.
+ (Numpad) 2: Move selected light to current Camera Position.
+ x: Remove the selected light from the scene.
+ c: Reloads the configuration file (not al values have effect when reloading. See comments in the file)
+ Esc: Exit the program

## Legenda
+ The currently selected light is highlighted in red.
+ There must always be at least one light in the scene.

## External sources
+ PNG output: [lodepng](http://lodev.org/lodepng/)
+ Triangle/Box intersection based the [example code](http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/) from: Tomas Akenine-Möller, ``Fast 3D Triangle-Box Overlap Testing,'' journal of graphics tools, vol. 6, no. 1, pp. 29-33, 2001.
