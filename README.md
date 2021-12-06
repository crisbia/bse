**NB: This project is going through some modernization work at the moment. The version on 'main' is an initial drop of some pretty old code in an unstable state.**


**BSE – Bi-dimensional Simulation Environment**

**Platform**

Tested on MacOS. More platforms coming soon.

**Overview**

The main purpose of this project has always been to make some programming practise on some games related technologies. Given the time constraints of my job, I decided to go for a 2D framework, in order to be able to implement more features, simpler than the equivalent 3D version, in the same time. Most of the concepts and algorithms I used apply also to 3D problems, so implementing them in 2D gave me at least a rough idea of what it would take to do the same in a 3D context.

**Description**

BSE is a 2D framework for real time simulations. Its components are _bsePhysics_ and _bseAI_. _bsePhysics_ is a 2D physics engine and the idea behind _bseAI_ is to collect some typical game AI tools.

The physics engine currently implements the following features:

- Rigid body dynamics, based on a split impulses method and an islands solver.
- Broadphase collision system, both simple (_O(n __2__ )_) and Sweep and Prune.
- Narrow phase collision system: circle and box primitive intersection algorithm, simple polygon-polygon contact determination (no gjk).
- Raycasting system.
- Quadtree based object selection.

The SAP implementation is of the &#39;incremental&#39; type. Every time a geometric shape moves, its aligned bounding box is updated. SAP maintains one list of intervals per axis (x and y). The lists are updated according to shapes&#39; movements. New overlaps (or intervals stopping overlapping) are detected while &#39;sweeping&#39; the intervals into the proper order. If few objects move or there are a lot of static objects, this algorithm is really efficient. Some tests run on the BSEDemo (pyramid of bodies) show that the SAP broadphase collider is about 60-65% faster than the naive collider that checks the AABB intersections between every pair of shapes. In the Maze demo, where the majority of objects is static, the benefits of the SAP approach are huge: the SAP broadphase is around 200 times faster (maze test with 2.500 cells, 10 npc chasing the player).

The AI engine is very limited, most of the effort was put in implementing an _A\*_ pathfinder, actually. The pathfinder algorithm is quite standard, there&#39;s no particular simplification or optimization implemented. Although, I had a lot of care for making sure that two generic instances of the search algorithm running at the same time share the pathfinder graph only in read mode. All the data needed for the search to run are completely independent. This makes the search a so called &quot;embarrassingly parallel&quot; problem, which basically means that many instances of the search can safely run in parallel and don&#39;t need to synchronize in writing shared data.

Some know issues:

- No effort has been put so far in trying to make &quot;stacks of bodies&quot; stable and robust. This was not the main initial goal.
- Lack of documentation.
- Many and many other optimizations to do.

Missing (interesting) features:

- Mechanical joints.
- Scene description importer.
- Pluggable memory management. Started thinking about it, but not much work done.
- Many interesting game AI features that I would like to try and implement, for example: hierarchical state machines, decision trees, flocking behaviours.

**Demos**

There are currently 3 demo projects:

- _BSEDemo_ is a generic demo framework. A _TestManager_ manages all the tests included in the demo, taking care of custom simulation code, custom rendering code, input management, message logging, debug graphics, scene settings, and so on. All the tests that are currently part of the demo are meant to show the physics capabilities of BSE (no AI).
- _BrickBSE_ is a very basic implementation of a _BreakOut_ clone. It&#39;s minimal, it could be not very interesting from a technical point of view, but it shows how a simple game like that can be written in a few lines of code when a 2d engine is available.
- _MazeBSE_ shows the pathfinder at work. It generates a random &quot;perfect maze&quot; (eg: any 2 points in the maze are connected) and place some &quot;enemies&quot; on it. The enemies run the search algorithm for the player and try to follow the path. The demo shows how the pathfinder can easily be run in parallel. Move the light-blue character with the arrow keys. The non-player characters continuously run a pathfinder and follow the computed path. The key &#39;1&#39; enables/disables the asynchronous pathfinder. When the asynchronous search is enabled, all the available logical processors are used.

All the demos share some simple useful piece of tech, for example a basic debug graphics manager, which allows to draw primitive through _opengl_ in a quite transparent way, and a user input manager that collect mouse and keyboard input.
