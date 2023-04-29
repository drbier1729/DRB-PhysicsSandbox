--------------------------------------------------------------------------------
About
--------------------------------------------------------------------------------
- Created for CS550 at DigiPen Institute of Technology, Spring 2023
- Demo real-time physics engine which features:
    - Dynamic bounding volume hierarchy of AABBs for broad-phase collision detection
    - Rigid geometry represented by composite of polyhedra (half-edge meshes)
    - Separating Axis Test for narrow-phase collision detection using Gauss Map optimization
    - Contact generation using Sutherland-Hodgman clipping algorithm
    - Extended Position-Based Dynamics (XPBD) with substepping for collision constraint resolution
    - (Unfinished) implementation of Gilbert-Johnson-Keerthi algorithm to support in narrow-phase collision detection

--------------------------------------------------------------------------------
Build and Run
--------------------------------------------------------------------------------
- if you'd like to build...
    - go to "solution" directory
    - open the PhysicsSandbox.sln using Visual Studio 2022
    - select "Release" build ("Debug" works, but is too slow for the complex
        scenes)
    - build and run from Debugger
- required dependencies...
     - all dependencies (assimp, imgui, etc) are included in "deps" directory which includes their licences.
        I know this isn't very profesh, but since this is a hobby/academic project I don't feel too bad about it. 
        Feel free to set up those dependencies on your own if you'd like.

--------------------------------------------------------------------------------
Demo Controls
--------------------------------------------------------------------------------

- ESC         : Quit

- WASD        : Move camera forward/backward + left/right
- Arrows      : Apply force to selected RigidBody in +/- y and +/- x directions
- Left Mouse  : Select RigidBody (hover and click)
- Right Mouse : Hold and drag mouse to turn camera

- NumKey 1    : Toggle Pause
- NumKey 2    : Toggle Single-step mode

(While Paused)
- NumKey 9/0  : Step backward/forward

--------------------------------------------------------------------------------
GUI
--------------------------------------------------------------------------------

Scene Selector:
- Click the button to choose the scene to display.
- Scenes cannot be reset except by restarting the executable.

Simulation Info:
- View avg render FPS, avg simulating time in microseconds, and the current
    step of the simulation.
- STATUS will display "SIMULATING" when the physics world is progressing (either
    by unpausing, or stepping forward while paused), and will show "REPLAY" when
    stepping backward/forward through recorded frames.
- Click "Draw Bounding Volume Hierarchy" and "Draw Contact Manifolds" to show
    each of these. Note that these are not accurate in "REPLAY" mode since they
    are not recorded.
    
RigidBody Viewer:
- Click a RigidBody in the scene to select it.
- The physical parameters of the body can be viewed for this sim step. This is
    recorded and so will be accurate while in "REPLAY" mode.

--------------------------------------------------------------------------------
Visuals
--------------------------------------------------------------------------------

- Dynamic rigidBodies are RED, Static are GREEN. 

- RigidBody highlighted in BLUE is the one your cursor is hovering over (click to
"select" it). The currently "selected" RigidBody is highlighted in ORANGE.

- WHITE arrows are linear velocity vectors.

(While Drawing Contact Manifolds)
- YELLOW cubes are contact points on each body.
- BLUE quad is the contact plane on the "reference" body.
- YELLOW arrow is the contact normal.

(While Drawing Bounding Volume Hierarchy)
- WHITE wireframes are nodes in the AABB tree.
