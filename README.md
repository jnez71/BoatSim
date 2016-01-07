Autonomous Boat Simulator implemented in MatLab (R2013a).
Run BoatSim.m in the Command Window to begin.

Overview
---

While the simulator is running, various parameters can be adjusted
using the buttons in the bottom left of the figure.
("Center" recenters the camera on the boat).

The initial conditions are displayed in the Command Window when the sim starts. Any errors or warnings will also be printed there so take a look if something behaves incorrectly.

TimeSeries mode runs the sim for Config_Sim.T seconds as fast as your computer can compute it and then plots the results immediately. It is not interactive. 2D mode is realtime and interactive, but it only shows the 2D cross-section of what's going on (even though full 3D physics are still computed). This is a bit easier for most computers to run graphics-wise. 3D mode is the same as 2D mode but a full 3D model is animated. Since animation is done manually with the MatLab plot tool, it's computationally intensive, but can run decently smooth on good computers in slightly under realtime.

To design the robot, sim models, and/or initial conditions and parameters, adjust the config_* files according to the details below.

- All units are SI base
- Body coordinates are XYZ = FORWARD LEFT UP

config_robot
---
The only required function in this class is Decide(). It must output [command] which is a 1by4 array. It is given access to any of the other class instances (state, boat, sim...) and can use them however you design it. As you can see, sensor implementation is up to the robot designer. If the boat you are running is an...

**azi drive**

command = [thrust left, thrust right, angle left, angle right]

azi angles are measured positive from body x to body y

**fixed drive**

command = [back left, back right, front left, front right] thrusts

**direct (wrench) drive**

command = [world x force, world y force, world z torque, 0]

config_boat
---
Boat specific parameters, geometry, and thruster models. It must have at least these three functions...

- [Ft, Mt] = AziThrust(boat, state, sim, command)
- [Ft, Mt] = FixedThrust(boat, state, command)
- [Ft, Mt] = DirectThrust(boat, state, command)

where Ft and Mt are the resulting net thruster force (world frame) and moment (body frame) on the boat given the current state of the thrusters stored in state.thrusters (follows same formatting as [command]).

config_env
---
Environmental parameters and models. Each is commented accordingly. Feel free to change any parameters or models as you see fit.

config_sim
---
Simulation parameters like timestep, duration (for timeseries sims), and initial viewing window.

Notes
---
If you are wondering about the boat graphic being used for the 3D sim, have a look here: <http://propagator.org/?page_id=1765>.

BoatSim was created primarily for fun (who doesn't love playing god?). A lot of the coding is... dirty (the fact that all my configs inherent from Handle is hilarious), because I rushed finishing it so I could get started on a bigger simulation project that will stand upon the experience I got making this one. The new simulator is in Python and is called jSim. It may or may not exist yet.

Don't get me wrong though, despite being messy internally, this sim works very well! Feel free to contact me (nez.jason7@gmail.com) if you want a better description of how it works.

Pictures for Hype
---
![alt tag](http://s27.postimg.org/tt7ggn2fn/Boat_Sim_Pic_1.png)
![alt tag](http://s3.postimg.org/wqplm1m3n/Boat_Sim_Pic_2.png)
![alt tag](http://s30.postimg.org/isf577odd/Boat_Sim_Pic_3.png)
![alt tag](http://s27.postimg.org/j2t0mg3kj/Boat_Sim_Pic_4.png)
