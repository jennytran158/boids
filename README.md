# Boids
## Introduction
A simulation in Scrimmage of a battle between two swarms of drones, each with different objectives and setup.
## Video Demo
https://youtu.be/g0th54Eyt7c
<iframe width="560" height="315" src="https://www.youtube.com/embed/g0th54Eyt7c" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
* Note
	* Red Team (defenders) vs Blue Team (attackers)
	* Each yellow line is the velocity vector of an individual drone.
	* Each red line indicates a pair of defender (red) and its target (blue)
	* The green hyper rectangular at the origin is the defenders’ base.  Its color will turn from green to red once it has been attacked

### Objectives
* Red Team (defenders): Protect the team’s base against blue team.
* Blue Team (attackers): Attack defenders’ base.
* To win the game
	* Red Team (defenders): Eliminate all attackers before any reaches the base
	* Blue Team (attackers): At least one attacker crosses the boundaries of the base
### Team Setup
Drones from two teams have some similarities and differences in their setup.

<img src="https://github.com/jennytran158/boids/blob/master/images/FOV.png" alt="FOV" width="250"/>

* Similarities
	* Same maximal speed
	* Same observable area. Any drone can detect enemies that are inside the grey pyramid (The pyramid is hidden in the demo but shown here for visualization) and communicate to team members within a specified radius.
* Differences
	* Ability to attack: Only defenders can attack.  Any attackers flying within the red pyramid are attackable and if they remain inside the red pyramid for a certain duration (3 timesteps in the demo), they are eliminated.  A defender can eliminate only one attacker at a time.
	* Strategies: The strategies are implemented at individual level and inspired by boids. The velocity vector of each drone at the next time step is found as follows:
		* Find repulsion vectors between: the drone and all of its local neighbors (team or non-team)
		* Find attraction vectors between: the drone and its goal, the drone and average position of the drone’s local team members
		* Find the average heading of local team members
		* Finally, find the desired velocity vector which is the weighted sum of the vectors found above
		* __Red Team (defenders)__: Each defender’s goal position is dynamically changed during the battle and is either an attacker or the base.  If the defender detects any enemies, its goal position is set to the closest enemy.  If no enemies are detected, the goal position is set to the base.  The weights for defenders are below.  Because the weights for avoiding attackers, centroid and align are 0s, the defenders’ general strategy is to attack closest enemies or to go back to the base while avoiding collision with team members.![picture alt](https://github.com/jennytran158/boids/blob/master/images/defenders_weights.png)
		* __Blue Team (attackers)__: Attackers’ goal position stays the same for the whole simulation and is the defenders’ base.  The attackers’ strategy is to move together (because align and centroid weights > 0) toward the base while avoiding collisions with the defenders (avoid non-team weight > 0) and other team members (avoid team weight > 0).
		![picture alt](https://github.com/jennytran158/boids/blob/master/images/attackers_weights.png)

## Implementation:
Original source code: Scrimmage's [Boids](https://github.com/gtri/scrimmage/blob/master/src/plugins/autonomy/Boids/Boids.cpp) autonomy 

Scrimmage: https://www.scrimmagesim.org/sphinx/html/index.html
__Plugins used in this project:__
![picture alt](https://github.com/jennytran158/boids/blob/master/images/boids.png "Plugins used in the project")

#### Files for boids autonomy:
* src/plugins/autonomy
	* the source file (.cpp) contains the implementation and logics of boids
* include/boids/plugins/autonomy
	* a header (.h)
	* a parameter (.xml) Boids’ settings - weights

## Build
    $ cd path-to-project
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
## Run
    $ cd path-to-project
    $ source ~/.scrimmage/setup.bash 
    $ scrimmage missions/boids.xml
