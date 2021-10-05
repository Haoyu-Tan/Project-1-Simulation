# Project 1 Simulation

## Description

This is a navigation simulation project using Processing. The program will generate a random scene with obstacles and several agents. Each agent will be assigned to a random goal in the scene at the beginning, and it will find a reasonable path to move naturally toward the goal while the program is running. The program uses Probabilistic Roadmap(PRM) with A* to generate an optimal path for each agent, based on a large number of nodes generate in the scene. At the same time, each agent will also find shortcut during its movement. Once an agent reach its current goal, the next goal will be automatically selected and assigned to it.

In challenge part, each agent will try its best to avoid other agents while finding its goal. The strategy implemented in this part is TTC forces.
 
## Features Implemented

PART 2B:

* Single Agent Navigation
* 3D Rendering & Camera
* Improved Agent & Scene Rendering
* Orientation Smoothing
* Multiple Agents Planning

Challenge: Crowd Simulation (grad*)

## User Controls

* 'r' -- reset to a randomly generate scene
* 'z' -- add an agent to scene. The maximum number of agents is 10
* 'x' -- remove an agent from scene. The minimum number of agents is 0
* 'c' -- click to able or disable the rendering of sphere boundary of each 3D model
* 'v' -- click to able or disable the rendering of path from an agent's current position to its goal
* 'w', 'a', 's', 'd', 'q', 'e' -- navigate the movement of camera
* 'up', 'down', 'left', 'right' -- control the rotation of camera


## Difficulties:

The first difficulty is to modify the structure based on the previous part (plan path and running PRM). Chaning the project from 2D to 3D requires fixing the structure of existing code, which is not hard but time consuming.

The second difficulty is the challenge part. I first implemented the TTC step by step based on what we learnt on class. However, when the simulation ran for a long time, agents will started getting stuck with each other when they stay among several obstacles. This problem was greatly improved when I chose to update the forces and velocity on an agent together rather than seperately update the forces for all agents first and then update their velocity. The effect was that some agents looked "arrogant" and pushed others away a little bit to avoid all of them stucking together.

## Images

## Video & Time Stamp

## Art Contest
Christmas Eve

![cf94475902130bc47f88cfe8caaaaeb](https://user-images.githubusercontent.com/35856355/135948495-f3576bb6-8381-4290-bca6-70c97b504a8a.png)

With debug mode open

![3ba84c0af78cdfedfaf3f80fb6ce7ff](https://user-images.githubusercontent.com/35856355/135948047-a957fbc9-3096-4d3c-b028-98dec0320c81.png)



## References

* model of sled: https://assetstore.unity.com/packages/3d/characters/low-poly-winter-pack-78938
* model of ground texture: https://assetstore.unity.com/packages/2d/textures-materials/lowpoly-textures-pack-140717
* all other models: https://quaternius.com/index.html
* Camera code and the basic framework for running PRM is provided by Liam Tyler and professor Guy
