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

Single agent navigate in environment

![36aeddc3f271ce016594667e263b8d1](https://user-images.githubusercontent.com/35856355/135949147-f3dc30a7-26e7-4d73-8a3f-21499f720313.png)


Multiple agents navigate in environment

![7292672e5a955ea95c113416929efe5](https://user-images.githubusercontent.com/35856355/135948744-317b4119-842f-4508-88e1-f6913f0da8f5.png)


Multiple agents navigate in environment while avoid colliding with each other

![950a6121019ad5707ef2f03537bf672](https://user-images.githubusercontent.com/35856355/135949012-fde53e79-5f09-4d85-af6b-d11d275398b2.png)

![5e15067367fff5aaf5c1adedc198d54](https://user-images.githubusercontent.com/35856355/135948987-c82ec72e-45fc-4348-8d20-874093f45352.png)


## Video & Time Stamp
Link:
https://youtu.be/ZLE3K21MKyI

Time Stamp:
* 0:00:09-0:00:22 delete an agent from scene
* 0:00:26-0:00:27 single agent navigation
* 0:0027-0:0037 adding new agents to scene and multiple agents navigations
* 0:00:42-0:00:50 enable and disable sphere sourround agents and obstacles
* 0:00:58-0:01:00 enable and disable lines connect from agents to goal
* 0:01:00-0:01:35 part 2b multiple agents navigation and avoid obstacles
* 0:01:38-0:02:10 camera movement and rotation
* 0:02:12-0:03:07 challenge part -- simulations in 2 scenes


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
