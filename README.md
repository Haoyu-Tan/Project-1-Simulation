# Project 1 Simulation

## Description

This is a navigation simulation project using Processing. The program will generate a random scene with obstacles and several agents. Each agent will be assigned to a random goal in the scene at the beginning, and it will find a reasonable path to move naturally toward the goal while the program is running. The program uses Probabilistic Roadmap(PRM) with A* to generate an optimal path for each agent, based on a large number of nodes generate in the scene. At the same time, each agent will also find shortcut during its movement. Once an agent reach its current goal, the next goal will be automatically selected and assigned to it.

In challenge part, each agent will try its best to avoid other agents while finding its goal.
 
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



## Images

## Video & Time Stamp

## Art Contest

## References

* model of sled: https://assetstore.unity.com/packages/3d/characters/low-poly-winter-pack-78938
* model of ground texture: https://assetstore.unity.com/packages/2d/textures-materials/lowpoly-textures-pack-140717
* all other models: https://quaternius.com/index.html
* Camera code and the basic framework for running PRM is provided by Liam Tyler and professor Guy
