//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The function connectNeighbors() will always be called before planPath()
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of the positions in the nodePos array will ever be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file (PRM.pde) for compatabilty reasons.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it uses BFS which will not provide the shortest path.
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment. This file is
// intended to illustrate the basic set-up for the assignmtent, don't assume 
// this example funcationality is correct and end up copying it's mistakes!).



//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
//int maxTotalNodes = maxNumNodes + 2;
int minSide = height > width ? height : width;
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node
//Boolean[] fringed = new Boolean[maxTotalNodes];
float[] path_cost = new float[maxNumNodes];
float[] f_cost = new float[maxNumNodes];
ArrayList<Integer> start_neighbors = new ArrayList();
ArrayList<Integer> goal_neighbors = new ArrayList();

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}




//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes, Vec2[] centers, float[] radii, int numObstacles){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      Vec2 dir = nodePos[i].minus(point).normalized();
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, point, dir, dist);
      if (!circleListCheck.hit){
        closestID = i;
        minDist = dist;
      }
    }
  }
  return closestID;
}


ArrayList<Integer> findNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, Vec2 p){
  ArrayList<Integer> nlist = new ArrayList<Integer>();
  float maxDist = 0;
  float minDist = 0;
  

  while (nlist.isEmpty() && maxDist < minSide){
    minDist = maxDist;
    maxDist += 350;
    
    //println("min distance " + minDist);
    //println("max distance " + maxDist);
    for(int i = 0; i < numNodes; i++){
        float distBetween = nodePos[i].distanceTo(p);
        if (distBetween <= minDist || distBetween > maxDist) continue;
        Vec2 dir = nodePos[i].minus(p).normalized();
        //println("node " + i);
        hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, p, dir, distBetween);
        if (!circleListCheck.hit){
          nlist.add(i);
        }
    }
  }  
  
  return nlist;
}



void findNeighborsSilly(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, Vec2 startPos, Vec2 goalPos){
    start_neighbors = new ArrayList();
    goal_neighbors = new ArrayList();
    for (int i = 0; i < numNodes; i++){
      float distBetweenS = nodePos[i].distanceTo(startPos);
      Vec2 dirS = nodePos[i].minus(startPos).normalized();
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, startPos, dirS, distBetweenS);
      if (!circleListCheck.hit){
        start_neighbors.add(i);
      }
      

      float distBetweenG = nodePos[i].distanceTo(goalPos);
      Vec2 dirG = goalPos.minus(nodePos[i]).normalized();
      hitInfo circleListCheckG = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dirG, distBetweenG);
      if (!circleListCheckG.hit){
        goal_neighbors.add(i);
      }
      
    }
}


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!modify this arg-list before submit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!delete the BOOLEAN before submit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

ArrayList<Integer> planPath2(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, int astar){
  ArrayList<Integer> path = new ArrayList();
  
  if (astar == 0){
    //findNeighborsSilly(centers, radii, numObstacles, nodePos, numNodes, startPos, goalPos);
    start_neighbors = findNeighbors(centers, radii, numObstacles, nodePos, numNodes, startPos);
    goal_neighbors = findNeighbors(centers, radii, numObstacles, nodePos, numNodes, goalPos);
    //println(start_neighbors);
    //println("goal neighbor is " + goal_neighbors);
    
    path = runAStar(nodePos, numNodes, startPos, goalPos);
  }
  
  else if (astar == 1){
    int startID = closestNode(startPos, nodePos, numNodes, centers, radii, numObstacles);
    int goalID = closestNode(goalPos, nodePos, numNodes, centers, radii, numObstacles);
    
    path = runBFS(nodePos, numNodes, startID, goalID);
  }
  
  else if (astar == 2){
    findNeighborsSilly(centers, radii, numObstacles, nodePos, numNodes, startPos, goalPos);
    path = runAStar(nodePos, numNodes, startPos, goalPos);
  }
  
  return path;
}


ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  
  ArrayList<Integer> path = new ArrayList();
  
  Vec2 sgDir = goalPos.minus(startPos).normalized();
  float sgDist = startPos.distanceTo(goalPos);
  hitInfo sgHit = rayCircleListIntersect(centers, radii, numObstacles, startPos, sgDir, sgDist);
  if (!sgHit.hit){
    return path;
  }
    
  findNeighborsSilly(centers, radii, numObstacles, nodePos, numNodes, startPos, goalPos);
    
  path = runAStar(nodePos, numNodes, startPos, goalPos);

  
  return path;
}

float HDist(Vec2 nodePos, Vec2 goalPos){
  return nodePos.distanceTo(goalPos);
}


//return the index to insert in fringe
//return -1 if the node appends to the end(has largest value) or the list is empty
int insertFringe(ArrayList<Integer> fringe, int node){
  int res = -1;
  for (int curr = 0; curr < fringe.size(); curr++){
    int currNode = fringe.get(curr);
    if (f_cost[node] < f_cost[currNode]){
      res = curr;
      break;
    }
  }
  
  return res;
}

//A*
ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, Vec2 startPos, Vec2 goalPos){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  int goalID = -1;
  
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
    path_cost[i] = 0.0;
    f_cost[i] = 0.0;
    //fringed[i] = false;
  }

  //println("\nBeginning Search");
  
  for (int startNeighbor : start_neighbors){
    //println("neighbor is " + startNeighbor);
   
    visited[startNeighbor] = true;
    path_cost[startNeighbor] = nodePos[startNeighbor].distanceTo(startPos);
    f_cost[startNeighbor] = path_cost[startNeighbor] + HDist(nodePos[startNeighbor], goalPos);
     //println("path cost is " + path_cost[startNeighbor]);
     //println("hdist is " + f_cost[startNeighbor]);
    int index = insertFringe(fringe, startNeighbor);
    if (index == -1){
      fringe.add(startNeighbor);
      continue;
    }
    fringe.add(index, startNeighbor);
  }
  
  //println(fringe.size());
  
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (goal_neighbors.contains(currentNode)){
      //println("Goal found!");
      goalID = currentNode;
      break;
    }
    
    //fringed[currentNode] = true;
    
    for (int i = 0; i < neighbors[currentNode].size(); i++){
        int neighborNode = neighbors[currentNode].get(i);
        //println("neighbor node " + neighborNode);
        //println("visited? " + visited[neighborNode]);
     
        if (!visited[neighborNode]){
          
          visited[neighborNode] = true;
          parent[neighborNode] = currentNode;
          
          //calculate f(x) = g(x) + h(x)
          float g_cost = 0;
         
          g_cost = path_cost[currentNode] + nodePos[neighborNode].distanceTo(nodePos[currentNode]);
          path_cost[neighborNode] = g_cost;
          f_cost[neighborNode] = g_cost + HDist(nodePos[neighborNode], goalPos);
        
          //find the place to insert
          int index = insertFringe(fringe, neighborNode);
          if (index == -1){
            fringe.add(neighborNode);
            continue;
          }
          fringe.add(index, neighborNode);

        }
        
       
    }
    
  }
 
  if (fringe.size() == 0 && goalID == -1){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  return path;
}


//BFS (Breadth First Search)
ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}
