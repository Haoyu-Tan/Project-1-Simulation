//Name: Haoyu Tan
//ID#: 5677259

//import processing.opengl.*;

//#
int numObstacles = 25;
int numNodes = 500;
int numAgents = 5;

int maxNumNodes = 1000;
int maxNumObstacles = 1000;
int maxNumAgents = 10;
  
//arr
//agent
ArrayList<Agent> agents = new ArrayList<>();
//obstacle
Vec2 circlePos[] = new Vec2[maxNumObstacles];
float circleRad[] = new float[maxNumObstacles];
//node
Vec2[] nodePos = new Vec2[maxNumNodes];
float nodeRad = 25;

//Camera
Camera camera;


//Models
PShape agent;
PShape obstacle;
PShape goal;
PImage ground;

//Models sources
String[] img = {"Winter_Hardsnow.png", "Winter_ChunkyIce.png", "Winter_ce_Cracked_B.png"};

String oPath = "tree1.obj";
//String[] aPath = {"frog.obj"};
//String[] gPath = {"gift.obj"};
//String aPath = "frog.obj";
String aPath = "sled.obj";
String gPath = "gift.obj";

//size of models
float agentSF = 1.01;
float obstacleSF = 1.85;
//float obstacleSF = 1;
float nodeSF = 125;

//testing
int numCollisions = 9999; 
float pathLength = 9999;

//mode
boolean isCollider = false;
boolean isLine = true;


void setup(){
  size(1024, 768, P3D);
  camera = new Camera();
  
  //LoadModel
  obstacle = loadShape(oPath);
  agent = loadShape(aPath);
  goal = loadShape(gPath); //https://free3d.com/3d-model/gift-box-v3--783755.html
  ground = loadImage(img[0]);
  
  
  //Set Scene & run PRM
  resetScene();
  for(int i = 0; i < numAgents; i++){
    runPRM(agents.get(i));
  }
}

//update
void draw(){
  
  //draw background
  background(0, 0, 0); //Grey background
  lights();
  directionalLight(100, 100, 100, 0, -0.8,  0.6);
  
  float deltaT = 1.0 / frameRate;
  
  //update camera
  camera.Update(deltaT);
  
  
  
  for (int i = 0; i < numAgents; i++){

    Agent currA = agents.get(i);
    int count = 5;
    while (currA.curPath.size() <= 0 && count > 0){
      
      currA.resetStart(currA.pos);
      currA.resetGoal(sampleFreePos());
      runPRM(currA);
      count--;
    }
    
    if (currA.curPath.size() <= 0) currA.status = 0;
  }
  
  
  for(int i = 0; i < numAgents; i++){
    Agent curA = agents.get(i);
    curA.setVelDir();
    computeAgentForces(curA);
    //println(i,agents.get(i).agentF);
    
    curA.update(deltaT);
  }
  
  //update agents
  for (int i = 0; i < numAgents; i++){
    Agent currA = agents.get(i);
    currA.updatePosition(deltaT);
    currA.findShorterPath();
    
    
    
    if (currA.curPath.size() > 0){
      Vec2 d = (currA.pos).minus(currA.curPath.get(0)).normalized();
      float dtn = (currA.pos).distanceTo(currA.curPath.get(0));
      hitInfo hi = rayCircleListIntersect(circlePos, circleRad, numObstacles, currA.curPath.get(0), d, dtn);
      
      if (hi.hit){
        currA.resetStart(currA.pos);
        runPRM(currA);
      }
    }
    
  }
  
  
  //draw a board
  drawBoard();
  
  //draw obstacles
  drawObstacles();
  
  //draw nodes
  //drawNodes();
  
  
  drawAgents();
  
  //for test only
  if (isLine)
    drawLine();
   
  if (isCollider)
    drawTest();
  
}

//================compute update agent==============
float tH = 3;
void computeAgentForces(Agent a){
    
    //Find velocity pointing to the goal
    
    //if (a.status == 0) return;
    //a.setVelDir();
    a.agentF = new Vec2(0,0);
    
    for (int j = 0; j < numAgents; j++){
      if (j == a.aIndex) continue;
      
      Agent jAgent = agents.get(j);
      //if (jAgent.status == 0) continue;
      
      
      //jAgent.setVelDir();
      
      float ttc = computeTTC(a.pos, a.vel, nodeRad, jAgent.pos, jAgent.vel, nodeRad);
      if (ttc <= 0) continue;
      
      Vec2 A_future = a.pos.plus((a.vel).times(ttc));
      Vec2 J_future = jAgent.pos.plus((jAgent.vel).times(ttc));
      
      Vec2 rf_dir = A_future.minus(J_future).normalized();
      
      float k_avoid = (tH - ttc) / ttc;
      
      Vec2 af = rf_dir.times(k_avoid * (1 / ttc));
      
      (a.agentF).add(af);
      
      /**
      Vec2 A_future = agentPos[id].plus(agentVel[id].times(ttc));
      Vec2 J_future = agentPos[j].plus(agentVel[j].times(ttc));
      
      Vec2 rf_dir = A_future.minus(B_future).normalized();
      Vec2 af = rf_dir.times(k_avoid * (1 / ttc));
     
      acc.add(af);
      */
    }
}


//===================draw  objects==================
float sf = 1.0;
float objZ  = 25.8;
float objH = 50;
void drawObstacles(){
  //fill(255, 0, 0);
  for (int i = 0; i < numObstacles; i++){
    
    pushMatrix();
    translate(circlePos[i].x, 0, circlePos[i].y);
    //rotateX(PI / 2);
    float or = circleRad[i] - nodeRad;
    scale(or * obstacleSF);
    //obstacle.setFill(color(0, 255, 0));
    shape(obstacle);
    
    popMatrix();
    
    //test only
    /**
    pushMatrix();
    translate(circlePos[i].x, circlePos[i].y, objZ);
    drawCylinder( 20,  circleRad[i], objH);
    
    popMatrix();
    */
  }
}

//might for debug only
void drawNodes(){
  fill(0, 255, 0);
  for (int i = 0; i < numNodes; i++){
    pushMatrix();
    translate(nodePos[i].x, nodePos[i].y, objZ);
    drawCylinder( 10,  nodeRad, objH);
    popMatrix();
  }
}


void drawAgents(){

  for(int i = 0; i < numAgents; i++){
     //agents.get(i).draw();
     Agent a = agents.get(i);
     
     pushMatrix();
     translate(a.pos.x,0, a.pos.y);
     //rotateX(PI / 2);
     rotateY(PI / 2 +  a.curAngle * PI / 180);
     scale(nodeRad * agentSF);
     agent.setFill(a.col);
     shape(agent);
     popMatrix();
     
     pushMatrix();
     translate(a.goal.x,0,  a.goal.y);
     //rotateX(PI / 2);
     scale(nodeSF);
     //goal.setFill(a.col);
     shape(goal);
     noStroke();
     //fill(a.col);
     //sphere(nodeRad / 100);
     popMatrix();
     
     pushMatrix();
     translate(a.goal.x, objH,  a.goal.y);
     //noStroke();
     fill(a.col);
     sphere(nodeRad / 2);
     popMatrix();
    
     
     /**
     //for test only
     fill(255, 255, 0);
     pushMatrix();
     translate(a.pos.x, a.pos.y, objZ);
     drawCylinder( 10,  a.radii, objH);
     popMatrix();
    
     //draw goal
     fill(255, 255, 255);
     pushMatrix();
     translate(a.goal.x, a.goal.y, objZ);
     drawCylinder( 10,  a.radii, objH);
     popMatrix();
     */
  }
  //fill(0, 0, 0);
}

void drawBoard(){
  noStroke();
  fill(0, 255, 255);
  pushMatrix();
  //translate(500, 1000,0);
  beginShape();
  texture(ground);
  vertex(-50,0,  -30, 0, 0);
  vertex(1050, 0,  -30, ground.width, 0);
  vertex(1050, 0, 830, ground.width, ground.height);
  vertex(-50, 0, 830,0, ground.height);
  endShape(CLOSE);
  popMatrix();
  
  /**
  fill(0, 255, 255);
  pushMatrix();
  translate(500, 0, 1000);
  beginShape();
  vertex(-500, 0, -1000);
  vertex(5000, 0, -1000);
  vertex(5000, 0, 1000);
  vertex(-500, 0, 1000);
  endShape(CLOSE);
  popMatrix();
  */
  
}

//for test only
void drawLine(){
  
  
  //for test only
  for (int i = 0; i < numAgents; i++){
    Agent currAgent = agents.get(i);
    stroke(currAgent.col);
    strokeWeight(5);
    if (currAgent.curPath.size() == 0){
      line(currAgent.pos.x, 1, currAgent.pos.y, currAgent.goal.x, 1,currAgent.goal.y);
      return;
    }
    
    //println("draw line");
    line(currAgent.pos.x,1,currAgent.pos.y, currAgent.curPath.get(0).x, 1, currAgent.curPath.get(0).y);
    for (int j = 0; j < currAgent.curPath.size() - 1; j++){
      Vec2 curNode = currAgent.curPath.get(j);
      Vec2 nextNode = currAgent.curPath.get(j+1);
      line(curNode.x, 1, curNode.y, nextNode.x, 1, nextNode.y);
    }
    //line(currAgent.goal.x, currAgent.goal.y,  0, currAgent.curPath.get(currAgent.curPath.size()-1).x, currAgent.curPath.get(currAgent.curPath.size()-1).y, 0);
  }
  //stroke(5);
}

void drawTest(){
  //drawLine();
  
  stroke(100, 100, 100);
  strokeWeight(1);
  //draw neighbors of nodes
  //stroke(100,100,100);
  //strokeWeight(1);
  
  /**
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
  */
  
  
  //obstacle collider
  fill(255, 0, 0);
  for (int i = 0; i < numObstacles; i++){
    float or = circleRad[i] - nodeRad;
    pushMatrix();
    translate(circlePos[i].x, objZ, circlePos[i].y);
    rotateX(-PI / 2);
    
    //float or = circleRad[i] - nodeRad;
    //drawCylinder( 20,  or, objH);
    

    //println("circleRad is " + or);
    drawCylinder(20, or, objH);
    popMatrix();
  }
  
  
  //draw agent
  //for (int i = 0; i < numAgents; i++){
  //  Agent a = agents.get(i);
  //}
  
  /**
  //draw agent
  for (int i = 0; i < numAgents; i++){
     Agent a = agents.get(i);
     fill(255, 255, 0);
     pushMatrix();
     translate(a.pos.x, a.pos.y, objZ);
     drawCylinder( 10,  a.radii, objH);
     popMatrix();
    
     //draw goal
     fill(255, 255, 255);
     pushMatrix();
     translate(a.goal.x, a.goal.y, objZ);
     drawCylinder( 10,  a.radii, objH);
     popMatrix();
  }
  */
  //drawNodes();
  for(int i = 0; i < numAgents; i++){
     Agent a = agents.get(i);
     
     pushMatrix();
     fill(a.col);
     
     translate(a.pos.x,0,  a.pos.y);
     rotateX(-PI / 2);
     drawCylinder(25, nodeRad, 0);
     //circle(a.pos.x, a.pos.y, nodeRad);
     popMatrix();
  }
}

//draw simple geometry
void drawCylinder( int sides, float r, float h)
{
    float angle = 360 / sides;
    float halfHeight = h / 2;

    // draw top of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x, y, -halfHeight);
    }
    endShape(CLOSE);

    // draw bottom of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x, y, halfHeight);
    }
    endShape(CLOSE);
    
    // draw sides
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x, y, halfHeight);
        vertex( x, y, -halfHeight);    
    }
    endShape(CLOSE);

}

//==================set scene data==================
//plan path only, not responsible for connecting neighbors => tired to rename it to ranPlanPath()
//not responsible for setting the status!!!!
void runPRM(Agent currAgent){
    Vec2 startPos = currAgent.start;
    Vec2 goalPos = currAgent.goal;
    
    //println("start pos " + startPos + ", goalPos " + goalPos);
    
    /**
    //!!!!!!!!!!!!!!!!for test only, delete later!!!!!!!!!!!!!!!!!!!!!!!
    long startTime = System.nanoTime();
    currAgent.curPath = planPath2(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes, 2);
    long endTime = System.nanoTime();
    pathQuality(currAgent);
  
    println("A* -- neighbors without restricted distance!");
    println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
            " Path Len:", pathLength, " Path Segment:", currAgent.curPath.size()+1,  " Num Collisions:", numCollisions);
            
    startTime = System.nanoTime();
    currAgent.curPath = planPath2(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes, 1);   
    endTime = System.nanoTime();
    pathQuality(currAgent);
    println("BFS!");
    println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
            " Path Len:", pathLength, " Path Segment:", currAgent.curPath.size()+1,  " Num Collisions:", numCollisions);
    */
    
    ArrayList<Integer> res = new ArrayList();
    
    long startTime = System.nanoTime();
    res = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);  
   
    long endTime = System.nanoTime();
    pathQuality(res, currAgent);
    println("A* -- neighbors with restricted distance!");
    println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
            " Path Len:", pathLength, " Path Segment:", currAgent.curPath.size()+1,  " Num Collisions:", numCollisions);
            
    //if (currAgent.curPath.size() == 1 && currAgent.curPath.get(i) == -1) currAgent.curPath.clear();
    currAgent.curPath.clear();
    if ((res.size() > 0 && res.get(0) != -1) || res.size() == 0){
      for (int j = 0; j < res.size(); j++){
        currAgent.curPath.add(nodePos[res.get(j)]);
      }
      
      currAgent.curPath.add(currAgent.goal);
      
      //currAgent.setVelDir();
    }
    
    //preset a velocity direction
   
    //println("size is " + currAgent.curPath.size());
    //println("goal is " + currAgent.goal);
    //println("in plan path: " + currAgent.curPath);
    
}

//not responsible for rerun PRM!!!!!!!!!
void resetScene(){
  
  placeRandomObstacles(numObstacles);
  generateRandomNodes(numNodes, circlePos, circleRad);
  //generateFunnyNodes(numNodes, circlePos, circleRad);
  
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  
  agents.clear();
  
  //randomly pick the location of agent by default
  for (int i = 0; i < numAgents; i++){
    Vec2 tempS = generatePos();
    Vec2 tempG = generatePos();
      
    Agent currAgent = new Agent(tempS, tempG, nodeRad, numAgents);
    //Agent currAgent = new Agent(sampleFreePos(), sampleFreePos(), nodeRad, i);
    agents.add(currAgent);
  }
}

//fix
Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  }
  return randPos;
}

//fix
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
  
}

void generateAllFunnyNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
}

void generateFunnyNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  //println("in generate");
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    int cIndex = pointInCircleListFunny(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    int randTime = 0;
    boolean notAdd = false;
    while (cIndex != -1 && randTime <= 5){
      Vec2 rCenter = circleCenters[cIndex];
      float randAngle = random(360) * PI / 180;
      //println("randAngle "  + randAngle);
      Vec2 randVec = new Vec2(cos(randAngle), sin(randAngle));
      randPos = rCenter.plus(randVec.times(circleRadii[cIndex] + 0.1));
      
      cIndex = pointInCircleListFunny(circleCenters, circleRadii, numObstacles, randPos, 0);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
      
      randTime += 1;
      if (randTime > 5){
        println("greater than 5");
        i--;
        notAdd = true;
      }
    }
    
    if (!notAdd)
      nodePos[i] = randPos;
  }
  //println("after generate");
}



  
void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
    //println("before, node rad is " + circleRad[i]);
    circleRad[i] += nodeRad;
    //println("after, node rad is " + circleRad[i]);
  }
  //circleRad[0] = 30; //Make the first obstacle big
}

boolean insideAgent(Vec2 p){
  
  for (int i = 0; i < agents.size(); i++){
    if (pointInCircle(agents.get(i).pos, 2 * nodeRad, p, 0.0)){
      return true;
    }
  }
  
  return false;
}

Vec2 generatePos(){
  Vec2 p = new Vec2(0, 0);
  
  do{
    p =  sampleFreePos();
    println("inside!");
  
  }while (insideAgent(p));
  
  return p;
}

//==============handle keyboard input==============
boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    //handle reset scene
    resetScene();
    
    for (int i = 0; i < numAgents; i++){
      runPRM(agents.get(i));
    }
    return;
  }
  
  if (key == 'z'){
    if (numAgents <= maxNumAgents){
      Vec2 tempS = generatePos();
      Vec2 tempG = generatePos();
      
      Agent a = new Agent(tempS, tempG, nodeRad, numAgents);
      numAgents += 1;
      agents.add(a);
    }
  }
  
  if (key == 'x'){
    if (numAgents > 0){
      agents.remove(agents.size() - 1);
      numAgents -= 1;
    }
  }
  
  
  //mode
  if (key == 'c'){
    isCollider = !isCollider;
  }
  
  if (key == 'v'){
    isLine = !isLine;
  }
  
  //camera
  camera.HandleKeyPressed();
  
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
  
  //camera
  camera.HandleKeyReleased();
}



void mousePressed(){
  Agent currAgent = agents.get(0);
  if (mouseButton == RIGHT){
    //currAgent.start = new Vec2(mouseX, mouseY);
    Vec2 temp = new Vec2(mouseX, mouseY);
    boolean insideAnyCircle = pointInCircleList(circlePos,circleRad, numObstacles, temp ,2);
    if (!insideAnyCircle){
      currAgent.resetStart(temp);
      //println("New Start is",currAgent.start.x, currAgent.start.y);
    }
  }
  else{
    //currAgent.goal = new Vec2(mouseX, mouseY);
    Vec2 temp = new Vec2(mouseX, mouseY);
    boolean insideAnyCircle = pointInCircleList(circlePos,circleRad, numObstacles, temp ,2);
    if (!insideAnyCircle){
      currAgent.resetGoal(temp);
      currAgent.resetStart(currAgent.pos);
      //println("New Goal is",currAgent.goal.x, currAgent.goal.y);
    }
  }
  
  runPRM(currAgent);
}

//=================test PRM QUALITY ========================
void pathQuality(ArrayList<Integer> curPath, Agent a){
  Vec2 dir;
  hitInfo hit;
  float segmentLength;
  numCollisions = 9999; 
  pathLength = 9999;
  if (curPath.size() == 1 && curPath.get(0) == -1) return; //No path found  
  
  pathLength = 0; numCollisions = 0;
  
  if (curPath.size() == 0 ){ //Path found with no nodes (direct start-to-goal path)
    segmentLength = a.start.distanceTo(a.goal);
    pathLength += segmentLength;
    dir = a.goal.minus(a.start).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, a.start, dir, segmentLength);
    if (hit.hit) numCollisions += 1;
    return;
  }
  
  segmentLength = a.start.distanceTo(nodePos[curPath.get(0)]);
  pathLength += segmentLength;
  dir = nodePos[curPath.get(0)].minus(a.start).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, a.start, dir, segmentLength);
  if (hit.hit) numCollisions += 1;
  
  
  //println("in path quality, curPath size is " + a.curPath.size());
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    segmentLength = nodePos[curNode].distanceTo(nodePos[nextNode]);
    pathLength += segmentLength;
    
    dir = nodePos[nextNode].minus(nodePos[curNode]).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[curNode], dir, segmentLength);
    if (hit.hit) numCollisions += 1;
  }
  
  int lastNode = curPath.get(curPath.size()-1);
  segmentLength = nodePos[lastNode].distanceTo(a.goal);
  pathLength += segmentLength;
  dir = a.goal.minus(nodePos[lastNode]).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[lastNode], dir, segmentLength);
  if (hit.hit) numCollisions += 1;
}
