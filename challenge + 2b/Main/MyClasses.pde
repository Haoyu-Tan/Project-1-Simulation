class Agent{
  //default information
  public Vec2 start;
  public Vec2 goal;
  public float radii;
  color col;
  int aIndex;
  
  
  //real time information
  public Vec2 pos;
  public float speed;
  public Vec2 dir;
  public int status; //0 -- idle;  1 -- move;  2 -- turn??
  public Vec2 agentF;
  public Vec2 vel;
  Vec2 nextNode;
  
  //real time rotate info
  float angleToGoal;
  float curAngle;
  float angSpeed;
 
  
  //ArrayList<Integer> curPath;
  ArrayList<Vec2> curPath;
  
  public Agent(Vec2 s, Vec2 g, float r, int ind){
    this.start = s;
    this.goal = g;
    
    //speed & dir
    this.pos = start;
    this.speed = 100.0;
    this.dir = new Vec2(0, 0);
    this.radii = r;
   
    this.col = color(int(random(255)), int(random(255)), int(random(255)));
    this.status = 0;
    
    this.nextNode = new Vec2(0, 0);
    curPath = new ArrayList();
    
    this.agentF = new Vec2(0, 0);
    this.vel = new Vec2(0, 0);
    
    angleToGoal = 0;
    curAngle = 0;  
    angSpeed = 200;
    
    aIndex = ind;
  }
  
  void resetStart(Vec2 newPos){
    start = newPos;
    pos = start;
  }
  
  void resetGoal(Vec2 newPos){
    goal = newPos;
  }
  
  void setVelDir(){
    if (curPath.size() > 0){
      this.vel = curPath.get(0).minus(pos).normalized();
      this.vel.mul(speed);
    }
    else this.vel = new Vec2(0, 0);
  }
  
  void avoidObstacle(){
    for (int i = 0; i < numObstacles; i++){
      if (pos.distanceTo(circlePos[i]) < circleRad[i]){
        Vec2 norm = (pos.minus(circlePos[i])).normalized();
        pos = circlePos[i].plus(norm.times(circleRad[i]).times(1.01));
        return;
        
      }
    }
  }
  
  void updateVel(float dt){
    
      ///////
      //println("old",vel);
      //println("AgentF is ",agentF);
      vel.add(agentF.times(dt));
      //agentF = new Vec2(0,0);
      //println("new",vel);
      //avoid a large speed
      if (vel.length() > 1.5 * speed) vel.setToLength(1.5 * speed);
     
      //update velocity
    
   
  }
  
  void updatePosition(float dt){
    if (curPath.size() > 0){
      Vec2 currNode = curPath.get(0);
      boolean reachNode = false;
      int nodeRegion = 5;
      
      float distToCurrNode = pos.distanceTo(currNode);
      //already inside current node region
      if (distToCurrNode <= nodeRegion) reachNode = true;
      
      float hitT = rayCircleIntersectTime(currNode, nodeRegion, pos, vel.normalized());
      Vec2 deltaPos = vel.times(dt);
      if (hitT >= 0 && hitT <= deltaPos.length()) reachNode = true;
      if (reachNode) curPath.remove(0);
      
      pos.add(deltaPos);
      
      avoidObstacle();
    }
  }
  
  void update(float dt){
    if (status == 0){
      //idle
      if (curPath.size() <= 0)
        return;
      status = 1;
      
      
    }
    if (status == 1){
      
      // Update Velocity
      updateVel(dt);
      
     
      //Recompute DIR!!!!!! It is not normalized!!!!
      
      //HANDLE AGENT ROTEATE
      dir = vel.normalized();
      float sine = cross(new Vec2(1, 0), dir);
      float sign = sine > 0 ? 1 : -1;
      angleToGoal = acos(dot(dir, new Vec2(1, 0))) * sign;
      angleToGoal = angleToGoal * 180 / PI;
     
      //curAngle = angleToGoal;
      //println("sin is " + sine);
      //println("angle to goal is " + angleToGoal);
      //println("angle is " + curAngle);
      
      float diff = angleToGoal - curAngle;
      
      if (diff < - 180) diff += 360;
      else if (diff > 180) diff -= 360;
      
      float turnAngle = dt * angSpeed;
      
      if (turnAngle >= abs(diff)){
        curAngle = angleToGoal;
        //return;
      }
      else{
        if (diff < 0) turnAngle *= -1;
        curAngle += turnAngle;
        
        if (curAngle > 180) curAngle -= 360;
        else if (curAngle < -180) curAngle += 360;
      }
      
    }
    
  }
  
  
  
  void findShorterPath(){
    int maxIndex = -1;
    for (int i = curPath.size() - 1; i > 0 ; i--){
         Vec2 tempPos = curPath.get(i);
         Vec2 tempDir = tempPos.minus(pos).normalized();
         float tempDist = pos.distanceTo(tempPos);
         hitInfo tempInfo = rayCircleListIntersect(circlePos, circleRad, numObstacles, pos, tempDir, tempDist);
         if (!tempInfo.hit){
           maxIndex = i;
           //println("maxIndex is " + i);
           //if (maxIndex == curPath.size() - 1) println("everything before goal is gonna remove!!!");
           break;
         }
      }
      
      
      for (int i = 0; i < maxIndex; i++){
        curPath.remove(0);
      }
      
      //reset the velocity dir
      //setVelDir();
    
  }
}

/**
class Obstacle{
  public Vec2 center;
  public float rad;
  
  
  public Obstacle(Vec2 c, float r ){
    this.center = c;
    this.rad = r;
  }
}


class Node{
  public Vec2 pos;
  Node(){
  }
}
*/
