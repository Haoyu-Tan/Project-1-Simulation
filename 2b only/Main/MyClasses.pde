class Agent{
  //default information
  public Vec2 start;
  public Vec2 goal;
  public float radii;
  color col;
  
  
  //real time information
  public Vec2 pos;
  public float speed;
  public Vec2 dir;
  public int status; //0 -- idle;  1 -- move;  2 -- turn??
  Vec2 nextNode;
  
  float angleToGoal;
  float curAngle;
  float angSpeed;
 
  
  //ArrayList<Integer> curPath;
  ArrayList<Vec2> curPath;
  
  public Agent(Vec2 s, Vec2 g, float r){
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
    
    angleToGoal = 0;
    curAngle = 0;  
    angSpeed = 200;
  }
  
  void resetStart(Vec2 newPos){
    start = newPos;
    pos = start;
  }
  
  void resetGoal(Vec2 newPos){
    goal = newPos;
  }
  
  void update(float dt){
    if (status == 0){
      //idle
      if (curPath.size() <= 0)
        return;
      status = 1;
      
      
    }
    if (status == 1){
      // MOVE
      nextNode = curPath.get(0);
      //println("next node: " + nextNode);
      float moveD = speed * dt;
      float distToNode = pos.distanceTo(nextNode);
      
      if (moveD >= distToNode){
        //reach the goal
        pos = nextNode;
        curPath.remove(0);
        if (curPath.size() == 0){
          status = 0;
          return;
        }
        
        moveD -= distToNode;
        
      }
      nextNode = curPath.get(0);
      dir = nextNode.minus(pos).normalized();
      pos = pos.plus(dir.times(moveD));
      
      //Test if agent is able to find a shorcut path from here
      //findShorterPath();
     
      
      //HANDLE AGENT ROTEATE
      //
      //println(cross(pos.normalized(),nextNode.normalized()));
    
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
        return;
      }
     
      if (diff < 0) turnAngle *= -1;
      curAngle += turnAngle;
      
      if (curAngle > 180) curAngle -= 360;
      else if (curAngle < -180) curAngle += 360;
      //println("curAngle is " + curAngle);
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
    
  }
  
  /**
  void draw(){
    println("my start is " + start.x + ", " + start.y);
    println("my goal is " + goal.x + ", "  + goal.y);
    //draw start
    //yellow temporary
    fill(255, 255, 0);
    pushMatrix();
    translate(pos.x, pos.y, objZ);
    drawCylinder( 10,  radii, objH);
    popMatrix();
    
    //draw goal
    fill(255, 255, 255);
    pushMatrix();
    translate(goal.x, goal.y, objZ);
    drawCylinder( 10,  radii, objH);
    popMatrix();
  }
  
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
  */
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
