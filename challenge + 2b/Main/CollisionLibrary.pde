//Name: Haoyu Tan
//ID#: 5677259


/////////
// Point Intersection Tests
/////////

//Returns true iff the point, pointPos, is inside the box defined by boxTopLeft, boxW, and boxH
boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  Vec2 dist =pointPos.minus(boxTopLeft);
  if (dist.x > 0 && dist.y > 0 && dist.x < boxW && dist.y < boxH){
    return true;
  }
  return false;
}

//Returns true iff the point, pointPos, is inside a circle defined by center and radius r
// If eps is non-zero, count the point as "inside" the circle if the point is outside, but within the distance eps of the edge
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos, float eps){
  //enlarge the r to avoid hit the agent
  //r += nodeRad;
  float dist = pointPos.distanceTo(center);
  if (dist - r < eps) return true;
  
  return false;
}



//Returns true if the point is inside any of the circles defined by the list of centers,"centers", and corisponding radii, "radii".
// As above, count point within "eps" of the circle as inside the circle
//Only check the first "numObstacles" circles.
boolean pointInCircleList(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  //if (numObstacles > centers.length ||numObstacles > radii.length) return false;
  
  for (int i = 0; i < numObstacles; i++){
    if (pointInCircle(centers[i], radii[i], pointPos, eps)) return true;
  }
  return false;
}

int pointInCircleListFunny(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  
  for (int i = 0; i < numObstacles; i++){
    if (pointInCircle(centers[i], radii[i], pointPos, eps)) return i;
  }
  return -1;
}


/////////
// Ray Intersection Tests
/////////

//This struct is used for ray-obstaclce intersection.
//It store both if there is a collision, and how far away it is (int terms of distance allong the ray)
class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}



//Constuct a hitInfo that records if and when the ray starting at "ray_start" and going in the direction "ray_dir"
// hits the circle centered at "center", with a radius "radius".
//If the collision is further away than "max_t" don't count it as a collision.
//You may assume that "ray_dir" is always normalized
hitInfo rayCircleIntersect(Vec2 center, float radius, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  //enlarge the radius to avoid hit the agent
  //radius += nodeRad;
  
  //Vec2 velDir = ray_dir.minus(new Vec2(0, 0));
  float time = rayCircleIntersectTime(center, radius, ray_start, ray_dir);
  if (time > 0 && time <= max_t){
     hit.hit = true;
     hit.t = time;
  }
  return hit;
}

//Constuct a hitInfo that records if and when the ray starting at "ray_start" and going in the direction "ray_dir"
// hits any of the circles defined by the list of centers,"centers", and corisponding radii, "radii"
//If the collision is further away than "max_t" don't count it as a collision.
//You may assume that "ray_dir" is always normalized
//Only check the first "numObstacles" circles.
hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii, int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  for (int i = 0; i < numObstacles; i++){
    hitInfo tempHit = rayCircleIntersect(centers[i], radii[i], l_start, l_dir, max_t);
    if (tempHit.hit && tempHit.t <= hit.t) {
      //print("i is: ", i , "\n");
      //print("hit time is ", hit.t, "\n");
      hit = tempHit;
    }
  }
  
  return hit;
}

float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  //rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir)
  float r_total = radius1 + radius2;
  Vec2 l_dir = vel2.minus(vel1);
  float time_t = rayCircleIntersectTime(pos1, r_total, pos2, l_dir);
  
  return time_t;
}



//This helper function is from our attendence practice
float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 
  Vec2 toCircle = center.minus(l_start);
 
  float a = l_dir.lengthSqr();
  float b = -2*dot(l_dir,toCircle);
  float c = toCircle.lengthSqr() - (r*r);
 
  float d = b*b - 4*a*c; 
 
  if (d >=0 ){
    float t = (-b - sqrt(d))/(2*a); 
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; 
}
