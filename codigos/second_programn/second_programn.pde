float r;

// Angle and angular velocity, accleration
float theta;
float theta_vel;
float theta_acc;

void setup() {
  size(640, 360);
  
  // Initialize all values
  r = height * 0.45;
  theta = 50;
  theta_vel = 0;
  theta_acc = 0.0001;
}
boolean back = false;
void draw() {
  
  background(0);
  
  // Translate the origin point to the center of the screen
  translate(width/2, height/2);
  
  // Convert polar to cartesian
  float x = r * cos(theta);
  float y = r * sin(theta);
  
  // Draw the ellipse at the cartesian coordinate
  ellipseMode(CENTER);
  noStroke();
  fill(200);
  ellipse(x, y, 32, 32);
  ellipse(0, 0, 32, 32);
  stroke(153);
  line(0,0,x,y);
  if(back)theta += 0.01;
  else theta -= 0.01;
    
  // Apply acceleration and velocity to angle (r remains static in this example)
  theta_vel += theta_acc;
  //theta += theta_vel;
  
  stroke(255);
  point(120,50);
  println(x,y);
  
  if(x<=-r*0.9){
    line(0,0,x,y);
    back = true;
  }
  if(x>=r*0.9)back = false;

}
