float r;
float escala = 80;
// Angle and angular velocity, accleration
float theta;
float theta_vel;
float theta_acc;
float posX,posY;
void setup() {
  size(640, 640);
  
  // Initialize all values
  r = height * 0.45;
  theta = 50;
  theta_vel = 0;
  theta_acc = 0.0001;
  posX = 1;
  posY = 0;
  background(0);
  translate(width/2, height/2);
  stroke(153);
  line(0,height/2,0,-height/2);
  stroke(153,0,0);
  line(-width/2,0,width/2,0);// line horizontal
  
  
  
  
}
boolean back = false;
void draw() {
  translate(width/2, height/2);
  
  
  float distancia = sqrt(pow(posX-0,2) + pow(posY-0,2)); //<>//
  println(distancia); //<>//
  println(PI*(distancia-1)); //<>//
  println(asin(PI*(distancia-1))); //<>//
  posY = asin(PI*(distancia-1))*posX; //<>//
  
  
  println(posX,posY); //<>//
  println(distancia); //<>//
  fill(0,255,0);
  ellipse(posX,posY, 10, 10);
  ellipse(escala*1,escala*0, 10, 10);
  
  
  posX -= 1;
  //translate(width/2, height/2);
  //// Translate the origin point to the center of the screen
  
  
  //// Convert polar to cartesian
  //float x = r * cos(theta);
  //float y = r * sin(theta);
  
  //// Draw the ellipse at the cartesian coordinate
  //ellipseMode(CENTER);
  //noStroke();
  //fill(200);
  //ellipse(x, y, 32, 32);
  //ellipse(0, 0, 32, 32);
  //stroke(153);
  //line(0,0,x,y);
  //if(back)theta += 0.01;
  //else theta -= 0.01;
    
  //// Apply acceleration and velocity to angle (r remains static in this example)
  //theta_vel += theta_acc;
  ////theta += theta_vel;
  
  //stroke(255);
  //point(120,50);
  //println(x,y);
  
  //if(x<=-r*0.9){
  //  line(0,0,x,y);
  //  back = true;
  //}
  //if(x>=r*0.9)back = false;

}
