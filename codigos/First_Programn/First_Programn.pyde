def setup():
    size(640, 480)

def draw():
    if  mousePressed:
        fill(20,30,255)
    else:
        fill(255)
    ellipse(mouseX, mouseY, 80, 80)
