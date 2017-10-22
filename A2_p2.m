clear all 
close all
a=0;
R1 = [1 0 0 0; 
      0 cos(a) sin(a) 0;
      0 -sin(a) cos(a) 0; 
      0 0 0 1];
angY=0;
R2 = [cos(angY) 0 sin(angY) 0;
      0 1 0 0;
      -sin(angY) 0 cos(angY) 0;
      0 0 0 1];
      
angZ = 90*(pi/180);   
R3 = [cos(angZ) sin(angZ) 0 0;
      -sin(angZ) cos(angZ) 0 0;
      0 0 1 0;
      0 0 0 1];  #CCW matrix 

p1=[1 0 0 1];

#h = quiver3(0,0,0, p1(1),p1(2),p1(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'green');
#hold on;     
      
#p1a = p1*R3;
  
#h = quiver3(0,0,0, p1a(1),p1a(2),p1a(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'red');
#hold on; 
#xlabel('x'); ylabel('y'); zlabel('z');  
      
#break

# Define line with two points

p1 = [1 1 1 1];
p2 = [5 5 10 1];



# 1 Translate line to origin

T1 = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      -p1(1) -p1(2) -p1(3) 1];

p1a = p1*T1; 
p2a = p2*T1;

#h = quiver3(p1(1),p1(2),p1(3), p2(1),p2(2),p2(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'red');
#hold on; 
#h = quiver3(p1new(1),p1new(2),p1new(3), p2new(1),p2new(2),p2new(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'blue');
#hold on;

# 2 Rotate line to xz plane (required: angle a, depends on vector coord) 

vec = [p2a(1)-p1a(1) p2a(2)-p1a(2) p2a(3)-p1a(3)];
a = atan(vec(3)/vec(2));  #angle about x-axis
#a = 90*(pi/180);  
R1 = [1 0 0 0; 
      0 cos(a) sin(a) 0;
      0 -sin(a) cos(a) 0; 
      0 0 0 1];
      
p1b = p1a*R1;
p2b = p2a*R1;

#h = quiver3(p1b(1),p1b(2),p1b(3), p2b(1),p2b(2),p2b(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'black');
#hold on;
#xlabel('x'); ylabel('y'); zlabel('z');
#view([0 90 0]);
#axis equal

# 3 Rotate about y 

vec = [p2b(1)-p1b(1) p2b(2)-p1b(2) p2b(3)-p1b(3)]; 
#vec is on x-z so use vec(3) and vec(1) for atan
angY = atan(vec(1)/vec(3)); 

#NOTE: important!!! need to use theta between z and vec, above is correct!

R2 = [cos(angY) 0 -sin(angY) 0;
      0 1 0 0;
      sin(angY) 0 cos(angY) 0;
      0 0 0 1];
      
p1c=p1b*R2;
p2c=p2b*R2;

h = quiver3(p1c(1),p1c(2),p1c(3), p2c(1),p2c(2),p2c(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'black');
hold on;
xlabel('x'); ylabel('y'); zlabel('z');
#view([0 90 0]);
axis equal





