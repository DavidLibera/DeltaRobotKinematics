close all;
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
Introduction: Program Kinematics of Delta Robot
Note: 
-vectors have 3 characters ex. bB1 containing ref b, name of point B1
-scalars have 2 characters at most

%TYPE 1: Inverse Position Kinematics (IPK) Solution 
% For given parameters, wb,L,l what will the the1, the2, the3 for x,y,z 

%TYPE 2: 

%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Geometric parameters
angle = 30*pi/180; 

wb = .163732;
ub = wb/sin(angle);
sb = ub*cos(angle)*2;

up = .0723323;
wp = up*sin(angle); 
sp = up*cos(angle)*2;

L = 0.275; 
l = 0.746;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SOLUTION: DIRECT KINEMATICS 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Variables
dthe = 5*pi/180;
ithe = -25*pi/180;
fthe = 90*pi/180;

ca=1;
M=[];

%the1=115*pi/180; %:dthe:fthe
%the2=115*pi/180;
%the3=115*pi/180;

for t = 0:0.01*pi:2*pi
%for the1 =ithe:dthe:fthe
%  for the2 =ithe:dthe:fthe
%    for the3 =ithe:dthe:fthe
the1 = (45*pi/180)*sin(t);
the2 = (45*pi/180)*sin(2*t);
the3 = (45*pi/180)*sin(3*t);
  
% Vector components
bB1 = [ 0 , -wb , 0 ];
bB2 = [sqrt(3)/2*wb , 1/2*wb , 0];
bB3 = [-sqrt(3)/2*wb , 1/2*wb , 0];

pP1 = [0 , -up, 0 ];
pP2 = [sp/2 , wp, 0 ]; 
pP3 = [-sp/2 , wp, 0 ];

bL1 = [0 , -L*cos(the1), -L*sin(the1) ];  
bL2 = [sqrt(3)/2*L*cos(the2) , 1/2*L*cos(the2) , -L*sin(the2) ]; 
bL3 = [-sqrt(3)/2*L*cos(the3) , 1/2*L*cos(the3) , -L*sin(the3) ]; 

bA1 = [0 , -wb - L*cos(the1) , -L*sin(the1) ]; 
bA2 = [sqrt(3)/2*(wb+L*cos(the2)), 1/2*(wb+L*cos(the2)), -L*sin(the2) ]; 
bA3 = [-sqrt(3)/2*(wb+L*cos(the3)), 1/2*(wb+L*cos(the3)), -L*sin(the3)];

% Sphere Centers
bA1v = [0 , -wb-L*cos(the1)+up , -L*sin(the1) ];
bA2v = [sqrt(3)/2*(wb+L*cos(the2))-sp/2, 1/2*(wb+L*cos(the2))-wp, -L*sin(the2) ]; 
bA3v = [-sqrt(3)/2*(wb+L*cos(the3))+sp/2, 1/2*(wb+L*cos(the3))-wp, -L*sin(the3)];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SOLUTION: INTERSECTION OF THREE SPHERES 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pCell = num2cell(bA1v);
[x1,y1,z1] = pCell{:};

pCell = num2cell(bA2v);
[x2,y2,z2] = pCell{:};

pCell = num2cell(bA3v);
[x3,y3,z3] = pCell{:};

r1 = l; r2 = l; r3 =l;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check sphere z = z1 = z2 =z3 singularity
if(abs(z1-z2)<0.001)
  if(abs(z1-z3)<0.001)
     a = 2*(x3-x1);
     b = 2*(y3-y1);
     c = r1**2 - r3**2 - x1**2 -y1**2 +x3**2 +y3**2;
     d = 2*(x3-x2); 
     e = 2*(y3-y2);
     f = r2**2 -r3**2 -x2**2 -y2**2 +x3**2 +y3**2; 
     
     x = (c*e-b*f)/(a*e-b*d);
     y = (a*f-c*d)/(a*e-b*d);
     zn = z1; % = z2 = z3
     
     A = 1;
     B = -2*zn;
     C = zn**2 -r1**2 +(x-x1)**2 +(y-y1)**2;
     
     % Solving Az^2 + Bz + C =0
     zp = .5*(-B+sqrt(B**2 -4*C));
     zm = .5*(-B-sqrt(B**2 -4*C));
     
     if (zp<0)
       z = zp;
     elseif(zm<0)
       z = zm;
     else 
       continue;
     end 

   end

% Check singularity 
if (2*(x3-x1)*2*(y3-y2)-2*(y3-y1)*2*(x3-x2)==0)
  fprintf("Singularity 1, should not happen, because robot is symmetric"); 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute a4, a5, a6, a7
else 

a11 = 2*(x3-x1); 
a12 = 2*(y3-y1);
a13 = 2*(z3-z1); 
a21 = 2*(x3-x1);
a22 = 2*(y3-y2);
a23 = 2*(z3-z2);
b1 = (r1**2 -r3**2) +(-x1**2 -y1**2 -z1**2) +(x3**2 +y3**2 +z3**2);
b2 = (r2**2 -r3**2) +(-x2**2 -y2**2 -z2**2) +(x3**2 +y3**2 +z3**2);

a1 = a11/a13 - a21/a23;
a2 = a12/a13 - a22/a23;
a3 =b2/a23 - b1/a13;
a4 = -a2/a1;
a5 = -a3/a1; 
a6 = (-a21*a4 -a22)/a23;
a7 = (b2 -a21*a5)/a23;

a = a4**2 + 1 + a6**2; 
b = 2*a4*(a5-x1) - 2*y1 + 2*a6*(a7-z1); 
c = a5*(a5-2*x1) + a7*(a7-2*z1) + x1**2 +y1**2 + z1**2 -r1**2; 

ysol1 = (-b + sqrt(b**2 - 4*a*c))/(2*a); 
ysol2 = (-b - sqrt(b**2 - 4*a*c))/(2*a); 

xsol1 = a4*ysol1 + a5;
xsol2 = a4*ysol2 + a5;

zsol1 = a6*ysol1 + a7; 
zsol2 = a6*ysol2 + a7;

if(zsol1<0)
  x=xsol1; y=ysol1; z=zsol1;
elseif(zsol2<0)
  x=xsol2; y=ysol2; z=zsol2;
else
  continue;
end


end % if z1,z2,z3 are not equal if statement end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SOLVING FOR VECTORS (lower arm) FOR PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a = wb-up;
b = sp/2 - sqrt(3)/2*wb;
c = wp - 1/2*wb;

bl1 = [x,y+L*cos(the1)+a , z+L*sin(the1)];
bl2 = [x-sqrt(3)/2*L*cos(the2)+b , y-1/2*L*cos(the2)+c , z+L*sin(the2) ];
bl3 = [x+sqrt(3)/2*L*cos(the3)-b,y-1/2*L*cos(the3)+c,z+L*sin(the3)];

bPp = bB1+bL1+bl1-pP1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
PLOTTING 
Once x,y,z are solved can put into the below
-quiver3( starting point, vector ) 
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTTING Points for various thetas
vec =[bPp(1),bPp(2),bPp(3)];
M=[M;vec];
ca=ca+1;
%end
%end
%end
end

axis equal;
plot3(M(:,1),M(:,2),M(:,3),'bo-');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTTING DELTA ROBOT CONFIGURATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Arm 1 
%h = quiver3(0,0,0,  bB1(1),bB1(2),bB1(3)); set(h,"linewidth",1.2,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bB1(1),bB1(2),bB1(3), bL1(1),bL1(2),bL1(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'green');
hold on;
h = quiver3(bA1(1),bA1(2),bA1(3),bl1(1),bl1(2),bl1(3));set(h,"linewidth",2,"showarrowhead",'off',"color",'red');
hold on;
 
%Arm 2 

%h = quiver3(0,0,0,  bB2(1),bB2(2),bB2(3));set(h,"linewidth",1.2,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bB2(1),bB2(2),bB2(3), bL2(1),bL2(2),bL2(3));set(h,"linewidth",2,"showarrowhead",'off',"color",'green');
grid on;
h = quiver3(bA2(1),bA2(2),bA2(3),bl2(1),bl2(2),bl2(3));set(h,"linewidth",2,"showarrowhead",'off',"color",'red');
hold on;

%Arm 3 
%h = quiver3(0,0,0,  bB3(1),bB3(2),bB3(3));set(h,"linewidth",1.2,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bB3(1),bB3(2),bB3(3), bL3(1),bL3(2),bL3(3)); set(h,"linewidth",2,"showarrowhead",'off',"color",'green');
grid on;
h = quiver3(bA3(1),bA3(2),bA3(3),bl3(1),bl3(2),bl3(3));set(h,"linewidth",2,"showarrowhead",'off',"color",'red');
hold on;

%Tool plate
%h = quiver3(bPp(1),bPp(2),bPp(3),  pP1(1),pP1(2),pP1(3)); set(h,"linewidth",1.2,"showarrowhead",'off',"color",'black');
hold on;
%h = quiver3(bPp(1),bPp(2),bPp(3),  pP2(1),pP2(2),pP2(3)); set(h,"linewidth",1.2,"showarrowhead",'off',"color",'black');
hold on;
%h = quiver3(bPp(1),bPp(2),bPp(3),  pP3(1),pP3(2),pP3(3)); set(h,"linewidth",1.2,"showarrowhead",'off',"color",'black');
hold on; 

%Tool plate
h = quiver3(bPp(1)+pP1(1),...
            bPp(2)+pP1(2),...
            bPp(3)+pP1(3),...
            -pP1(1)+pP2(1),...
            -pP1(2)+pP2(2),...
            -pP1(3)+pP2(3)); 
set(h,"linewidth",4,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bPp(1)+pP1(1),...
            bPp(2)+pP1(2),...
            bPp(3)+pP1(3),...
            -pP1(1)+pP3(1),...
            -pP1(2)+pP3(2),...
            -pP1(3)+pP3(3)); 
set(h,"linewidth",4,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bPp(1)+pP3(1),...
            bPp(2)+pP3(2),...
            bPp(3)+pP3(3),...
            -pP3(1)+pP2(1),...
            -pP3(2)+pP2(2),...
            -pP3(3)+pP2(3)); 
set(h,"linewidth",4,"showarrowhead",'off',"color",'black');
hold on;


%Base plate illustration 
h = quiver3(bB1(1),bB1(2),bB1(3),-bB1(1)+bB2(1),-bB1(2)+bB2(2),-bB1(3)+bB2(3)); set(h,"linewidth",4,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bB1(1),bB1(2),bB1(3),-bB1(1)+bB3(1),-bB1(2)+bB3(2),-bB1(3)+bB3(3)); set(h,"linewidth",4,"showarrowhead",'off',"color",'black');
hold on;
h = quiver3(bB2(1),bB2(2),bB2(3),-bB2(1)+bB3(1),-bB2(2)+bB3(2),-bB2(3)+bB3(3)); set(h,"linewidth",4,"showarrowhead",'off',"color",'black');
hold on;

axis equal;
view([45,45,45])
%saveas(gcf,'test1.png');
