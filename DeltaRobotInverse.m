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
%{
angle = 30*pi/180; 

wb = .163732;
ub = wb/sin(angle);
sb = ub*cos(angle)*2;

up = .0723323;
wp = up*sin(angle); 
sp = up*cos(angle)*2;

L = 0.275; 
l = 0.746;
%}
angle = 30*pi/180; 

sb = 0.567;
ub = sb/(cos(angle)*2);
wb = ub*sin(angle);

sp = 0.076;
up = sp/(cos(angle)*2);
wp = up*sin(angle); 


L = 0.524; 
l = 1.244;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SOLUTION: DIRECT KINEMATICS 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ca=1;
M=[];

for t = 0:pi/50:2*pi
  
x = 0.5*cos(t);
y = 0.5*sin(t);
z = -.7;%0.2*sin(2*t)-1; 

%x=0; 
%y=0;
%z=-0.9;

a = wb-up;
b = sp/2-sqrt(3)/2*wb;
c = wp -1/2*wb;

E1 = 2*L*(y+a);
F1 = 2*z*L;
G1 = x*x + y*y +z*z +a*a +L*L +2*y*a -l*l;

E2 = -L*(sqrt(3)*(x+b)+y+c);
F2 = 2*z*L;
G2 = x*x + y*y +z*z +b*b+c*c+L*L +2*(x*b+y*c) -l*l;

E3 = L*(sqrt(3)*(x-b)-y-c);
F3 = 2*z*L;
G3 = x*x + y*y +z*z +b*b+c*c +L*L +2*(-x*b+y*c) -l*l;



t11= (-F1+sqrt(E1*E1+F1*F1-G1*G1))/(G1-E1);
t12= (-F1-sqrt(E1*E1+F1*F1-G1*G1))/(G1-E1);
the11 = 2*atan(t11)*(180/pi);
the12 = 2*atan(t12)*(180/pi);

t21= (-F2+sqrt(E2*E2+F2*F2-G2*G2))/(G2-E2);
t22= (-F2-sqrt(E2*E2+F2*F2-G2*G2))/(G2-E2);
the21 = 2*atan(t21)*(180/pi);
the22 = 2*atan(t22)*(180/pi);

t31= (-F3+sqrt(E3*E3+F3*F3-G3*G3))/(G3-E3);
t32= (-F3-sqrt(E3*E3+F3*F3-G3*G3))/(G3-E3);
the31 = 2*atan(t31)*(180/pi);
the32 = 2*atan(t32)*(180/pi);

% Converting all values to positive angles 
%if(the11<0); the11 = (360+the11)*pi/180; end;
%if(the12<0); the12 = (360+the12)*pi/180; end;
%if(the21<0); the21 = (360+the21)*pi/180; end;
%if(the22<0); the22 = (360+the22)*pi/180; end;
%if(the31<0); the31 = (360+the31)*pi/180; end;
%if(the32<0); the32 = (360+the32)*pi/180; end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAWING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
the1 = mod(the11,2*pi);
the2 = mod(the21,2*pi);
the3 = mod(the31,2*pi);
%%%%%%%%%%FAILED
%{
if ( ((the11>(90/180*pi)) && (the11<(270/180*pi)) ))% || (the11<0)) )
  the1 = the12;
elseif ( ((the12>(90/180*pi)) && (the12<(270/180*pi)) ))
  the1 = the11;
else
  the1 = 0;
end

if ( ((the21>(90/180*pi)) && (the21<(270/180*pi)) ))% || (the11<0)) )
  the2 = the22;
elseif ( ((the22>(90/180*pi)) && (the22<(270/180*pi)) ))
  the2 = the21;
else
  the2 = 0;
end

if ( ((the31>(90/180*pi)) && (the31<(270/180*pi)) ))% || (the11<0)) )
  the3 = the32;
elseif ( ((the32>(90/180*pi)) && (the32<(270/180*pi)) ))
  the3 = the31;
else
  the3 = 0;
end
%}

%%%% FAILED 

if ( (the11>0) && (the11<pi/2) ) || ( (the11>270/180*pi) && (the11< pi) ) 
  the1 = the11;
elseif ( (the11<0) && (the11>-pi/2) ) || ((the11<-270/180*pi) && (the11>-pi) ) 
  the1 = the11; 
else
  the1 = the12;
end;

if ( (the21>0) && (the21<pi/2) )  || ( (the21>270/180*pi) && (the21< pi) ) 
  the2 = the21;
elseif ( (the21<0) && (the21>-pi/2) )  || ( (the21<-270/180*pi) && (the21>-pi) ) 
  the2 = the21; 
else
  the2 = the22;
end;

if ( (the31>0) && (the31<pi/2) ) || ( (the31>270/180*pi) && (the31< pi) ) 
  the3 = the31;
elseif ( (the11<0) && (the11>-pi/2) ) || ( (the11<-270/180*pi) && (the11>-pi) ) 
  the3 = the31; 
else
  the3 = the32;
end;


vec =[x,y,z,the1,the2,the3,t];
M=[M;vec];
ca=ca+1;

end

figure(1)
axis equal;
plot3(M(:,1),M(:,2),M(:,3),'bo-');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;

%ending loop 
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ANGLE PLOTS 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%the1 = mod(M(:,4)*(180/pi),360);
%the2 = mod(M(:,5)*(180/pi),360);
%the3 = mod(M(:,6)*(180/pi),360);

the1 = M(:,4);%*(180/pi);
the2 = M(:,5);%*(180/pi);
the3 = M(:,6);%*(180/pi);


figure(2)
subplot(3,1,1)
plot(M(:,7),the1);
ylabel('THETA 1');
subplot(3,1,2)
plot(M(:,7),the2);
ylabel('THETA 2');
subplot(3,1,3)
plot(M(:,7),the3);
ylabel('THETA 3');


break;


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

bl1 = [x,y+L*cos(the1)+a , z+L*sin(the1)];
bl2 = [x-sqrt(3)/2*L*cos(the2)+b , y-1/2*L*cos(the2)+c , z+L*sin(the2) ];
bl3 = [x+sqrt(3)/2*L*cos(the3)-b,y-1/2*L*cos(the3)+c,z+L*sin(the3)];

bPp = bB1+bL1+bl1-pP1;

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

