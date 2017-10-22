clear all

% A1 problem

%% Setup 

%allowable length and theta iterations
LMAX = 4; 
thetaMAX = acos(1/LMAX);

% Geometry
% 0 < theta < 90 
c=1;
for theta = 0: thetaMAX/20 : thetaMAX

%truss
L = 1/cos(theta);
diamO = 50;
diamI = 45;
A = pi*(diamO-diamI)^2/4; 
I = pi*(diamO^4-diamI^4)/64;

% Loads 
m = 200; 
F = m*9.8;

%% Computing forces and stresses along beam 
R = 0.5*F; 

% forces at end (point of interest)
FS = R*cos(theta);
FN = R*sin(theta);
MB = R*cos(theta)*L*1000;  % L is meters -> mm 

% stresses
tau = FS/A;
sigmaN = FN/A;
sigmaB = MB*diamO/I;  % sigmaB varies with distant from axis y=diamO here

% stresse on element
sigmaX = sigmaN + sigmaB;
sigmaY = 0;
tauXY = tau; 

% principle stresses
sigma1 = (sigmaX+sigmaY)/2 + sqrt(((sigmaX-sigmaY)/2)^2 + tauXY^2);
sigma2 = (sigmaX+sigmaY)/2 - sqrt(((sigmaX-sigmaY)/2)^2 + tauXY^2);

% yield criterion

tauMAX = 1/2*(sigma1-sigma2);  % Tresca criterion
tMAX(c) = tauMAX;
sigmaV = 1/sqrt(2)*sqrt((sigma1-sigma2)^2 + sigma1^2 +sigma2^2);  % von mises
sV(c) = sigmaV; 

Len(c) = L; 
the(c) = theta; 
c=c+1;

end
 
 the = the*(180/pi);
 
 %{
 figure(1)
 subplot(2,1,1)
 plot(Len,tMAX,'rx-')
xlabel('Length');
subplot(2,1,2);
 plot(the,tMAX,'bo-')
xlabel('theta');
 title('tau max = 1/2(sigma 1 - sigma 2) vs parameter');
 %}
 
 figure(1)
 subplot(2,1,1)
 plot(Len,sV,'bo-')
 xlabel('Length');
 title('Von Mises vs. parameter');
 subplot(2,1,2);
 plot(the,sV,'bo-')
 xlabel('theta');
