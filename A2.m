clear all;

T1= [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     -1 -1 -1 1];
     
T2= [1 0 0 0;
     0 9 4 0;
     0 -4 9 0;
     0 0 0 1];

T3= [sqrt(97) 0 4 0;
     0 1 0 0;
     -4 0 sqrt(97) 0;
     0 0 0 1];
 
T4 = [-1 0 0 0;
      0 -1 0 0;
      0 0 1 0;
      0 0 0 1];

%p = [1 1 1];% + [4 4 9]; 
p = [5 5 10];
vec = [p 1];
 pCell= num2cell(vec*T1*T2*T3*(1/sqrt(97)));

[x,y,z,k1] = pCell{:}

pCell2= num2cell(vec*T1*T2*T3*T4*(1/sqrt(97)));

[x,y,z,k1] = pCell2{:}

