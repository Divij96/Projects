function [ Ht ] = DH2Ht(  theta, d,a,alpha)

Hrz=[cos(theta) -sin(theta) 0 0;
     sin(theta)  cos(theta) 0 0;
     0           0          1 0;
     0           0          0 1];
 Htz=[ eye(3,3) [0;0;d]; 0 0 0 1]
 
 Hrx=eye(4,4);
 Htx=eye(4,4);
 
 Ht=Hrz*Htz*Hrx*Htx
 
 

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


end

