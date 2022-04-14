syms q1 q2 q3 l1 l2
%(thera, d, a, alpha)
DH =[q1 0 l1 0;
     q2 0 l2 0;
     q3 0  0 0];
T0_W = eye(4);
[T, RT] = robot_ForwardKinematic( DH, 'CM', false, 'T0_W', T0_W);
[J_0] = robot_Jacobians(T, 'rrr', 'CM', false, 'T0_W', T0_W);
%% Print Jacobians
fprintf('--- Jacobian of end effector with respect to Baseframe 0 ---')
Jef_0 = J_0(:,:,3)
%% Print Transformations
%fprintf('--- HT of end effector with respect to Baseframe 0 ---')
%Tef_0 = T(:,:,3)


% Tef_0 =
%  
% [cos(q1 + q2 + q3), -sin(q1 + q2 + q3), 0, l2*cos(q1 + q2) + l1*cos(q1)]
% [sin(q1 + q2 + q3),  cos(q1 + q2 + q3), 0, l2*sin(q1 + q2) + l1*sin(q1)]
% [                0,                  0, 1,                            0]
% [                0,                  0, 0,                            1]

% Jef_0 =
%  
% [- l2*sin(q1 + q2) - l1*sin(q1), -l2*sin(q1 + q2), 0]
% [  l2*cos(q1 + q2) + l1*cos(q1),  l2*cos(q1 + q2), 0]
% [                             0,                0, 0]
% [                             0,                0, 0]
% [                             0,                0, 0]
% [                             1,                1, 1]