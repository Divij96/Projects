close all
%X = [x y phi];deg2rad(q1)
%https://automaticaddison.com/homogeneous-transformation-matrices-using-denavit-hartenberg/
%https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/

l1 = 0.6;
l2 = 0.6;

q1=3.14/8;
q2=3.14/8;
q3=0;

X0= ef_pos_from_q(q1, q2, q3, l1, l2);
Xf= [0.5 0.5 -pi/2]';
t0 = 0;
dt = 0.01;
tf = 4;





t=t0:dt:tf;

a0 = X0;
a1 = [0 0 0]';
a2 = (3/(tf^2))*(Xf-X0);
a3 = (-2/(tf^3))*(Xf-X0);

X = a0 + a1 * t + a2 * t.^2 + a3 * t.^3;
Xdot = a1 + 2 * a2 * t + 3 * a3 * t.^2;
Xdotdot = 2 * a2 + 6 *a3 * t;
qdot = zeros(3, size(X,2));
q = zeros(3, size(X,2));
q(1,1) = q1;
q(2,1) = q2;
q(3,1) = q3;
qdot(1,1) = 0;
qdot(2,1) = 0;
qdot(3,1) = 0;
for i = 2:(length(X))
%J = [-l2*sin(q(1,i-1))*cos(q(2,i-1))- l2*cos(q(1,i-1))*sin(q(2,i-1))-l1*sin(q(1,i-1)), -l2*sin(q(1,i-1))*cos(q(2,i-1))-l2*cos(q(1,i-1))*sin(q(2,i-1)); l2*cos(q(1,i-1))*cos(q(2,i-1))-l2*sin(q(1,i-1))*sin(q(2,i-1))+l1*cos(q(1,i-1)), l2*cos(q(1,i-1))*cos(q(2,i-1))-l2*sin(q(1,i-1))*sin(q(2,i-1))];
J = [- l2*sin(q(1,i-1) + q(2,i-1))-l1*sin(q(1,i-1)), -l2*sin(q(1,i-1) + q(2,i-1)), 0; 
    l2*cos(q(1,i-1) + q(2,i-1)) + l1*cos(q(1,i-1)), l2*cos(q(1,i-1) + q(2,i-1)), 0
    1,                                              1,                             1  ];
qdot(:,i) = J\(X(: , i)-ef_pos_from_q(q(1 , i-1), q(2 , i-1), q(3 , i-1), l1, l2)); %Xdot(:, i)  instead X(: , i)-X(: , i-1) 
q(:,i) = q(:,i-1) + qdot(:,i); % qdot(:,i) * dt

end


figure

plot(X(1,:), X(2,:)); grid on;
xlabel('X Position');
ylabel('Y Position');

%J = [-l2*sin(q1)*cos(q2)- l2*cos(q1)*sin(q2)-l1*sin(q1), -l2*sin(q1)*cos(q2)-l2*cos(q1)*sin(q2); l2*cos(q1)*cos(q2)-l2*sin(q1)*sin(q2)+l1*cos(q1), l2*cos(q1)*cos(q2)-l2*sin(q1)*sin(q2)];
for i = 2:length(X)
hold on
if mod(i,4) == 0
PlotCF( q(1,i), q(2,i), q(3,i),0.6, 0.6)
end
end
% figure % plot x
% 
% subplot (3, 1, 1);
% plot(t, X(1,:)); grid on;
% xlabel('Time[s]');
% ylabel('Position_X[m]');
% 
% subplot (3, 1, 2);
% plot(t, Xdot(1,:)); grid on;
% xlabel('Time[s]');
% ylabel('Velocity_X[m/s]');
% 
% subplot (3, 1, 3);
% plot(t, Xdotdot(1,:)); grid on;
% xlabel('Time[s]');
% ylabel('Acceleration_X[m/s^2]');
% 
% figure %plot y
% 
% subplot (3, 1, 1);
% plot(t, X(2,:)); grid on;
% xlabel('Time[s]');
% ylabel('Position_Y[m]');
% 
% subplot (3, 1, 2);
% plot(t, Xdot(2,:)); grid on;
% xlabel('Time[s]');
% ylabel('Velocity_Y[m/s]');
% 
% subplot (3, 1, 3);
% plot(t, Xdotdot(2,:)); grid on;
% xlabel('Time[s]');
% ylabel('Acceleration_Y[m/s^2]');
% 
figure % plot phi

subplot (3, 1, 1);
plot(t, X(3,:)); grid on;
xlabel('Time[s]');
ylabel('Rotation_{Phi}[m]');

subplot (3, 1, 2);
plot(t, Xdot(3,:)); grid on;
xlabel('Time[s]');
ylabel('Velocity_{Phi}[m/s]');

subplot (3, 1, 3);
plot(t, Xdotdot(3,:)); grid on;
xlabel('Time[s]');
ylabel('Acceleration_{Phi}[m/s^2]');
% 
% figure %plot path
% 
% plot(X(1,:), X(2,:)); grid on;
% xlabel('X Position');
% ylabel('Y Position');

%plot qdot
figure
subplot (3, 1, 1);
plot(t, qdot(1,:)); grid on;
xlabel('Time[s]');
ylabel('Qdot1');

subplot (3, 1, 2);
plot(t, qdot(2,:)); grid on;
xlabel('Time[s]');
ylabel('Qdot2');

subplot (3, 1, 3);
plot(t, qdot(3,:)); grid on;
xlabel('Time[s]');
ylabel('Qdot3');


%plot q
figure

subplot (3, 1, 1);
plot(t, q(1,:)); grid on;
xlabel('Time[s]');
ylabel('Q1');

subplot (3, 1, 2);
plot(t, q(2,:)); grid on;
xlabel('Time[s]');
ylabel('Q2');

subplot (3, 1, 3);
plot(t, q(3,:)); grid on;
xlabel('Time[s]');
ylabel('Q3');




function X = ef_pos_from_q(q1, q2, q3, l1, l2)
%X = [x y phi]
X = [l2*cos(q1 + q2) + l1*cos(q1) l2*sin(q1 + q2) + l1*sin(q1) q1 + q2 + q3]';
end

