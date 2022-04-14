l1 = 0.6;
l2 = 0.6;

q1=3.14/8;
q2=3.14/8;
q3=0;
Q0 = [q1, q2, q3]';
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
J = [- l2*sin(q(1,i-1) + q(2,i-1))-l1*sin(q(1,i-1)), -l2*sin(q(1,i-1) + q(2,i-1)), 0; 
    l2*cos(q(1,i-1) + q(2,i-1)) + l1*cos(q(1,i-1)), l2*cos(q(1,i-1) + q(2,i-1)), 0
    1,                                              1,                             1  ];
qdot(:,i) = J\(X(: , i)-X(: , i-1)); %Xdot(:, i)  instead X(: , i)-X(: , i-1) 
q(:,i) = q(:,i-1) + qdot(:,i); % qdot(:,i) * dt

end

%%Special output for simulink workspace
t = t';

Xd_input_data.time = t;
Xd_input_data.dimension = [length(X) 1];
Xd_input_data.signals.values = X';

q_input_data.time = t;
q_input_data.dimension = [length(X) 1];
q_input_data.signals.values = [Q0, q(:, 1:length(X) -1)]';


function X = ef_pos_from_q(q1, q2, q3, l1, l2)
X = [l2*cos(q1 + q2) + l1*cos(q1) l2*sin(q1 + q2) + l1*sin(q1) q1 + q2 + q3]';
end