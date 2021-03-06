function [ output_args ] = PlotCF( q1, q2, q3,l1, l2)


%Plot an imaginary Box using CF in each corner and in the box center (see
%Fig. 4.2 --session1.pdf

hVector=[0 0 0 1];
tOff=0.05;
aLength=0.1;
%q3 = 3.14/3;
figure(1)
O_0=[0;0;0];
O_1=[l1*cos(q1);l1*sin(q1);0];
O_2=[l2*cos(q1 + q2) + l1*cos(q1) ;l2*sin(q1 + q2) + l1*sin(q1);0];

grid on
hold on
axisX_0=[aLength;0;1];
axisY_0=[0;aLength;1];
T = [cos(q1 + q2 + q3) -sin(q1 + q2 + q3) O_2(1);
     sin(q1 + q2 + q3) cos(q1 + q2 + q3) O_2(2);
     0       0       1      ];
axisX_0 = T* axisX_0;
axisY_0 = T * axisY_0;
% Origin O_0
plot3(O_0(1),O_0(2),O_0(3), 'k .','MarkerSize',30)
plot3(O_1(1),O_1(2),O_1(3), 'k .','MarkerSize',30)
plot3(O_2(1),O_2(2),O_2(3), 'k .','MarkerSize',30)
plot3([O_0(1);O_1(1)],[O_0(2);O_1(2)],[O_0(3);O_1(3)],'k -')
plot3([O_1(1);O_2(1)],[O_1(2);O_2(2)],[O_1(3);O_2(3)],'k -')
text(O_0(1)+tOff,O_0(2)+tOff,O_0(3)+tOff, 'O_0');
% %Plot x-axis
% plot3([O_0(1);axisX_0(1)],[O_0(2);axisX_0(2)],[O_0(3);axisX_0(3)],'r -')
% %Plot y-axis
% plot3([O_0(1);axisY_0(1)],[O_0(2);axisY_0(2)],[O_0(3);axisY_0(3)],'g -')
% %Plot x-axis
% plot3([O_1(1);axisX_1(1)],[O_1(2);axisX_1(2)],[O_1(3);axisX_1(3)],'r -')
% %Plot y-axis
% plot3([O_1(1);axisY_1(1)],[O_1(2);axisY_1(2)],[O_1(3);axisY_1(3)],'g -')
% %Plot x-axis
 plot([O_2(1);axisX_0(1)],[O_2(2);axisX_0(2)],'r -')
% %Plot y-axis
 plot([O_2(1);axisY_0(1)],[O_2(2);axisY_0(2)],'g -')



end

