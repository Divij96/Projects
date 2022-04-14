robot = rigidBodyTree('dataFormat','row');

L1=0.6; %m
L2=0.6; %m
D1=0.3; %m

% I am not using the dh parameters
dhparams = [0   0  D1  0;
            L1	0  0   0;
            L2	0  0	0];


body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');

%setFixedTransform(jnt1,dhparams(1,:),'dh');
%setFixedTransform(jnt2,dhparams(2,:),'dh');
%setFixedTransform(jnt3,dhparams(3,:),'dh');
tform = trvec2tform([0, 0, D1]);
tform2 = trvec2tform([L1, 0, 0]);
tform3 = trvec2tform([L2, 0, 0]);

setFixedTransform(jnt1,tform);
setFixedTransform(jnt2,tform2);
setFixedTransform(jnt3,tform3);

% Joint limits
jnt1.PositionLimits=[deg2rad(15), deg2rad(165)];
jnt1.HomePosition=deg2rad(15);

jnt2.PositionLimits=[deg2rad(4), deg2rad(180)];
jnt2.HomePosition=deg2rad(4);

jnt3.PositionLimits=[deg2rad(-95), deg2rad(95)];
jnt3.HomePosition=deg2rad(0);

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;

addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')

% Endeffector
bodyEndEffector = rigidBody('endeffector');
tform4 = trvec2tform([0, 0, -0.1]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'body3');

showdetails(robot)
config = homeConfiguration(robot);