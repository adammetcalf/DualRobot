function[T01,T02,T03,T04,T05,T06,T07] = f_DH(RobotOrigin,T1,T2,T3,T4,T5,T6,T7)
%This function performs standard DH for forward kinematics, obtaining the
%joint frames from the robot angles

d1 = 0.34;
d3 = 0.4;
d5 = 0.4;
d7 = 0.126;



T01 = [cos(T1) 0 -sin(T1) RobotOrigin(1)
    sin(T1) 0 cos(T1) RobotOrigin(2)
    0 -1 0 RobotOrigin(3)+d1
    0 0 0 1];

T12 = [cos(T2) 0 sin(T2) 0
    sin(T2) 0 -cos(T2) 0
    0 1 0 0
    0 0 0 1];

T23 = [cos(T3) 0 sin(T3) 0
    sin(T3) 0 -cos(T3) 0
    0 1 0 d3
    0 0 0 1];

T34 = [cos(T4) 0 -sin(T4) 0
    sin(T4) 0 cos(T4) 0
    0 -1 0 0
    0 0 0 1];

T45 = [cos(T5) 0 -sin(T5) 0
    sin(T5) 0 cos(T5) 0
    0 -1 0 d5
    0 0 0 1];

T56 = [cos(T6) 0 sin(T6) 0
    sin(T6) 0 -cos(T6) 0
    0 1 0 0
    0 0 0 1];

T67 = [cos(T7) -sin(T7) 0 0
    sin(T7) cos(T7) 0 0
    0 0 1 d7
    0 0 0 1];

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
T07 = T01*T12*T23*T34*T45*T56*T67;

end