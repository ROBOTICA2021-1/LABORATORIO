%%
% ROBÓTICA - 2021-I
%% LABORATORIO: ANÁLISIS Y SIMULACIÓN DE UN ROBOT INDUSTRIAL
%
% *PRESENTADO POR:*
%
%% Universal Robots UR5e
%
clear 
clf 
clc

%//////////////////////////////////////////////////////////////////////////

robot = rigidBodyTree;

body1 = rigidBody('body1'); 
jnt1 = rigidBodyJoint('jnt1','revolute'); 
jnt1.HomePosition = 0; 
tform = trvec2tform([0,0,89.2/1000])*axang2tform([1 0 0 0]); % User defined 
setFixedTransform(jnt1,tform); 
body1.Joint = jnt1; 
addVisual(body1,"Mesh","ur5/base.stl",trvec2tform([0,0,-89.2/1000]));
addVisual(body1,"Mesh","ur5/shoulder.stl");
addBody(robot,body1,'base')

body2 = rigidBody('body2'); 
jnt2 = rigidBodyJoint('jnt2','revolute'); 
jnt2.HomePosition = -pi/2; % User defined 
tform2 = trvec2tform([0,134/1000,0])*axang2tform([1 0 0 -pi/2]); % User defined 
setFixedTransform(jnt2,tform2); 
body2.Joint = jnt2;
addVisual(body2,"Mesh","ur5/upper_arm.stl",axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/2]));
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3'); 
jnt3 = rigidBodyJoint('jnt3','revolute'); 
jnt3.HomePosition = 0; % User defined 
tform3 = trvec2tform([425/1000,0,-119/1000])*axang2tform([1 0 0 pi]); %User defined 
setFixedTransform(jnt3,tform3); 
body3.Joint = jnt3; 
addVisual(body3,"Mesh","ur5/forearm.stl",axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 -pi/2]));
addBody(robot,body3,'body2'); % Add body3 to body2

body4 = rigidBody('body4'); 
jnt4 = rigidBodyJoint('jnt4','revolute'); 
jnt4.HomePosition = pi/2; % User defined 
tform4 = trvec2tform([392/1000,0,-94.75/1000])*axang2tform([1 0 0 pi]); %User defined 
setFixedTransform(jnt4,tform4); 
body4.Joint = jnt4; 
addVisual(body4,"Mesh","ur5/wrist_1.stl",trvec2tform([0,0,-94.75/1000])*axang2tform([1 0 0 pi/2]));
addBody(robot,body4,'body3'); % Add body4 to body3

body5 = rigidBody('body5'); 
jnt5 = rigidBodyJoint('jnt5','revolute'); 
jnt5.HomePosition = 0; % User defined 
tform5 = trvec2tform([0,-94.75/1000,0])*axang2tform([1 0 0 pi/2]); %User defined 
setFixedTransform(jnt5,tform5);
body5.Joint = jnt5; 
addVisual(body5,"Mesh","ur5/wrist_2.stl",trvec2tform([0,0,-94.75/1000]));
addBody(robot,body5,'body4'); % Add body5 to body4

body6 = rigidBody('body6'); 
jnt6 = rigidBodyJoint('jnt6','revolute'); 
jnt6.HomePosition = 0; % User defined 
tform6 = trvec2tform([0,0,0])*axang2tform([1 0 0 -pi/2]); %User defined 
setFixedTransform(jnt6,tform6);
body6.Joint = jnt6;
addVisual(body6,"Mesh","ur5/wrist_3.stl",axang2tform([1 0 0 pi/2]));
addBody(robot,body6,'body5'); % Add body6 to body5

bodyEndEffector = rigidBody('endeffector'); 
tform7 = trvec2tform([0, 0, 81.5/1000]); % User defined 
setFixedTransform(bodyEndEffector.Joint,tform7); 
addBody(robot,bodyEndEffector,'body6');

showdetails(robot)
%show(robot)

TCP=getTransform(robot,homeConfiguration(robot),'endeffector')
POS=tform2trvec(TCP)*1000
ROT=rad2deg(tform2eul(TCP,'XYZ'))

%universalUR5 = loadrobot("universalUR5")
%config = homeConfiguration(universalUR5)
%config(2).JointPosition = -pi/2;
%config(4).JointPosition = -pi/2;
%show(universalUR5,config,'Visuals','on')

%CINEMÁTICA DIRECTA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=homeConfiguration(robot);
config1 = [0.5 0.2 0.4 0.5 0 1.5];
for i=1:6
    q(i).JointPosition = q(i).JointPosition + config1(i);
end
TCP=getTransform(robot,q,'endeffector');
POS1=tform2trvec(TCP)*1000
ROT1=rad2deg(tform2eul(TCP,'XYZ'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=homeConfiguration(robot);
config2 = [-pi/2 0.3 0 pi/2 0.4 1.2];
for i=1:6
    q(i).JointPosition = q(i).JointPosition + config2(i);
end
TCP=getTransform(robot,q,'endeffector');
POS2=tform2trvec(TCP)*1000
ROT2=rad2deg(tform2eul(TCP,'XYZ'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=homeConfiguration(robot);
config3 = [0 1 -0.5 2 1 0.5];
for i=1:6
    q(i).JointPosition = q(i).JointPosition + config3(i);
end
TCP=getTransform(robot,q,'endeffector');
POS3=tform2trvec(TCP)*1000
ROT3=rad2deg(tform2eul(TCP,'XYZ'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=homeConfiguration(robot);
config4 = [-1 -0.3 -pi/5 0.4 0.2 1];
for i=1:6
    q(i).JointPosition = q(i).JointPosition + config4(i);
end
TCP=getTransform(robot,q,'endeffector');
POS4=tform2trvec(TCP)*1000
ROT4=rad2deg(tform2eul(TCP,'XYZ'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%CINEMÁTICA INVERSA
ik = inverseKinematics('RigidBodyTree',robot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C1 = [500 500 500 -90 0 0]
TRVEC=[C1(1,1:3)/1000 1]';
ROTM=eul2tform(deg2rad(C1(1,4:6)),'XYZ');
TCP = [ROTM(:,1:3) TRVEC];
[configSol,solInfo] = ik('endeffector',TCP,[1 1 1 1 1 1],homeConfiguration(robot));
qhome=homeConfiguration(robot);
q=zeros(1,6);
for i=1:6
    q(i)=configSol(i).JointPosition-qhome(i).JointPosition;
end
q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C2 = [100 -200 300 90 90 45]
TRVEC=[C2(1,1:3)/1000 1]';
ROTM=eul2tform(deg2rad(C2(1,4:6)),'XYZ');
TCP = [ROTM(:,1:3) TRVEC];
[configSol,solInfo] = ik('endeffector',TCP,[1 1 1 1 1 1],homeConfiguration(robot));
qhome=homeConfiguration(robot);
q=zeros(1,6);
for i=1:6
    q(i)=configSol(i).JointPosition-qhome(i).JointPosition;
end
q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C3 = [-300 400 -500 -30 -45 -90]
TRVEC=[C3(1,1:3)/1000 1]';
ROTM=eul2tform(deg2rad(C3(1,4:6)),'XYZ');
TCP = [ROTM(:,1:3) TRVEC];
[configSol,solInfo] = ik('endeffector',TCP,[1 1 1 1 1 1],homeConfiguration(robot));
qhome=homeConfiguration(robot);
q=zeros(1,6);
for i=1:6
    q(i)=configSol(i).JointPosition-qhome(i).JointPosition;
end
q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C4 = [123 456 789 -90 -45 -30]
TRVEC=[C4(1,1:3)/1000 1]';
ROTM=eul2tform(deg2rad(C4(1,4:6)),'XYZ');
TCP = [ROTM(:,1:3) TRVEC];
[configSol,solInfo] = ik('endeffector',TCP,[1 1 1 1 1 1],homeConfiguration(robot));
qhome=homeConfiguration(robot);
q=zeros(1,6);
for i=1:6
    q(i)=configSol(i).JointPosition-qhome(i).JointPosition;
end
q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config=q;
qtest=homeConfiguration(robot);
for i=1:6
    qtest(i).JointPosition = qtest(i).JointPosition + config(i);
end
TCP=getTransform(robot,qtest,'endeffector');
POS=tform2trvec(TCP);
show(robot,qtest,'Frames','off')
axis tight;
hold on;
quiver3(0,0,0,POS(1),0,0,':r','LineWidth',2,'AutoScale','off','ShowArrowHead','off')
quiver3(POS(1),0,0,0,POS(2),0,':g','LineWidth',2,'AutoScale','off','ShowArrowHead','off')
quiver3(POS(1),POS(2),0,0,0,POS(3),':b','LineWidth',2,'AutoScale','off','ShowArrowHead','off')
text(POS(1),0,0," ("+POS(1)+",0,0)",'Color','r','HorizontalAlignment','right')
text(POS(1),POS(2),0,"("+POS(1)+","+POS(2)+",0)",'Color','g','HorizontalAlignment','right')
text(POS(1),POS(2),POS(3)," ("+POS(1)+","+POS(2)+","+POS(3)+")",'Color','b','HorizontalAlignment','left')

%MODELO DIFERENCIAL DE PRIMER ORDEN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = homeConfiguration(robot);
configq = [0.5278 0.6336 -1.0047 -1.6383 -0.5278 -0.0000]
for i=1:6
    q(i).JointPosition = q(i).JointPosition + configq(i);
end
J0 = geometricJacobian(robot,q,'endeffector');
J0 = [J0(4:6,:)*1000;J0(1:3,:)]

v=[100 200 50]'
w=[5 10 -5]'
q_dot=inv(J0)*[v;w]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%INTEGRACIÓN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%GRAFICA DEL PLANO
figure(2)
vector = [-1 0 1];
x=0; y=0; z=0; d=0;
q=homeConfiguration(robot);
q(1).JointPosition=-pi/2;%Rotación del Robot para poner en posición del plano
show(robot,q)
hold on
quiver3(0,0,0,-1,0,1,'Color','#000058','LineWidth',2,'AutoScale','off')
axis equal
axis([-0.5 1.1 -1.0 1.0 -0.8 0.8])
view([-135 30])
r = 750/1000; %Para un espacio de trabajo de 750mm
[Xs,Ys,Zs]=sphere;
Xs=Xs*r; Ys=Ys*r;Zs=Zs*r;
surf(Xs,Ys,Zs+89.2/1000,'LineStyle',':','FaceColor','none')
for i=-1:0.05:1
    for j=-1:0.05:1
        x=i; y=j; z=1*x-0*y+d; %Ecuación del plano: -1x+0y+1z=0
        plot3(x,y,z,'b.','MarkerSize',5)
    end
end

%TRAYECTORIAS Y PUNTOS
step = 0.005;
lim = 0.7*750/1000;
radius = 0.07; %Radio de redondeo de las esquinas
d = 0.4; %Desplazamiento X de la ruta en el plano
viapoints=[0 0 0];

%sideUp
for i=-lim/2+radius:step:lim/2-radius
    x=lim/5+d; y=-i; z=1*x-0*y; %Ecuación del plano: -1x+0y+1z=0
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%corner1
for i=0:step/2:radius
    x=lim/5-radius+d+sqrt(radius^2-i^2); y=-lim/2+radius-i; z=1*x-0*y;
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%sideRight
for i=-lim/5+radius:step:lim/5-radius
    x=-i+d; y=-lim/2; z=1*x-0*y; %Ecuación del plano: -1x+0y+1z=0
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%corner2
for i=0:step/2:radius
    x=-lim/5+radius+d-i; y=-lim/2+radius-sqrt(radius^2-i^2); z=1*x-0*y;
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%sideDown
for i=-lim/2+radius:step:lim/2-radius
    x=-lim/5+d; y=i; z=1*x-0*y; %Ecuación del plano: -1x+0y+1z=0
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%corner3
for i=0:step/2:radius
    x=-lim/5+radius-sqrt(radius^2-i^2)+d; y=lim/2-radius+i; z=1*x-0*y;
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%sideLeft
for i=-lim/5+radius:step:lim/5-radius
    x=i+d; y=lim/2; z=1*x-0*y; %Ecuación del plano: -1x+0y+1z=0
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
%corner4
for i=0:step/2:radius
    x=lim/5-radius+i+d; y=lim/2-radius+sqrt(radius^2-i^2); z=1*x-0*y;
    plot3(x,y,z,'b.','MarkerSize',12)
    viapoints = [viapoints; [x y z]];
end
viapoints

%CINEMÁTICA INVERSA
configPreview=homeConfiguration(robot);
qinitial = [0.2469    0.4564   -0.7207   -3.5179    1.7445    1.7471]; %Vector 'q' de referencia para reducir multiplicidad
for i=1:6
    configPreview(i).JointPosition = configPreview(i).JointPosition + qinitial(i);
end
qs=[0 0 0 0 0 0]; %Vector 'qs' para las configuraciones de trayectoria 
for i=2:size(viapoints)
    C = [viapoints(i,1) viapoints(i,2) viapoints(i,3) 0 135 180];
    TRVEC=[C(1,1:3) 1]';
    ROTM=eul2tform(deg2rad(C(1,4:6)),'XYZ');
    TCP = [ROTM(:,1:3) TRVEC];
    [configSol,solInfo] = ik('endeffector',TCP,[1 1 1 1 1 1],configPreview);
    qhome=homeConfiguration(robot);
    q=zeros(1,6);
    for j=1:6
        q(j)=configSol(j).JointPosition-qhome(j).JointPosition;
    end
    configPreview=configSol;
    qs = [qs; q];
end
qs

%GRAFICO DE TRAYECTORIA
vel=500/1000;
tim=step/vel;
for i=2:size(qs)
    q=homeConfiguration(robot);
    config1 = qs(i,:);
    for j=1:6
        q(j).JointPosition = q(j).JointPosition + config1(j);
    end
    show(robot,q,'PreservePlot',false)
    drawnow
    %pause(tim/50)
end

%GRAFICO ANGULOS DE ARTICULACIÓN
figure(3)
[m,n]=size(qs);
axis([0 m -1 1]) 
for i=2:m
    subplot(3,2,1)
    grid minor
    ylabel('Ángulo de J1 (RAD)')
    plot(i-1,qs(i,1),'b.','MarkerSize',3)
    hold on
    title('J1')
    
    subplot(3,2,3)       
    grid minor
    ylabel('Ángulo de J2 (RAD)')
    plot(i-1,qs(i,2),'b.','MarkerSize',3)     
    hold on
    title('J2')
    
    subplot(3,2,5)      
    grid minor
    ylabel('Ángulo de J3 (RAD)')
    plot(i-1,qs(i,3),'b.','MarkerSize',3)
    hold on
    title('J3')
    
    subplot(3,2,2)       
    grid minor    
    ylabel('Ángulo de J4 (RAD)')
    plot(i-1,qs(i,4),'b.','MarkerSize',3)
    hold on
    title('J4')
    
    subplot(3,2,4)       
    grid minor
    ylabel('Ángulo de J5 (RAD)')
    plot(i-1,qs(i,5),'b.','MarkerSize',3)
    hold on
    title('J5')
    
    subplot(3,2,6)       
    grid minor
    ylabel('Ángulo de J6 (RAD)')
    plot(i-1,qs(i,6),'b.','MarkerSize',3)
    hold on
    title('J6')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%