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
% Se determinan las MTH generales expresando las variables de forma simbólica
syms q1 q2 q3 q4 q5 q6
%      Link('revolute/prismatic','a','alpha','d/theta','offset','qlim','modified/standard') 
L(1) = Link('revolute', 'a',   0, 'alpha',     0, 'd',  89.2, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q1
L(2) = Link('revolute', 'a',   0, 'alpha', -pi/2, 'd',   134, 'offset', -pi/2, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q2
L(3) = Link('revolute', 'a', 425, 'alpha',    pi, 'd',   119, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q3
L(4) = Link('revolute', 'a', 392, 'alpha',    pi, 'd', 94.75, 'offset',  pi/2, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q4
L(5) = Link('revolute', 'a',   0, 'alpha',  pi/2, 'd', 94.75, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q5
L(6) = Link('revolute', 'a',   0, 'alpha', -pi/2, 'd',     0, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q6

plot_opt = {'workspace',[-1 1 -1 1 -0.5 2]*750,'scale',0.4,'tilesize',0.5,'tile1color',[0.756, 0.945, 0.933],'jaxes','view',[130 30],'noa'};
axis equal
% Construccion del robot/manipulador
Robot = SerialLink(L,'name','Universal Robot UR5e','plotopt',plot_opt);
Robot.tool = [1  0  0    0;...
              0  1  0    0;...
              0  0  1 81.5;...
              0  0  0    1];
Robot
MTH_0T1 = roundA(L(1).A(q1),1)
MTH_1T2 = roundA(L(2).A(q2),1)
MTH_2T3 = roundA(L(3).A(q3),1)
MTH_3T4 = roundA(L(4).A(q4),1)
MTH_4T5 = roundA(L(5).A(q5),1)
MTH_5T6 = roundA(L(6).A(q6),1)
TCP = Robot.fkine([q1 q2 q3 q4 q5 q6]); % Solución problema geométrico directo
TCP = roundA(TCP,1)

%//////////////////////////////////////////////////////////////////////////
q = [0 0 0 0 0 0];
% Robot.plot(q) %Grafica en plano para poder publicar desde MATLAB.
Robot.teach(q)
hold on 

MTH0T1 = L(1).A(0);
MTH1T2 = L(2).A(0);
MTH2T3 = L(3).A(0);
MTH3T4 = L(4).A(0);
MTH4T5 = L(5).A(0);
MTH5T6 = L(6).A(0);
TCP = Robot.fkine(q); % Solución problema geométrico directo
trplot(eye(4),'length',100,'thick',3,'rgb') % Grafica el origen
trplot(MTH0T1,'length',100,'thick',3,'rgb')
trplot(MTH0T1*MTH1T2,'length',100,'thick',3,'rgb')
trplot(MTH0T1*MTH1T2*MTH2T3,'length',100,'thick',3,'rgb')
trplot(MTH0T1*MTH1T2*MTH2T3*MTH3T4,'length',100,'thick',3,'rgb')
trplot(MTH0T1*MTH1T2*MTH2T3*MTH3T4*MTH4T5,'length',100,'thick',3,'rgb')
trplot(MTH0T1*MTH1T2*MTH2T3*MTH3T4*MTH4T5*MTH5T6,'length',100,'thick',3,'rgb')
trplot(TCP,'length',100,'rgb')

%CINEMÁTICA DIRECTA
%//////////////////////////////////////////////////////////////////////////
q = [0 0 0 0 0 0]
TCP = Robot.fkine(q) % Solución problema geométrico directo para el *q* dado
[ROT,POS]=tr2rt(TCP);
POS = POS' %Posición en coordenadas cartesianas 
ROT = tr2rpy(ROT,'deg') %Orientación en ángulos fijos.
%//////////////////////////////////////////////////////////////////////////
q = [0.5 0.2 0.4 0.5 0 1.5];
TCP = Robot.fkine(q); % Solución problema geométrico directo para el *q* dado
[ROT1,POS1]=tr2rt(TCP);
POS1 = POS1' %Posición en coordenadas cartesianas 
ROT1 = tr2rpy(ROT1,'deg') %Orientación en ángulos fijos.
%//////////////////////////////////////////////////////////////////////////
q = [-pi/2 0.3 0 pi/2 0.4 1.2];
TCP = Robot.fkine(q); % Solución problema geométrico directo para el *q* dado
[ROT2,POS2]=tr2rt(TCP);
POS2 = POS2' %Posición en coordenadas cartesianas 
ROT2 = tr2rpy(ROT2,'deg') %Orientación en ángulos fijos.
%//////////////////////////////////////////////////////////////////////////
q = [0 1 -0.5 2 1 0.5];
TCP = Robot.fkine(q); % Solución problema geométrico directo para el *q* dado
[ROT3,POS3]=tr2rt(TCP);
POS3 = POS3' %Posición en coordenadas cartesianas 
ROT3 = tr2rpy(ROT3,'deg') %Orientación en ángulos fijos.
%//////////////////////////////////////////////////////////////////////////
q = [-1 -0.3 -pi/5 0.4 0.2 1];
TCP = Robot.fkine(q); % Solución problema geométrico directo para el *q* dado
[ROT4,POS4]=tr2rt(TCP);
POS4 = POS4' %Posición en coordenadas cartesianas 
ROT4 = tr2rpy(ROT4,'deg') %Orientación en ángulos fijos.
%//////////////////////////////////////////////////////////////////////////

%CINEMÁTICA INVERSA
%ik_sym = Robot.ikine_sym(6);
options = struct('MaxFunEvals', 1000);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C1 = [500 500 500 -90 0 0]
TRVEC=[C1(1,1:3) 1]';
ROTM=rpy2tr(deg2rad(C1(1,4:6)));
TCP = [ROTM(:,1:3) TRVEC];
qC1 = Robot.ikinem(TCP,[0.5 0.6 -1.0 -1.6 -0.5 0.0])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C2 = [100 -200 300 90 90 45]
TRVEC=[C2(1,1:3) 1]';
ROTM=rpy2tr(deg2rad(C2(1,4:6)));
TCP = [ROTM(:,1:3) TRVEC];
qC2 = Robot.ikinem(TCP,[-2.0 2.1 2.6 0.5 0.4 -2.3])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C3 = [-300 400 -500 -30 -45 -90]
TRVEC=[C3(1,1:3) 1]';
ROTM=rpy2tr(deg2rad(C3(1,4:6)));
TCP = [ROTM(:,1:3) TRVEC];
qC3 = Robot.ikinem(TCP,[1.8 3.1 1.1 0.3 0.9 1.2])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C4 = [123 456 789 -90 -45 -30]
TRVEC=[C4(1,1:3) 1]';
ROTM=rpy2tr(deg2rad(C4(1,4:6)));
TCP = [ROTM(:,1:3) TRVEC];
qC4 = Robot.ikinem(TCP,[0.8 1.0 0.8 -0.1 -0.1 -0.5])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
%MODELO DIFERENCIAL DE PRIMER ORDEN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Se determinan las MTH generales expresando las variables de forma simbólica
syms q1 q2 q3 q4 q5 q6
%      Link('revolute/prismatic','a','alpha','d/theta','offset','qlim','modified/standard') 
L(1) = Link('revolute', 'a',   0, 'alpha',     0, 'd',  89.2, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q1
L(2) = Link('revolute', 'a',   0, 'alpha', -pi/2, 'd',   134, 'offset', -pi/2, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q2
L(3) = Link('revolute', 'a', 425, 'alpha',    pi, 'd',   119, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q3
L(4) = Link('revolute', 'a', 392, 'alpha',    pi, 'd', 94.75, 'offset',  pi/2, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q4
L(5) = Link('revolute', 'a',   0, 'alpha',  pi/2, 'd', 94.75, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q5
L(6) = Link('revolute', 'a',   0, 'alpha', -pi/2, 'd',     0, 'offset',     0, 'modified', 'qlim', [-2*pi/3 2*pi/3]); %'theta'=q6

plot_opt = {'workspace',[-1 1 -1 1 -0.5 2]*750,'scale',0.4,'tilesize',0.5,'tile1color',[0.756, 0.945, 0.933],'jaxes','view',[130 30],'noa'};
axis equal
% Construccion del robot/manipulador
Robot = SerialLink(L,'name','Universal Robot UR5e','plotopt',plot_opt);
Robot.tool = [1  0  0    0;...
              0  1  0    0;...
              0  0  1 81.5;...
              0  0  0    1];
Robot
MTH_0T1 = roundA(L(1).A(q1),1)
MTH_1T2 = roundA(L(2).A(q2),1)
MTH_2T3 = roundA(L(3).A(q3),1)
MTH_3T4 = roundA(L(4).A(q4),1)
MTH_4T5 = roundA(L(5).A(q5),1)
MTH_5T6 = roundA(L(6).A(q6),1)
TCP = Robot.fkine([q1 q2 q3 q4 q5 q6]); % Solución problema geométrico directo
TCP = roundA(TCP,1)

%Jacobiano geométrico/convencional/base (6xn)
MTH_0T1 = MTH_0T1;
MTH_0T2 = MTH_0T1*MTH_1T2;
MTH_0T3 = MTH_0T1*MTH_1T2*MTH_2T3;
MTH_0T4 = MTH_0T1*MTH_1T2*MTH_2T3*MTH_3T4;
MTH_0T5 = MTH_0T1*MTH_1T2*MTH_2T3*MTH_3T4*MTH_4T5;
MTH_0T6 = MTH_0T1*MTH_1T2*MTH_2T3*MTH_3T4*MTH_4T5*MTH_5T6;

%Cálculo de los Z respecto a 0
Z01=MTH_0T1(1:3,3);
Z02=MTH_0T2(1:3,3);
Z03=MTH_0T3(1:3,3);
Z04=MTH_0T4(1:3,3);
Z05=MTH_0T5(1:3,3);
Z06=MTH_0T6(1:3,3);

%Cálculo de los P
P17=TCP(1:3,4)-MTH_0T1(1:3,4);
P27=TCP(1:3,4)-MTH_0T2(1:3,4);
P37=TCP(1:3,4)-MTH_0T3(1:3,4);
P47=TCP(1:3,4)-MTH_0T4(1:3,4);
P57=TCP(1:3,4)-MTH_0T5(1:3,4);
P67=TCP(1:3,4)-MTH_0T6(1:3,4);

%Cálculo de las J
Jg1=[cross(Z01,P17); Z01]; %revolute
Jg2=[cross(Z02,P27); Z02]; %revolute
Jg3=[cross(Z03,P37); Z03]; %revolute
Jg4=[cross(Z04,P47); Z04]; %revolute
Jg5=[cross(Z05,P57); Z05]; %revolute
Jg6=[cross(Z06,P67); Z06]; %revolute
Jg=simplify([Jg1 Jg2 Jg3 Jg4 Jg5 Jg6])

%//////////////////////////////////////////////////////////////////////////

%Jacobiano analítico de posición en coordenadas cartesianas (3xn)
posX = TCP(1,4);
posY = TCP(2,4);
posZ = TCP(3,4);
J11 = diff(posX,q1); J12 = diff(posX,q2); J13 = diff(posX,q3); J14 = diff(posX,q4); J15 = diff(posX,q5);  J16 = diff(posX,q6);
J21 = diff(posY,q1); J22 = diff(posY,q2); J23 = diff(posY,q3); J24 = diff(posY,q4); J25 = diff(posY,q5);  J26 = diff(posY,q6);
J31 = diff(posZ,q1); J32 = diff(posZ,q2); J33 = diff(posZ,q3); J34 = diff(posZ,q4); J35 = diff(posZ,q5);  J36 = diff(posZ,q6);
Jpos = simplify([J11 J12 J13 J14 J15 J16;
                 J21 J22 J23 J24 J25 J26;
                 J31 J32 J33 J34 J35 J36]);

%Jacobiano analítico de orientación por derivada de la matriz de rotación
syms q1_dot q2_dot q3_dot q4_dot q5_dot q6_dot q1 q2 q3 q4 q5 q6
R_p1 = diff(TCP(1:3,1:3),q1)*q1_dot;
R_p2 = diff(TCP(1:3,1:3),q2)*q2_dot;
R_p3 = diff(TCP(1:3,1:3),q3)*q3_dot;
R_p4 = diff(TCP(1:3,1:3),q4)*q4_dot;
R_p5 = diff(TCP(1:3,1:3),q5)*q5_dot;
R_p6 = diff(TCP(1:3,1:3),q6)*q6_dot;

R_p = simplify(R_p1 + R_p2 + R_p3 + R_p4 + R_p5 + R_p6);
skew = simplify (R_p * transpose(TCP(1:3,1:3)));
%Se obtiene el vector w
w=[skew(3,2) skew(1,3) skew(2,1)];
w=expand(w);
w_temp=children(w);

Jrot=[0 w_temp{1}(1)/q2_dot w_temp{1}(2)/q3_dot w_temp{1}(3)/q4_dot (w_temp{1}(5)+w_temp{1}(6)+w_temp{1}(7)+w_temp{1}(8))/q5_dot (w_temp{1}(4)+w_temp{1}(9)+w_temp{1}(10)+w_temp{1}(11)+w_temp{1}(12))/q6_dot;
      0 w_temp{2}(1)/q2_dot w_temp{2}(2)/q3_dot w_temp{2}(3)/q4_dot (w_temp{2}(5)+w_temp{2}(6)+w_temp{2}(7)+w_temp{2}(8))/q5_dot (w_temp{2}(4)+w_temp{2}(9)+w_temp{2}(10)+w_temp{2}(11)+w_temp{2}(12))/q6_dot;
      w_temp{3}(1)/q1_dot 0 0 0 (w_temp{3}(2)+w_temp{3}(3)+w_temp{3}(4)+w_temp{3}(5))/q5_dot (w_temp{3}(6)+w_temp{3}(7)+w_temp{3}(8)+w_temp{3}(9))/q6_dot;];

Ja = simplify([Jpos;Jrot])

%//////////////////////////////////////////////////////////////////////////
q = [0.5278 0.6336 -1.0047 -1.6383 -0.5278 -0.0000]
J = Robot.jacob0(q)
%Se evalúan (subs) y simplifican (vpa) los J para el q dado  
Jg_q = vpa(subs(Jg,[q1,q2,q3,q4,q5,q6],[q]),6)
Ja_q = vpa(subs(Ja,[q1,q2,q3,q4,q5,q6],[q]),6)
%//////////////////////////////////////////////////////////////////////////


%% Función *roundA* para recorrer las MTH y simplificar cada elemento simbólico
%
function M=roundA(A,n)
    A=simplify(A);
    for i=1:4
        for j=1:4
            [C,T]=coeffs(A(i,j));
            C=double(C);
            C=round(C,n);
            M(i,j)=dot(C,T);
        end
    end
end