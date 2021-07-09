clear all;clc;close all;

%Defining Constants
M = 7802;       %[kg] Mass
Ac = 19.6350;   %[m^2] Capsule Area
N = 3;          %[-] Number of Parachutes
Cc = 1.3;       %[-] Coefficient of Friction of Capsule
Cp = .95;       %[-] Coefficient of Friction of Parachute
g = 9.81;       %[m/s^2] Acceleration due to gravity
rho = 1.225;    %[kg/m^3] Air density
Vf = 7.6;       %[m/s] Final Velocity
Df = 35.36;     %[m] Final Diameter
rhof = 1.225;
Ap = (pi/4)*Df^2; %Parachute Area


Vt = sqrt((2*M*g)/(rhof*(N*Ap*Cp+Ac*Cc))); % Terminal Velocity [m/s]
Ft = .5*N*rhof*Vt^2*Ap*Cp; %Terminal Force [N]


%Calculating Constant Parachute Force
%Z = 0.5*rho*N*Vf^2*0.25*pi*Df^2*Cp; %[N]
Z = Ft;
Z_high = Z; 


%Symbolic x(t) representing the parachute diameter
syms x(t)

%Derived Physics Differential Equation
ode = (Z_high/M)*(1+(Cc/Cp)*Ac/(N*(pi/4)*x^2)) - sqrt(8*Z_high/(pi*rho*N*Cp))*diff(x,t)/x^2 == g;

%Solving for x(t), xdot(t), xdotdot(t)

Dx = diff(x,t);
cond = x(0) == 0.03*Df;
xSol(t) = dsolve(ode,cond);
xdotSol(t) = pi*diff(xSol,t);
xdotdotSol(t) = diff(xdotSol,t);

%Time Steps
T = [0:0.1:200];
xlin = linspace(0.03*Df,Df,length(T));

%Solving values at timesteps
x = xSol(T);
xdot = xdotSol(T);
xdotdot = xdotdotSol(T);



%find(x>
%plotting
subplot(3,1,1);
plot(T,x, 'b','LineWidth',2)
ylabel('Diameter[m]')
xlabel('Time[s]')


subplot(3,1,2);
plot(x,xdot,'g','LineWidth',2)
ylabel('Line Velocity[m/s]')
xlabel('Diameter [m]')

subplot(3,1,3);
plot(x,xdotdot,'r','LineWidth',2)
ylabel('Line Acceleration[m/s^2]')
xlabel('Diameter [m]')


figure

Tension = -0.043*log(xlin)+0.1533;
Tension = Tension./max(Tension);
xdot_Norm = xdot./(max(xdot));
xdotdot_Norm = -1*xdotdot./(min(xdotdot));

subplot(3,1,1);
plot(T,x, 'b','LineWidth',2)
ylabel('Diameter[m]')
xlabel('Time[s]')


subplot(3,1,2);
plot(x,xdot_Norm,'g','LineWidth',2)
ylabel('XDot Norm [- ]')
xlabel('Diameter [m]')

subplot(3,1,3);
plot(x,xdotdot_Norm,'r','LineWidth',2)
ylabel('Xdotdot Norm [-]')
xlabel('Diameter [m]')


figure

Ft = 1 - x./max(x);


Sum = xdot_Norm + xdotdot_Norm - Tension;
plot(x, -Sum);
ylabel('Normalized Friction Force')
xlabel('Diameter [m]')


del2 = asin(x./(468*0.3048));
del1 = deg2rad(18);
Trl = Z*(tan(del1-del2)-tan(del2))/(2*pi);




