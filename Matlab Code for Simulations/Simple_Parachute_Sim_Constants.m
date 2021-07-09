%% Simulation Constants
clc; close all; clear all;

%Intial States
X0 = 2750; %Parachute deployment altitude [m]
V0 = -58; %Vehicle Speed at Deployment [m/s]

%Vehicle Properties
M_v = 10829.52; %Vehicle Mass [kg] (FROM KOKI EMAIL)
Cd_v = 1.3; %Vehicle Drag Coefficient [-]
D_v = 5; %Vehicle Diameter [m]
A_v = (pi/4)*D_v^2; %Vehicle Area [m^2]

%Parachute Properties
M_p = 113.4; %Parachute Mass [kg] [FROM APPARENT MASS STUDY]
D_p = 35.36; %Inflated Parachute Diameter [m]
A_p = (pi/4)*D_p^2; %Inflated Parachute Area [m^2]
Cd_p = 0.95; %Parachute Drag Coefficient [-]
N = 3; %Number of parachutes [-]

%Physics Properties
g = -9.80665; %Acceleration due to Gravity [m/s]

%Density Calculations (Polyfit of Density Equations)
rhoCflag = 0; % Change to 1 if you want to use constant rho in calcs

if rhoCflag == 1
x1 = 0 ;
x2 = 0;
x3 = 1.225;

else 
x1 = 3.979E-09 ;
x2 = -0.000117;
x3 = 1.224;
end


%% Calculations

V_term = -1*sqrt((-2*(M_v+N*M_p)*g)/(x3*(N*A_p*Cd_p+A_v*Cd_v))); % Terminal Velocity [m/s]
Z = 0.5*N*x3*V_term^2*A_p*Cd_p; %Terminal Force [N]
Z_Factor = 1.2; %[-] Z Multiplier

Z = Z*Z_Factor;

%% Running simulink Model

tspan = 100; %[s] Duration of Model run, can change this if necessary
[T,X,Y] =sim('Basic_Parachute_Dynamics_Simulink_Model', [0,tspan]);


%Pulling Outputs from the model
Altitude = X(:,1);          %[m] System Altitude
Velocity = X(:,2);          %[m/s] System Velocity
Acceleration = Y(:,1);      %[m/s^2] System Acceleration
ParachuteForce = Y(:,2);    %[N] Parachute Force
Diameter_Profile = Y(:,4);  %[m] Reefing diameter as a function of time


%Solving for diameter profiles
D = Diameter_Profile; 
Ddot = pi*diff(D)./diff(T);
Ddotdot = diff(Ddot)./diff(T(1:end-1));



%Finding Points of Interest
Max_Acc = max(Acceleration);
Max_Force = min(ParachuteForce);
Collision_Vel = Velocity(end-1);
Max_Line_Velocity = max(Ddot);
Time_Index = find(Diameter_Profile == D_p); %Finding Steady state index
SS_Time = T(Time_Index(1)); %Finding time at SS index



display(['Max Acceleration = ' , num2str(Max_Acc), ' Gs'])
display(['Max Parachute Force = ' , num2str(-1*Max_Force), ' N'])
display(['Time to Steady State = ' , num2str(SS_Time), ' s'])
display(['Steady State Velocity = ' , num2str(Collision_Vel), ' m/s'])
display(['Terminal Velocity = ' , num2str(V_term), ' m/s'])
display(['Max Reefing Line Velocity = ' , num2str(Max_Line_Velocity), ' m/s'])


%% Plotting Results
figure 

subplot(3,1,1);
plot(T,D, 'b','LineWidth',2)
ylabel('Diameter[m]')
xlabel('Time[s]')
grid on
title(['Z = ' , num2str(round(-1*Max_Force,0)), ' N', ', Time to SS = ' ,...
num2str(round(SS_Time,1)), ' s' , ', Max RL V = ' , num2str(round(Max_Line_Velocity,1)), ' m/s']);

subplot(3,1,2);
plot(D(3:end),Ddot(2:end),'g','LineWidth',2)
ylabel('Line Velocity[m/s]')
xlabel('Diameter [m]')
grid on

subplot(3,1,3);
plot(T,-1*ParachuteForce, 'r','LineWidth',2)
ylabel('Parachute Force [N]')
xlabel('Time[s]')
grid on
set(gcf,'color','w');

%% IGNORE BELOW IN CASE OF EMERGENCIES

%% Simulation Variables
% Disreefing Method
% Continuous_Reefing = 1;
% Bilinear_Disreefing_Profile = 0;
% 
% if Continuous_Reefing == 1
%     
%     % Continuous Disreefing Simulation Properties
%     RL_speed1 = 1; %Reefing Line (Circumference) growth rate [m/s]
%     D_speed1 = RL_speed1/pi; %Diameter Growth rate [m/s]
%     DR_Start1 = 0; %Time to start disreefing [s]
%     D_Start1 = 0; %Diameter to start Increase 1
%     
%     
%     if     Bilinear_Disreefing_Profile == 1
%         RL_speed2 = 5 - RL_speed1; %2nd Reefing Line (Circumference) growth rate [m/s]
%     else
%         RL_speed2 = 0; %Reefing Line (Circumference) growth rate [m/s]
%     end
%     D_speed2 = RL_speed2/pi; %2nd Diameter Growth rate [m/s]
%     DR_Start2 = 25; %Time to start 2nd disreefing [s]
%     D_Start2 = 10; %Diameter to start increase 2 [m]
%     
%     
% else
%     %Reef Cutter Simulation Properties
%     RL_Cutter_Opening_Time = 1;
%     RL_speed1 = (D_p*pi)/1; %Reefing Line (Circumference) growth rate [m/s]
%     D_speed1 = RL_speed1/pi; %Diameter Growth rate [m/s]
%     DR_Start1 = 0; %Time to start disreefing [s]
%     D_Start1 = 0;
%     
%     RL_speed2 = 0; %Reefing Line (Circumference) growth rate [m/s]
%     D_speed2 = 0; %2nd Diameter Growth rate [m/s]
%     DR_Start2 = 0; %Time to start 2nd disreefing [s]
%     D_Start2 = 0; %Diameter to start increase 2 [m]
%     
%     
%     
% end
% 


% if Continuous_Reefing == 1
%     display(['Continuous Disreefing Speed: ' , num2str(RL_speed1),' m/s', ', and after ', num2str(D_Start2), ' seconds: ', num2str(RL_speed2+RL_speed1),  ' m/s'])
% else
%     display(['Reefing Cutter'])
% end
