% AER525H - ROBOTICS ASSIGNMENT 4

% By Naphish Lashba Raj (1002083791)
% Date: 12/01/2015

% We first declare the parameters

% These are the link parameters for the robotic arm in (m)
l1=1;       
l2=0.5;     
m1=2;       
m2=1;       

% The Joint angle limits are given as
theta1_min = -90;
theta1_max = +90;
theta2_min = 0;
theta2_max = 360;

% The end effector postions are given as
Pa = [-0.3 0.8];
Pb = [1.4 0];

% Other parameters
tf = 4;
tb = tf/4;
t = linspace(0,tf,200);
g = -10;   

% Preallocate matrices for all values to be plotted:
Pendx_vals = zeros(2,length(t));
Pendy_vals = zeros(2,length(t));
Pendx_dot_vals = zeros(2,length(t));
Pendy_dot_vals = zeros(2,length(t));
Pendx_ddot_vals = zeros(2,length(t));
Pendy_ddot_vals = zeros(2,length(t));

torque1_vals = zeros(2,length(t));
torque2_vals = zeros(2,length(t));
theta1_vals = zeros(2,length(t));
theta2_vals = zeros(2,length(t));
theta1_dot_vals = zeros(2,length(t));
theta2_dot_vals = zeros(2,length(t));
theta1_ddot_vals = zeros(2,length(t));
theta2_ddot_vals = zeros(2,length(t));

% Solving the forward kinematics problem:
% for a 0-3 transformation matrix:
P3x= @(theta1,theta2) l1*cosd(theta1)+l2*cosd(theta1+theta2);
P3y= @(theta1,theta2) l1*sind(theta1)+l2*sind(theta1+theta2);

% Solving the inverse kinemtics problem:
C2 = @(x,y) (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
s2 = @(x,y) sqrt(1-C2(x,y)^2);

% Equations for the Joint angles are given according to the example solved:
theta1 = @(x,y) atan2d(y,x)-atan2d(l2*S2(x,y), l1+l2*C2(x,y));
theta2 = @(x,y) atan2d(S2(x,y), C2(x,y));

% To find the torque equations we use the following:
torque1 = @(x,y,theta1_dot,theta1_ddot,theta2_dot,theta2_ddot) ...
    m2*l2^2*(theta1_ddot+theta2_ddot)+m2*l1*l2*C2(x,y)*(2*theta1_ddot+theta2_ddot)...
    +(m1+m2)*l1^2*theta1_ddot-m2*l1*l2*S2(x,y)*theta1_dot^2 ...
    -2*m2*l1*l2*S2(x,y)*theta1_dot*theta2_dot ...
    +m2*l2*g*cosd(theta1(x,y)+theta2(x,y)) ...
    +(m1+m2)*l1*g*cosd(theta1(x,y));

torque2 = @(x,y,theta1_dot,theta1_ddot,theta2_dot,theta2_ddot) ...
    m2*l1*l2*cosd(theta2(x,y))*theta1_ddot ...
    +m2*l1*l2*sind(theta2(x,y))*theta1_dot^2 ...
    +m2*l2*g*cosd(theta1(x,y)+theta2(x,y)) ...
    +m2*l2^2*(theta1_ddot+theta2_ddot);

% Now we obtain the initial and final values of theta using inverse
% kinematics with Pa and Pb representing the respective positions
theta1_i = theta1(Pa(1),Pa(2));
theta1_f = theta1(Pb(1),Pb(2));
theta2_i = theta2(Pa(1),Pa(2));
theta2_f = theta2(Pb(1),Pb(2));

% 1) JOINT SPACE

% Joint Space using parabolic blends
theta1_ddot = (theta1_f-theta1_i)/((tf^2/4)-(0.5*tf-tb)^2);
theta2_ddot = (theta2_f-theta2_i)/((tf^2/4)-(0.5*tf-tb)^2);

% Joint rates given in the blend regions
theta1_dot_bi = @(t) theta1_ddot*t;       
theta1_dot_bf = @(t) theta1_ddot*(tf-t);  
theta2_dot_bi = @(t) theta2_ddot*t;       
theta2_dot_bf = @(t) theta2_ddot*(tf-t);  

% Joint rates given in the linear section
theta1_dot_lin = (0.5*theta1_f-theta1_i)/(0.5*tf-tb);
theta2_dot_lin = (0.5*theta2_f-theta2_i)/(0.5*tf-tb);

% Now we create a loop for the function through certain number of time
% steps
for i=1:length(t)
    
    % Taking all the parameters seperately
    
    % The Initial blend
    if t(i) <= tb
        
        % The Joint rates
        theta1_dot_vals(1,i) = theta1_dot_bi(t(i));
        theta2_dot_vals(1,i) = theta2_dot_bi(t(i));
        
        % The Joint accelerations
        theta1_ddot_vals(1,i) = theta1_ddot;
        theta2_ddot_vals(1,i) = theta2_ddot;
        
        % The Joint angles
        theta1_vals(1,i) = theta1_i+0.5*theta1_ddot_vals(1,i)*t(i)^2;
        theta2_vals(1,i) = theta2_i+0.5*theta2_ddot_vals(1,i)*t(i)^2;
        
        blend_complete = i;
        
    % Linear section
    elseif t(i)>tb && t(i)<(tf-tb)
        
        % The Joint rates
        theta1_dot_vals(1,i) = (0.5*(theta1_f+theta1_i)-theta1_vals(1,blend_complete))/(0.5*tf-t(blend_complete));
        theta2_dot_vals(1,i) = (0.5*(theta2_f+theta2_i)-theta2_vals(1,blend_complete))/(0.5*tf-t(blend_complete));
        
        % The Joint accelerations were already preallocated to zero
        
        % The Joint angles
        theta1_vals(1,i) = theta1_vals(1,blend_complete)+theta1_dot_vals(1,i)*(t(i)-tb);
        theta2_vals(1,i) = theta2_vals(1,blend_complete)+theta2_dot_vals(1,i)*(t(i)-tb);
        
        blend_initial = i;
        
    % The Final blend
    else
        % The Joint rates
        theta1_dot_vals(1,i) = theta1_dot_bf(t(i));
        theta2_dot_vals(1,i) = theta2_dot_bf(t(i));
        
        % The Joint accelerations
        theta1_ddot_vals(1,i) = -theta1_ddot;
        theta2_ddot_vals(1,i) = -theta2_ddot;
    
        % The Joint angles
        theta1_vals(1,i) = theta1_vals(1,blend_initial)+0.5*theta1_ddot_vals(1,i)*((tf-t(i))^2-1);
        theta2_vals(1,i) = theta2_vals(1,blend_initial)+0.5*theta2_ddot_vals(1,i)*((tf-t(i))^2-1);
        
    end
    
    % Considering the end effector positions
    Pendx_vals(1,i) = Pendx(theta1_vals(1,i),theta2_vals(1,i));
    Pendy_vals(1,i) = Pendy(theta1_vals(1,i),theta2_vals(1,i));
    
    % Joint torques
    torque1_vals(1,i) = torque1(Pendx_vals(1,i),Pendy_vals(1,i),theta1_dot_vals(1,i), ...
        theta1_ddot_vals(1,i),theta2_dot_vals(1,i),theta2_ddot_vals(1,i));
    
    torque2_vals(1,i) = torque2(Pendx_vals(1,i),Pendy_vals(1,i),theta1_dot_vals(1,i), ...
        theta1_ddot_vals(1,i),theta2_dot_vals(1,i),theta2_ddot_vals(1,i));
    
end

% 2) CARTESIAN SPACE

% End effector acceleration from blend time equation (same as in joint space)
Pendx_ddot = (Pb(1)-Pa(1))/((tf^2/4)-(0.5*tf-tb)^2);
Pendy_ddot = (Pb(2)-Pa(2))/((tf^2/4)-(0.5*tf-tb)^2);

% End effector velocities in blend regions (same as in joint space)
Pendx_dot_bi = @(t) Pendx_ddot*t;       
Pendx_dot_bf = @(t) Pendx_ddot*(tf-t);  
Pendy_dot_bi = @(t) Pendy_ddot*t;       
Pendy_dot_bf = @(t) Pendy_ddot*(tf-t);  

% End effector Joint rates in linear section (same as in joint space)
Pendx_dot_lin = (0.5*Pb(1)-Pa(1))/(0.5*tf-tb);
Pendy_dot_lin = (0.5*Pb(2)-Pb(1))/(0.5*tf-tb);

for i=1:length(t)
        
    % The Initial blend
    if t(i) <= tb
        
        % The End effector rates
        Pendx_dot_vals(2,i) = Pendx_dot_bi(t(i));
        Pendy_dot_vals(2,i) = Pendy_dot_bi(t(i));
        
        % The End effector accelerations
        Pendx_ddot_vals(2,i) = Pendx_ddot;
        Pendy_ddot_vals(2,i) = Pendy_ddot;
        
        % The End effector positions
        Pendx_vals(2,i) = Pa(1)+0.5*Pendx_ddot_vals(2,i)*t(i)^2;
        Pendy_vals(2,i) = Pa(2)+0.5*Pendy_ddot_vals(2,i)*t(i)^2;
        
        blend_complete = i;
        
    % Linear section
    elseif t(i)>tb && t(i)<(tf-tb)
        
        % The End effector velocity
        Pendx_dot_vals(2,i) = (0.5*(Pb(1)+Pa(1))-Pendx_vals(2,blend_complete))/(0.5*tf-t(blend_complete));
        Pendy_dot_vals(2,i) = (0.5*(Pb(2)+Pa(2))-Pendy_vals(2,blend_complete))/(0.5*tf-t(blend_complete));

        % The End effector accelerations were already preallocated to zero
        
        % The End effector position
        Pendx_vals(2,i) = Pendx_vals(2,blend_complete)+Pendx_dot_vals(2,i)*(t(i)-tb);
        Pendy_vals(2,i) = Pendy_vals(2,blend_complete)+Pendy_dot_vals(2,i)*(t(i)-tb);
        
        
        
        blend_initial = i;
        
    % Final blend
    else
        
        % The End effector velocity
        Pendx_dot_vals(2,i) = Pendx_dot_bf(t(i));
        Pendy_dot_vals(2,i) = Pendy_dot_bf(t(i));
        
        % The End effector accels
        Pendx_ddot_vals(2,i) = -Pendx_ddot;
        Pendy_ddot_vals(2,i) = -Pendy_ddot;
    
        % The End effector position
        Pendx_vals(2,i) = Pendx_vals(2,blend_initial)+0.5*Pendx_ddot_vals(2,i)*((tf-t(i))^2-1);
        Pendy_vals(2,i) = Pendy_vals(2,blend_initial)+0.5*Pendy_ddot_vals(2,i)*((tf-t(i))^2-1);
        
    end 

    % Using Inverse Kinematics
    theta1_vals(2,i) = theta1(Pendx_vals(2,i),Pendy_vals(2,i));
    theta2_vals(2,i) = theta2(Pendx_vals(2,i),Pendy_vals(2,i));
    
    if i == 1
        theta1_dot_vals(2,i) = theta1_vals(2,i)/t(i);
        theta2_dot_vals(2,i) = theta2_vals(2,i)/t(i);
        theta1_ddot_vals(2,i) = theta1_dot_vals(2,i)/t(i);
        theta2_ddot_vals(2,i) = theta2_dot_vals(2,i)/t(i);

    else
        theta1_dot_vals(2,i)=(theta1_vals(2,i)-theta1_vals(2,i-1))/(t(i)-t(i-1));
        theta2_dot_vals(2,i)=(theta2_vals(2,i)-theta2_vals(2,i-1))/(t(i)-t(i-1));
        theta1_ddot_vals(2,i)=(theta1_dot_vals(2,i)-theta1_dot_vals(2,i-1))/(t(i)-t(i-1));
        theta2_ddot_vals(2,i)=(theta2_dot_vals(2,i)-theta2_dot_vals(2,i-1))/(t(i)-t(i-1));
    end

    % The Joint Torques are given as:
    torque1_vals(2,i) = torque1(Pendx_vals(2,i),Pendy_vals(2,i),theta1_dot_vals(2,i), ...
        theta1_ddot_vals(2,i),theta2_dot_vals(2,i),theta2_ddot_vals(2,i));
    
    torque2_vals(2,i) = torque2(Pendx_vals(2,i),Pendy_vals(2,i),theta1_dot_vals(2,i), ...
        theta1_ddot_vals(2,i),theta2_dot_vals(2,i),theta2_ddot_vals(2,i));

end

% PLOTTING THE GRAPHS FOR JOINT SPACE:

figure(1)
plot (Pendx_vals(1,:), Pendy_vals(1,:), 'b')
title('The End Effector postions With Joint Space Planning')
xlabel('X Position [m]')
ylabel('Y Position [m]')
axis([-0.5 1.5 -0.5 1.5])

figure(2)
plot(t,theta1_vals(1,:), 'b--', t, theta2_vals(1,:), 'r.')
title('The Joint angles With Joint Space Planning')
xlabel('Time [sec]')
ylabel('Joint Position [Deg]')
axis([0 tf -90 360])
legend('Joint 1', 'Joint 2')

figure(3)
plot(t,theta1_dot_vals(1,:), 'b--', t, theta2_dot_vals(1,:), 'r.')
title('The Joint Velocities With Joint Space Planning')
xlabel('Time [sec]')
ylabel('Joint Rate [Deg/sec]')
axis([0 tf -100 100])
legend('Joint 1', 'Joint 2')

figure(4)
plot(t,theta1_ddot_vals(1,:), 'b--', t, theta2_ddot_vals(1,:), 'r.')
title('The Joint Accelerations With Joint Space Planning')
xlabel('Time [sec]')
ylabel('Joint Acceleration [Deg/sec^2]')
axis([0 tf -100 100])
legend('Joint 1', 'Joint 2')

figure(5)
plot(t,torque1_vals(1,:), 'b--', t, torque2_vals(1,:), 'r.')
title('Joint Torques With Joint Space Planning')
xlabel('Times [sec]')
ylabel('Joint Torques [Nm]')
axis([0 tf -2000 2000])
legend('Joint 1', 'Joint 2')

% PLOTTING GRAPHS FOR CARTESIAN SPACE

figure(6)
plot(Pendx_vals(2,:), Pendy_vals(2,:), 'b')
title('The End Effector Positions With Cartesian Space Planning')
xlabel('X-Position [m]')
ylabel('Y-Position [m]')
axis([-0.5 1.5 -0.5 1.5])

figure(7)
plot(t,theta1_vals(2,:), 'b--', t, theta2_vals(2,:), 'r.')
title('Joint Positions With Cartesian Space Planning')
xlabel('Time [sec]')
ylabel('Joint Position [Deg]')
axis([0 tf -90 360])
legend('Joint 1', 'Joint 2')

figure(8)
plot(t,theta1_dot_vals(2,:), 'b--', t, theta2_dot_vals(2,:), 'r.')
title('Joint Rates With Cartesian Space Planning')
xlabel('Time [sec]')
ylabel('Joint Rate [Deg/sec]')
axis([0 tf -100 100])
legend('Joint 1', 'Joint 2')

figure(9)
plot(t,theta1_ddot_vals(2,:), 'b--', t, theta2_ddot_vals(2,:), 'r.')
title('Joint Accelerations With Cartesian Space Planning')
xlabel('Time [sec]')
ylabel('Joint Acceleration [Deg/sec^2]')
axis([0 tf -100 100])
legend('Joint 1', 'Joint 2')

figure(10)
plot(t,torque1_vals(2,:), 'b--', t, torque2_vals(2,:), 'r.')
title('Joint Torques With Cartesian Space Planning')
xlabel('Times [sec]')
ylabel('Joint Torques [Nm]')
axis([0 tf -2000 2000])
legend('Joint 1', 'Joint 2')