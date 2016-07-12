clc; clear all; close all;
%% Question A - Finding Joint Trajectories
%% Initial Conditions
L = 0.65; D = 0.75; t_step = 40e-3; g = 9.81;

%Position of Center of Mass(COM)
x_com(1) =0; y_com(1) = 2*L;

th_hr(1) = 0; th_kr(1) = 0; %Chosing clockwise direction as +ve
th_hl(1) = 30; th_kl(1) = (360-60); % Chosing anticlockwise direction as +ve

%Velocity of COM
vx_com(1) = 1; vy_com(1) = 0;

% From the expression of vx_com we get 1/L = w_kr + 2*w_hr for initial condition
% Thus assuming w_kr = 0, we get w_hr = 1/2*L
w_hr(1) = 1/(2*L); w_kr(1) = 0;

%Putting initial conditions in the ZMP equation we get acc_kr +2acc_hr = 0
%But choosing acc_hr(1) = 0.1; acc_kr(1) = -2*acc_hr(1); gives -ve value
%for w_hr which will mean that after the knee is at rest at w_hr(1) = 0 it will start
%rotating in anti-clockwise direction instead of clockwise, which is not
%possible. Hence we choose
acc_hr(1) = 0; acc_kr(1) = 0;

%By putting the initial values we get ay_com = -1/2L
ax_com(1) = 0; ay_com(1) = -1/(2*L);

w_hr(2) = w_hr(1) + acc_hr(1)*t_step;
w_kr(2) = w_kr(1) + acc_kr(1)*t_step; 
acc_hr(2) = (w_hr(2) - w_hr(1))/t_step;
acc_kr(2) = (w_kr(2) - w_kr(1))/t_step;

th_hr(2) = th_hr(1) + w_hr(1)*t_step;
th_kr(2) = th_kr(1) + w_kr(1)*t_step;

x_com(2) = L*( sin(th_kr(2)+ th_hr(2)) + sin(th_hr(2)));
y_com(2) = L*( cos(th_kr(2)+ th_hr(2)) + cos(th_hr(2)));

%Assuming that the left leg touches the ground when the COM is at the
%center of the step length
%Also it is assumed that the left leg is straight when it touches the
%ground
H = ((2*L)^(2) - (0.75/2)^(2))^(1/2);

i = 3;

%% Calculating the trajectory of th_hr and th_kr when the right leg is in contact with the ground

while( x_com(i-1) <= (0.75/2))
    
    th_hr(i) = sind(i);
    w_hr(i) = (th_hr(i) - th_hr(i-1))/t_step;
    acc_hr(i) = (w_hr(i) - w_hr(i-1))/t_step;
    
    %From the formula for ZMP we find acc_kr, we get
    w_kr(i) = w_kr(i-1) + acc_kr(i-1)*t_step;
    th_kr(i) = th_kr(i-1) + w_kr(i-1)*t_step;
    
    
    %acc_kr(i) = ( (sin(th_kr(i)) + sin(th_hr(i)))*( -acc_hr(i)*sin(th_kr(i)) - ((w_kr(i) + w_hr(i))^(2))*(cos(th_kr(i))) - acc_hr(i)*sin(th_hr(i)) - ((w_hr(i))^(2))*cos(th_hr(i)) + g/L )...
    %         - (cos(th_kr(i)) + cos(th_hr(i)))*( acc_hr(i)*cos(th_kr(i)) - ((w_kr(i) + w_hr(i))^(2))*( sin(th_kr(i))) - acc_hr(i)*cos(th_hr(i)) - ((w_hr(i))^(2))*sin(th_hr(i)) ) )/ ( cos(th_kr(i))*(cos(th_kr(i)) + cos(th_hr(i))) + sin(th_kr(i))*( sin(th_kr(i)) + sin(th_hr(i)) ) );
    
    acc_kr(i) = ( (sin(th_kr(i) + th_hr(i)) + sin(th_hr(i)))*( -acc_hr(i)*sin(th_kr(i) + th_hr(i)) - ((w_kr(i) + w_hr(i))^(2))*(cos(th_kr(i) + th_hr(i))) - acc_hr(i)*sin(th_hr(i)) - ((w_hr(i))^(2))*cos(th_hr(i)) + g/L )...
              - (cos(th_kr(i) + th_hr(i)) + cos(th_hr(i)))*( acc_hr(i)*cos(th_kr(i) + th_hr(i)) - ((w_kr(i) + w_hr(i))^(2))*(sin(th_kr(i) + th_hr(i))) - acc_hr(i)*cos(th_hr(i)) + ((w_hr(i))^(2))*sin(th_hr(i)) ) )/ ( cos(th_kr(i) + th_hr(i))*(cos(th_kr(i) + th_hr(i)) + cos(th_hr(i))) + sin(th_kr(i) + th_hr(i))*( sin(th_kr(i) + th_hr(i)) + sin(th_hr(i)) ) );
    
    % Now we can also calculate the position,velocity and acceleration of center of mass
    x_com(i) = L*( sin(th_kr(i)+ th_hr(i)) + sin(th_hr(i)));
    y_com(i) = L*( cos(th_kr(i)+ th_hr(i)) + cos(th_hr(i)));
    
    vx_com(i) =  L*((w_kr(i) +w_hr(i))*cos(th_kr(i) + th_hr(i)) + w_hr(i)*cos(th_hr(i)));
    %The negative sign here means the velocity is in downwards direction
    vy_com(i) = -L*((w_kr(i) +w_hr(i))*sin(th_kr(i) + th_hr(i)) +w_hr(i)*sin(th_hr(i)));
    
    ax_com(i) =  L *((acc_kr(i) + acc_hr(i))*cos(th_kr(i))  -(w_kr(i)+w_hr(i))^2*sin(th_kr(i))  +acc_hr(i)*cos(th_hr(i))  -w_hr(i)^2*sin(th_hr(i)) );
    ay_com(i) = -L *((acc_kr(i) + acc_hr(i))*sin(th_kr(i))  +(w_kr(i)+w_hr(i))^2*cos(th_kr(i))  +acc_hr(i)*sin(th_hr(i))  +w_hr(i)^2*cos(th_hr(i)) );
    
    i = i + 1;
    
end

%% Trajectory for angles of left leg, assuming that the th_hl and th_kl vary linearly till they touch the ground

%The hip angle varies from 30 degrees to 16.75 degrees
for j = 1:14
    %Using formula y - y1 = m(x - x1)
    th_hl(j) = (30 + ((16.75-30)/(14-1))*(j-1))*(pi/180);
    if j==1
        w_hl(j) = 0;
        w_kl(j) = 0;
        acc_kl = 0;
        acc_hl = 0;
    end
    if j > 1
        w_hl(j) = (th_hl(j) - th_hl(j-1))/t_step;
        acc_hl(j) = (w_hl(j) - w_hl(j-1))/t_step;
    end
    %Since we are taking the anti-clockwise direction as positive the th_kl
    %varies from (360-60) to (360 - 0)
    th_kl(j) = (300 + ((360 - 300)/(14-1))*(j-1))*(pi/180);
    if j > 1
        w_kl(j) = (th_kl(j) - th_kl(j-1))/t_step;
        acc_kl(j) = (w_kl(j) - w_kl(j-1))/t_step;
    end
    
end

%% At this point both the legs are in contact with the ground

%Assuming that the right leg loses contact with the ground at this point
%and the body moves to left leg on single point contact support


while( th_hl(i-1) >= 0 )
    th_hl(i) = (16.75 + ((0 - 16.75)/(39-14))*(i-14))*(pi/180);
    
    %code to calculate the th_kl
    w_hl(i) = (th_hl(i) - th_hl(i-1))/t_step;
    acc_hl(i) = (w_hl(i) - w_hl(i-1))/t_step;
    
    w_kl(i) = w_kl(i-1) + acc_kl(i-1)*t_step;
    th_kl(i) = th_kl(i-1) + w_kl(i-1)*t_step;
    
    if th_kl(i) > (360*pi/180)
        th_kl(i) = (360*pi/180);
    end
    
    acc_kl(i) = ( (sin(th_kl(i) + th_hl(i)) + sin(th_hl(i)))*( -acc_hl(i)*sin(th_kl(i) + th_hl(i)) - ((w_kl(i) + w_hl(i))^(2))*(cos(th_kl(i) + th_hl(i))) - acc_hl(i)*sin(th_hl(i)) - ((w_hl(i))^(2))*cos(th_hl(i)) + g/L )...
              - (cos(th_kl(i) + th_hl(i)) + cos(th_hl(i)))*( acc_hl(i)*cos(th_kl(i) + th_hl(i)) - ((w_kl(i) + w_hl(i))^(2))*(sin(th_kl(i) + th_hl(i))) - acc_hl(i)*cos(th_hl(i)) + ((w_hl(i))^(2))*sin(th_hl(i)) ) )/ ( cos(th_kl(i) + th_hl(i))*(cos(th_kl(i) + th_hl(i)) + cos(th_hl(i))) + sin(th_kl(i) + th_hl(i))*( sin(th_kl(i) + th_hl(i)) + sin(th_hl(i)) ) );
    
    
    x_com(i) = 0.75 - L*( sin(- th_kl(i)+ th_hl(i)) + sin(th_hl(i)));
    y_com(i) = L*( cos( th_kl(i)+ th_hl(i)) + cos(th_hl(i)));
    
    vx_com(i) =  L*((w_kl(i) +w_hl(i))*cos(th_kl(i) + th_hl(i)) + w_hl(i)*cos(th_hl(i)));
    vy_com(i) = -L*((w_kl(i) +w_hl(i))*sin(th_kl(i) + th_hl(i)) +w_hl(i)*sin(th_hl(i)));
    
    ax_com(i) =  L *((acc_kl(i) + acc_hl(i))*cos(th_kl(i))  -(w_kl(i)+w_hl(i))^2*sin(th_kl(i))  +acc_hl(i)*cos(th_hl(i))  -w_hl(i)^2*sin(th_hl(i)) );
    ay_com(i) = -L *((acc_kl(i) + acc_hl(i))*sin(th_kl(i))  +(w_kl(i)+w_hl(i))^2*cos(th_kl(i))  +acc_hl(i)*sin(th_hl(i))  +w_hl(i)^2*cos(th_hl(i)) );
    
    i = i + 1;
    
end


%% We assume angles of right leg vary linearly till the final position

for k = (j):i
    th_hr(k) = th_hr(j) + (((-30*pi/180) - th_hr(j))/(i - (j)))*(k - (j));
    w_hr(k) = (th_hr(k) - th_hr(k-1))/t_step;
    acc_hr(k) = (w_hr(k) - w_hr(k-1))/t_step;
    
    th_kr(k) = th_kr(j) + (((60*pi/180) - th_kr(j))/(i - (j)))*(k - (j));
    w_kr(k) = (th_kr(k) - th_kr(k))/t_step;
    acc_kr(k) = (w_kr(k) - w_kr(k))/t_step;
end
    
    

figure(1)
x_offset = 0;
for z = 1:3
for s = 1:i-1
    plot(x_com(s)+x_offset,y_com(s),'ob','MarkerSize',9)
    axis([0 3 0 3])
    hold on;
    
    %right knee position
    x_kr = x_com(s)+x_offset - L*sin(th_hr(s));
    y_kr = y_com(s) - L*cos(th_hr(s));
    
    plot(x_kr, y_kr, 'or','MarkerSize',9);
    
    %Drawing line from the hip to right knee
    line([x_com(s)+x_offset x_kr],[y_com(s) y_kr],'Color','k','Linewidth',4);
    
    
    %right foot position
    x_fr = x_kr - L*sin(th_hr(s)+ th_kr(s));
    y_fr = y_kr - L*cos(th_hr(s)+ th_kr(s));
    
    %Drawing line from the knee to the right foot
    line([x_kr x_fr],[y_kr y_fr],'Color','k','Linewidth',4);
    
    %left knee position
    x_kl = x_com(s)+x_offset + L*sin(th_hl(s));
    y_kl = y_com(s) - L*cos(th_hl(s));
    
    plot(x_kl, y_kl, 'or','MarkerSize',9);
    
    %drawing line from the hip to left knee
    line([x_com(s)+x_offset x_kl],[y_com(s) y_kl],'Color','k','Linewidth',4);
    
    %left foot position
    x_fl = x_kl + L*sin(th_hl(s)+ th_kl(s));
    y_fl = y_kl - L*cos(th_hl(s)+ th_kl(s));
    
    %Drawing line from left knee to the left foot
    line([x_kl x_fl],[y_kl y_fl],'Color','k','Linewidth',4);
    
    
    pause(0.25);
    clf;
end
x_offset = x_offset + x_com(s);
end



