function [data_model,t,u,r,v,a] = load_data(time_offset,reset_time,path,date,time,points,graph)
%LOAD_DATA - Summary of this function goes here:
%   --> INPUT PARAMS:
%   @param[in]  time_offset: apply a time_offset to the data (not account
%   for x seconds in the beginning)
%   @param[in]  reset_time:  get new t_0 after applying time_offset
%   @param[in]  path:        path where the input .csv data is located
%   @param[in]  date:        input data date
%   @param[in]  time:        input data time
%   @param[in]  points:      discard points at the variables start and end
%   @param[in]  graph:       display plots (true/false)
%   --> OUTPUT PARAMS:
%   @param[out] data_model:  data object for SysId Toolbox
%   @param[out] t:           time vector
%   @param[out] u:           input vector (motors speed)
%   @param[out] r:           range vector
%   @param[out] v:           velocity vector
%   @param[out] a:           acceleration vector

% Constants:
OPTITRACK_OFFSET = 0.51; %[m]
SMOOTH_POINTS    = 120;  % 120 by default
SAMPLING_FREQ    = 1/30;

file = strcat(path,date,'-',time,'.csv');
disp(file)

% Read csv and apply time offset
M = csvread(file,1,0);
M = M(M(:,1) > time_offset, :);

% Reset time to zero after applying offset
if reset_time
    M(:,1) = M(:,1) - M(1,1);
end

% Extracting the needed variables
t = M(:,1);
u = M(:,3);
r2 = M(:,2) - OPTITRACK_OFFSET;                      % Apply OptiTrack offset
r = smooth(M(:,2),SMOOTH_POINTS) - OPTITRACK_OFFSET; % Apply OptiTrack offset + smooth with 120 points

v2 = diff(r2)./(1/30); 
v = diff(r)./(1/30);                                 % Compute velocity v (as derivative of r)
a = smooth(diff(r,2)./SAMPLING_FREQ,SMOOTH_POINTS);
a2 = diff(r,2)./(1/30);

% Discard some points at the beginning and end (remove weird peaks)
a  = a(points:end-points);
a2  = a2(points:end-points);
v2  = v2(points:end-1-points);
v  = v(points:end-1-points);
r  = r(points:end-2-points);
r2 = r2(points:end-2-points);
t  = t(points:end-2-points);
t  = t - t(1);                                        % Reinitialize time              
u  = u(points:end-2-points);

% Generating data_model object
data_model = iddata(v, u, SAMPLING_FREQ);

% Plotting variables
    if graph
        figure
        subplot(2,2,1)
        plot(t,r,t,r2,'b'); grid on
        xlim([0 max(t)])
        xlabel('Time [sec]')
        ylabel('Output r [m]')
        title('Range vs. Time')
        
        subplot(2,2,2)
        plot(t,v2,t,v,'g'); grid on
        xlim([0 max(t)])
        xlabel('Time [sec]')
        ylabel('Output v [m]')
        title('Speed vs. Time')
        
        subplot(2,2,3)
        plot(t,a2,t,a,'r'); grid on
        xlim([0 max(t)])
        xlabel('Time [sec]')
        ylabel('Output a [m]')
        title('Accel vs. Time')

        subplot(2,2,4)
        plot(t,u,'k'); grid on
        xlim([0 max(t)])
        xlabel('Time [sec]')
        ylabel('Input u')
        title('Motor speed vs. Time')
        
        figure
        plot(t,r,'b'); grid on; hold on
        plot(t,v,'g')
        plot(t,a,'r')
        plot(t,u,'k')
        
    end
end

