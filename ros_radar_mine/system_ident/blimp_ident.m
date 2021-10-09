clear all; close all; clc

%% 1) LOAD DATA

% Constants + File path
time_offset = 0; % [sec]
reset_time  = true;
path        = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_';
points      = 100;
SAMPLE_FREQ = 1/30;

% Data with [-3, -2, -1, 0, 1, 2, 3] inputs

[data1,t1,u1,r1,v1,a1] = load_data(time_offset,reset_time,path,'2021-02-01','13-44-01',points,false);
[data2,t2,u2,r2,v2,a2] = load_data(time_offset,reset_time,path,'2021-02-01','11-24-25',points,false);
[data3,t3,u3,r3,v3,a3] = load_data(time_offset,reset_time,path,'2021-02-01','11-45-40',points,false);
[data4,t4,u4,r4,v4,a4] = load_data(time_offset,reset_time,path,'2021-02-01','11-40-50',points,false);
[data5,t5,u5,r5,v5,a5] = load_data(time_offset,reset_time,path,'2021-02-01','13-52-42',points,true);
[data6,t6,u6,r6,v6,a6] = load_data(time_offset,reset_time,path,'2021-02-01','14-02-37',points,false);

% Train on 5, validate on 6. Tf2 with 3 poles and 1 zero is the best one

%% 2) BUILD MODEL

% Open Application Here OR Load Calculated TFs
%load('mytfs_acc')
load('mytfs_vel')
load('mytfs_ran')

%% 3) CALCULATE DISCRETE TRANSFER FUNCTION
% Sample time: 1/30
tf_vel = c2d(tf_vel,SAMPLE_FREQ); % c2d(cont-TF, sample time)
tf_ran = c2d(tf_ran,SAMPLE_FREQ); % c2d(cont-TF, sample time)

%% 4) CALCULATE MODEL OUTPUT FROM DISC TF

number = 6;
t = eval(sprintf(['t',num2str(number)]));
u = eval(sprintf(['u',num2str(number)]));
r = eval(sprintf(['r',num2str(number)]));
v = eval(sprintf(['v',num2str(number)]));

[v_modelV,r_modelV,errorV] = calc_model_vel(t,u,r,v,tf_vel,false);
[r_modelR,errorR]          = calc_model_ran(t,u,r,tf_ran,2,true);

%% 5) COMPUTE ERROR FOR BOTH MODELS

number_list = 1:6;

rmseR_vec = zeros(length(number_list),1);
rmseV_vec = zeros(length(number_list),1);

ic_list = [1.6, 1.8, 1.5, 1.5, 1.7, 2];

for i = 1:length(number_list)
    
    number = number_list(i);
    t = eval(sprintf(['t',num2str(number)]));
    u = eval(sprintf(['u',num2str(number)]));
    r = eval(sprintf(['r',num2str(number)]));
    v = eval(sprintf(['v',num2str(number)]));
    
    [v_modelV,r_modelV,errorV] = calc_model_vel(t,u,r,v,tf_vel,false);
    [r_modelR,errorR]          = calc_model_ran(t,u,r,tf_ran,ic_list(i),false);
    
    rmseV_vec(i) = sqrt(mean(errorV.^2));
    rmseR_vec(i) = sqrt(mean(errorR.^2));
    
    %{
    figure(1)
    boxplot(errorV,'Labels',{num2str(i)}); hold on; grid on
    
    figure(2)
    boxplot(errorR); hold on; grid on
    %}
    
end
%%
figure
plot(number_list,rmseV_vec,'-o',number_list,rmseR_vec,'-o'); grid on
xlabel('Dataset Number')
ylabel('Range RMSE')
legend('Velocity Model','Range Model')
title('Range RMSE for both cases: velocity + range model')
%%
figure
plot(number_list,rmseR_vec,'-o'); grid on
xlabel('Dataset Number')
ylabel('Range RMSE')
legend('Range Model')
title('Range RMSE for range model')
%%
x = [0.36,0.3,0.35,0.29,0.34,0.31];
disp(num2str(mean(x)))
disp(num2str(std(x)))










