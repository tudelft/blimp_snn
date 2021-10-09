function [new,r_int,r_error] = calc_model_vel(t,u,r,v,tf,graphs)
%CALC_MODEL_VEL - Summary of this function goes here:
%   --> INPUT PARAMS:
%   @param[in]  t,u,r,v,tf: time, input, range, speed, transfer function
%                           (all vectors must have the same length)
%   --> OUTPUT PARAMS:
%   @param[out] new:   modeled velocity
%   @param[out] r_int: modeled range by integrating the modeled velocity

% Get numerator + denominator of transfer function
num = tf.Numerator;
den = tf.Denominator;

% Declare 'new' vector size + initial condition
new = zeros(length(v)+1,1);
new(1:max(length(den)-1,length(num))) = v(1:max(length(den)-1,length(num)));
%new(1:3) = 0;

% Applying the discrete model equation
for i = (max(length(den),length(num))+1):length(v)
    new(i) = -flip(den)*new((i-length(den)+1):i) + flip(num)*u((i-length(num)+1):i);
end
new = new(1:end-1);

% Compute range by integrating velocity and add IC condition r(1)
r_int = r(1) + cumtrapz(t, new);

% Compute the error
r_error = r-r_int;

% Plots
if graphs
    figure
    subplot(2,1,1)
    plot(t,v,'-',t,new,'--'); grid on
    xlabel('Time [sec]')
    ylabel('Velocity [m/s]')
    legend('True speed','Modeled speed')
    title(['Velocity Model -> Speed. TF with ', num2str(length(pole(tf))), 'p and ', num2str(length(zero(tf))),'z'])

    %figure
    %plot(t,u)

    %figure
    subplot(2,1,2)
    plot(t,r,'-',t,r_int,'--'); grid on
    xlabel('Time [sec]')
    ylabel('Range [m]')
    legend('True range','Modeled range')
    title(['Velocity Model -> Range. TF with ', num2str(length(pole(tf))), 'p and ', num2str(length(zero(tf))),'z'])
end

end

