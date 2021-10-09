function [new,r_error] = calc_model_ran(t,u,r,tf,ic_cond,graphs)
%CALC_MODEL_VEL - Summary of this function goes here:
%   --> INPUT PARAMS:
%   @param[in]  t,u,r,v,tf: time, input, range, speed, transfer function
%                           (all vectors must have the same length)
%   --> OUTPUT PARAMS:
%   @param[out] new:   modeled range

% Constants
IC_CONDITION = ic_cond; % It could also be r(1).

% Get numerator + denominator of transfer function
num = tf.Numerator;
den = tf.Denominator;

% Declare 'new' vector size + initial condition
new = zeros(length(r)+1,1);
%new(1:max(length(den)-1,length(num))) = r(1:max(length(den)-1,length(num)));
%new(1:3) = 0;

% Applying the discrete model equation
for i = (max(length(den),length(num))+1):length(r)
    new(i) = -flip(den)*new((i-length(den)+1):i) + flip(num)*u((i-length(num)+1):i);
end

new = new(1:end-1);

% Apply condition to range (add mean)
new = new + IC_CONDITION;

% Compute the error
r_error = r-new;

if graphs
    figure
    plot(t,r,'-',t,new,'--'); grid on
    xlabel('Time [sec]')
    ylabel('Range [m]')
    legend('True range','Modeled range')
    title(['Range Model -> Range. TF with ', num2str(length(pole(tf))), 'p and ', num2str(length(zero(tf))),'z'])
    %figure
    %plot(t,u)
end
end

