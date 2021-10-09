function calc_model_acc(t,u,r,v,a,tf)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

num = tf.Numerator;
den = tf.Denominator;

new = zeros(length(a)+1,1);
new(1:max(length(den)-1,length(num))) = a(1:max(length(den)-1,length(num)));
%new(1:3) = 0;

for i = (max(length(den),length(num))+1):length(a)
%for i = 4:length(r1)
    
    new(i) = -flip(den)*new((i-length(den)+1):i) + flip(num)*u((i-length(num)+1):i);
    %new(i) = -(den(2)*new_y1(i-1)+den(3)*new_y1(i-2)+den(4)*new_y1(i-3))+(num(2)*u1(i-1)+num(3)*u1(i-2)+num(4)*u1(i-3));
end

new = new(1:end-1);

figure
plot(t,a,t,new); grid on
%figure
%plot(t,u)

figure
v_int = cumtrapz(t, new)+v(1); grid on
plot(t,v,t,v_int)

figure
r_int = cumtrapz(t, v_int)+r(1); grid on
plot(t,r,t,r_int)

end

