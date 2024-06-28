function [Vk] = adative_velocity(waypoints,Vmax,Vmin,sigmaR,total)
%ADATIVE_RADIUS_SPEED 此处显示有关此函数的摘要
%   此处显示详细说明

Vk = zeros(total-2,1);
angle_k = angle_calc(waypoints,total);
for i = 1:1:total-2
    Vk(i,1) = Vmin+(Vmax-Vmin)*exp(-power(angle_k(i),2)/power(sigmaR,2));
end
