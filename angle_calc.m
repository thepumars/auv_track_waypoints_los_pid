function [angle_k] = angle_calc(waypoints,total)
angle_k = zeros(total-2,1);
for i = 1:1:total-2
    pointk_2 = waypoints(i+2,:);
    pointk_1 = waypoints(i+1,:);
    pointk = waypoints(i,:);
    alpha2 = atan2(pointk_2(1) - pointk_1(1),pointk_2(2) - pointk_1(2));
    alpha1 = atan2(pointk_1(1) - pointk(1),pointk_1(2) - pointk(2));
    alpha = alpha2-alpha1;
    angle_k(i)=alpha;
end
end

