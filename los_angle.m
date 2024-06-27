function theta = los_angle(waypoints,x,y,k,total,Lnpp)
    if k < 1 || k > total
    error("航路点错误");
    end
    if k == 1
    point_k_1 =  waypoints(k,:);
    point_k = [30,30];
    fai_k = atan2(point_k_1(1) - point_k(1),point_k_1(2) - point_k(2));
    err = -(y - point_k(2))*sin(fai_k)+(x - point_k(1))*cos(fai_k);
    beta = atan2(err,Lnpp);
    
    theta =pi/2-(fai_k-beta);
    
    else
    point_k_1 =  waypoints(k,:);
    point_k = waypoints(k-1,:);
    fai_k = atan2(point_k_1(1) - point_k(1),point_k_1(2) - point_k(2));
    err = -(y - point_k(2))*sin(fai_k)+(x - point_k(1))*cos(fai_k);
    beta = atan2(err,Lnpp);
    
    theta =pi/2-(fai_k-beta);
    
    end
    
end