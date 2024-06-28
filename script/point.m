function [isend,k,velocity] = point(way_points,x,y,k,total,R0,V0)
    isend = 0;
    point_k = way_points(k,:); 
    point_n = [x,y];
    
    r = norm(point_k-point_n);
    if k == 1
        R = 100;%rmax
        velocity = 2;
    elseif k == total
        R = 50;%rmin
        velocity = 1;
    else
        R = R0(k-1);
        velocity = V0(k-1);
    end
    
    if(r <= R) 
        disp(["到达航路点,",k]);
        if(k==total)
            disp("任务结束...");
            isend = 1;
        else
            k = k+1;
        end
  
   
    end