function [isend,k] = point(way_points,x,y,k,total,Lnpp,R0)
    isend = 0;
    point_k = way_points(k,:); 
    point_n = [x,y];
    
    r = norm(point_k-point_n);
    if(r <= R0 || r<Lnpp) 
        disp(["到达航路点,",k]);
        if(k==total)
            disp("任务结束...");
            isend = 1;
        else
            k = k+1;
        end
  
   
    end