function [out] = lpfilter(delta_T,T,last,now)

out = now*delta_T/T + ( 1-T/T )*last;

end

