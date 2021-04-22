function z = J_centrifugal(u,v)
% u is w
% v is r
    z = skew_sym(u)'*skew_sym(v)+skew_sym(my_cross(v,u));
end
