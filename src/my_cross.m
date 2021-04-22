function z=my_cross(u,v)
    z=[ u(2)*v(3)-u(3)*v(2); 
        u(3)*v(1)-u(1)*v(3);
        u(1)*v(2)-u(2)*v(1)];
end