function dx = EOM(~,x,u,n)
    % x(1:3) : position
    % x(4:6) : velocity

    dx = zeros(size(x));
    dx(4) = u(1)+3*n^2*x(1)+2*n*x(5);
    dx(5) = u(2)-2*n*x(4);
    dx(6) = u(3)-n^2*x(3);
    % dx(4) = u(1);
    % dx(5) = u(2);
    % dx(6) = u(3);
    dx(1)=x(4);
    dx(2)=x(5);
    dx(3)=x(6);
end