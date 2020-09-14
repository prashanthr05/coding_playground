function rpy = rot2rpy(R)
arguments
    R (3, 3) double;
end
if (R(3, 1) < 1.0)
    if (R(3, 1) > -1.0)
        r = atan2(R(3, 2), R(3, 3));
        p = asin(-R(3, 1));
        y = atan2(R(2,1), R(1,1));
    else
        % not a unique solution
        r = 0.0;
        p = pi/2.0;
        y = atan2(-R(2, 3), R(2, 2));
    end
else
        % not a unique solution
        r = 0.0;
        p = -pi/2.0;
        y = atan2(-R(2, 3), R(2, 2));
end
rpy = [r; p; y];
end

