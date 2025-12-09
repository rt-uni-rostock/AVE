function x0 = SolveQuadraticEquation(a, b, c)
    %ave.SolveQuadraticEquation Solve the quadratic equation 0 = a*x^2 + b*x + c for x.
    % 
    % PARAMETER
    % a ... Scalar coefficient a for quadratic equation 0 = a*x^2 + b*x + c.
    % b ... Scalar coefficient b for quadratic equation 0 = a*x^2 + b*x + c.
    % c ... Scalar coefficient c for quadratic equation 0 = a*x^2 + b*x + c.
    % 
    % RETURN
    % x0 ... 2-by-1 vector indicating the solution. If no solution exists [NaN; NaN] is returned. Otherwise
    %        [x_min, x_max] is returned, where x_max >= x_min. If only one solution exists, x_max is equal to x_min.
    arguments
        a (1,1) double
        b (1,1) double
        c (1,1) double
    end
    x0 = nan(2,1);
    if(abs(a) > 0.0)
        sr = b*b - 4.0*a*c;
        if(sr >= 0.0)
            sr = sqrt(sr);
            x1 = (-b + sr) / (a + a);
            x2 = (-b - sr) / (a + a);
            x0(1) = min(x1, x2);
            x0(2) = max(x1, x2);
        end
    else
        if(abs(b) > 0.0)
            x0(1) = -c / b;
            x0(2) = x0(1);
        end
    end
end
