function [xdot, y] = ASVIdentModel(~,state,input,weightU,weightV,weightR,f11,f12,f13,f14,f15,f16,f17,f18,f19,f1a,f1b,f1c,f21,f22,f23,f24,f25,f26,f27,f28,f29,f2a,f2b,f2c,f31,f32,f33,f34,f35,f36,f37,f38,f39,f3a,f3b,f3c,b11,b12,b13,b21,b22,b23,b31,b32,b33,varargin)
    % ODE function to be used for non-linear grey-box estimation in ave.ASVIdent.

    % Current state (velocity) and input (force)
    u  = state(1);
    v  = state(2);
    r  = state(3);
    X  = input(1);
    Y  = input(2);
    N  = input(3);

    % Parameters
    F = [ ...
        f11,f12,f13,f14,f15,f16,f17,f18,f19,f1a,f1b,f1c; ...
        f21,f22,f23,f24,f25,f26,f27,f28,f29,f2a,f2b,f2c; ...
        f31,f32,f33,f34,f35,f36,f37,f38,f39,f3a,f3b,f3c ...
    ];
    B = [b11 b12 b13; b21 b22 b23; b31 b32 b33];

    % Free response
    freeResponse = F * [u; v; r; v*r; u*r; u*v; u*u; v*v; r*r; u*u*u; v*v*v; r*r*r];

    % Forced response
    forcedResponse = B * [X;Y;N];

    % Model equation
    xdot = freeResponse + forcedResponse;

    % Output
    y = state;
    y(1) = weightU * y(1);
    y(2) = weightV * y(2);
    y(3) = weightR * y(3);
end
