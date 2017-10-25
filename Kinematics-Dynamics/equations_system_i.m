function [d_x, y ] = equations_system_i(t, x, u, c1_i, c2_i, c4_i,R, Ld, Lq, varargin)

% Output equations
y = [x(3);                                                                 % current d-axis
     x(4)];                                                                % current q-axis

% State equations
d_x = [
 u(1) - x(3) - (c4_i*x(1))/c3_i;
 u(2) - x(4) - (c4_i*x(2))/c3_i;
 (c1_i*u(1))/(Ld*c3_i) - x(3)*(R/Ld + c1_i/(Ld*c3_i)) - (x(1)*(c1_i*c4_i - c2_i*c3_i))/(Ld*c3_i^2);
 (c1_i*u(2))/(Lq*c3_i) - x(4)*(R/Lq + c1_i/(Lq*c3_i)) - (x(2)*(c1_i*c4_i - c2_i*c3_i))/(Lq*c3_i^2)];
end