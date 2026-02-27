function q2 = quat_inv(q1)
% Perform inverse of quaternion. q1 is 4x1 double with q(1:3) as vector and
% q(4) as scalar.

q2 = [- q1(1:3); q1(4)] / norm(q1)^2; % A.18 in Crassidis
end

