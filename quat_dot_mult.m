function q = quat_(q1,q2)
% Perform q1 * q2 as defined in Crassidis (A.8b). q1 and q2 are 4x1
% doubles. qi(4) is the scalar, q(1:3) is the vector.

skew = [0 -q1(3) q1(2);
    q1(3) 0 -q1(1);
    -q1(2) q1(1) 0]; % Skew symmetric of the vector portion of q1

q = [q1(4)*eye(3)+skew, q1(1:3);
    -q1(1:3)' q1(4)]*(q2); % Eqn A.8b in Crassidis

end

