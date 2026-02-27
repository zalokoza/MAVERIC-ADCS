Ixx = .03293;
Iyy = .13199;
Izz = .13943;
Ixy = .00005;
Iyz = -.00016;
Izx = -.00014;

% Is stowed? Not stowed?

Ib = [Ixx Ixy Izx;
    Ixy Iyy Iyz;
    Izx Iyz Izz];
Ibinv= Ib^-1;

q0 = [.5; .5; .5; .5];
q0 = [.3; .2; .6; -1];

q0 = q0/norm(q0);
dx0 = [.05; .05; .05];

% Bdot Sim
K = 10^5;
Ts = 1;
kick = [1; 2; 2];