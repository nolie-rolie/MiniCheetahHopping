function fc = GetContactForcesFWP(fnet, n, zCON)

load("FWP_each_foot");

mu = 0.7;
Af = [ 1  0 -mu/sqrt(2);
      -1  0 -mu/sqrt(2);
       0  1 -mu/sqrt(2);
       0 -1 -mu/sqrt(2);
       0  0  1
       0  0 -1];
bf = [0;
      0;
      0;
      0;
      150;
      0];

FWPFL = Polyhedron('V',V_FWPFL);
FWPFR = Polyhedron('V',V_FWPFR);
FWPHL = Polyhedron('V',V_FWPHL);
FWPHR = Polyhedron('V',V_FWPHR);

nForces = length(fnet(1,:));
% nForces = 2;
fc = sdpvar(3,4,nForces,'full');
nc = sdpvar(3,4,nForces,'full');
S = sdpvar(1,nForces,'full');

constr = [];

for i = 1:nForces
    constr = [constr, ...
              FWPFL.A*[fc(:,1,i);nc(:,1,i)] <= FWPFL.b - S(i), ...
              FWPFR.A*[fc(:,2,i);nc(:,2,i)] <= FWPFR.b - S(i), ...
              FWPHL.A*[fc(:,3,i);nc(:,3,i)] <= FWPHL.b - S(i), ...
              FWPHR.A*[fc(:,4,i);nc(:,4,i)] <= FWPHR.b - S(i), ...
              fnet(:,i) == fc(:,1,i) + fc(:,2,i) + fc(:,3,i) + fc(:,4,i), ...
              n(:,i) == nc(:,1,i) + nc(:,2,i) + nc(:,3,i) + nc(:,4,i)
              ];
    if zCON(2,i) == 1
        constr = [constr, ...
                  [fc(:,1,i);nc(:,1,i)] == zeros(6,1), ...
                  [fc(:,2,i);nc(:,2,i)] == zeros(6,1)];
    elseif zCON(3,i) == 1
        constr = [constr, ...
                  [fc(:,3,i);nc(:,3,i)] == zeros(6,1), ...
                  [fc(:,4,i);nc(:,4,i)] == zeros(6,1)];
    end
    
    % friction cone (single leg FWP half-space form deprecated(?)
    for leg = 1:4
        constr = [constr, ...
                  Af * fc(1:3,leg,i) <= bf];
    end
end


objective = 0;
for i = 1:nForces
    objective = objective - S(i);
end


options = sdpsettings('verbose',2,'solver','GUROBI','debug',1);
sol = optimize(constr,objective,options);

fc = value(fc);
