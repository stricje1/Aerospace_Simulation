% The following is the MATLAB" program used to generate missile dynamics
% using flat earth equations of motion.

function xdot = FlatEarthMissileEqns(u)

%**************************************************************************
% This Function computes the Flat Earth, Point Mass Equations
% for Missile Dynamics.
%
% Dr. Jeffrey S. Strickland
%
% The input vector is defined as:
%    u(1) = T,      thrust along the missile velocity vector
%    u(2) = wel,    Velocity Vector Rotation Rate in elevation
%    u(3) = waz,    Velocity Vector Rotation Rate in azimuth
%
% The State Vector is defined as:
%
%    Position Variables
%         u(4) = Px, Position North of (0,0,0)
%         u(7) = Py, Position East of (0,0,0)
%         u(10)= Pz, Height
%
%    Position Velocities
%         u(5) = U,  D(Px)/dt
%         u(8) = V,  D(Py)/dt
%         u(11)= W,  D(Pz)/dt
%
%    Position Accelerations
%         u(6) = Ax, D(Px)/dt
%         u(9) = Ay, D(Py)/dt
%         u(12)= Az, D(Pz)/dt
%
% Related Quantities
%    g,        Gravitational Force = 9.8 meters/secA2
%    cfric,    coefficient of friction
%    rho,      air density with altitude
%    mass,     missile mass
%    tToff,    Time of Thrust Shutoff
%    troll,    Time of Missile Rollover < tToff
%
% . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
% Declare Global Variables
     global g mass tToff troll cfric tmax;

% Define Control Variables from Inputs
     T   = u(1);    % thrust along missile velocity
     wel = u(2);    % turn rate in elevation
     waz = u(3);    % turn rate in azimuth

% Define State Variables from Inputs
     x = u(4:12);

% Location Variables
     Px = x(1);     % Position in Direction of North Pole
     Py = x(4);     % Position At Equator in y
     Pz = x(7);     % Position At Equator in z

% Body_Axes Velocities
     U = x(2);     % velocity in Px direction
     V = x(5);     % velocity in Py direction
     W = x(8);     % velocity in Pz direction ("Up")

% Body Axes Acceleration
     %Accx = x(3);
     %Accy = x(6);
     %Accz = x(9);
     
% Speed, Atmospheric Density and Drag
     Vxy2 = U^2 + V^2;
     Vxy  = sqrt(Vxy2);
     Vxz2 = U^2 + W^2;
     Vt2  = Vxz2 + V^2;
     Vt   = sqrt(Vt2);
     az   = atan2(V, U);
     el   = atan2(W, Vxy);

% Atmospheric Density in kg/meterA3
     if Pz < 0           % Travel inside the Earth is Viscous
          rho = 10^2;
     elseif Pz < 9144    % Altitudes below 9144 meters
          rho = 1.22557*exp(-Pz/9144);
     else                % Altitudes above 9144 meters
          rho = 1.75228763*exp(-Pz/6705.6);
     end

     beta = cfric*rho;
     Tacc = T/Vt;

% Compute the Derivatives
     dPx = U;
     dPy = V;
     dPz = W;

% Azimuth and Elevation Rollover
     dU = -waz*V + wel*W*cos(az) - beta*U + Tacc*U;
     dV =  waz*U + wel*W*sin(az) - beta*V + Tacc*V;
     dW = -wel*Vxy - g - beta*W + Tacc*W;

xdot = [dPx;
        dU ;
        0;
        dPy;
        dV ;
        0;
        dPz;
        dW ;
        0];