% The following is the MATLAB@ program used to initialize the ballistic 
% missile simulation.
%**************************************************************************
% MissileSimInit.m
% Dr. Jeffrey S. Strickland
%
% This script file initializes the flat earth point
% missile simulation
%**************************************************************************
% define globals
     global g T tar_T tar_tToff tar_troll tar_cfric tToff troll cfric xinit yinit tmax sinterval;
     

     g = 9.8;                 % gravity, meters/sec^2
     T = 7*g;                 % missile acceleration
     mass_0 = 85;             % initial mass
     mass_bo = 57;            % mass at burnout
     Ae = 0.011;              % exit area of rocket nozzle
     I_sp = 2224;             % specific impulse
     I_0 = 61;                % moment of inertia about y- and z-axes at launch
     I_bo = 47;               % moment of inertia about y- and z-axes at burnout
     x_cm0 = 1.55;            % distance from missile nose to center of mass at launch
     x_cmbo = 1.35;           % distance from missile nose to center of mass at burnout
     tToff = 120; % 100;      % time of thrust shut off (seconds)
     troll = 25;  % 30;       % time of missile rollover(seconds)
     cfric = 0.05;            % coefficient of friction
     sinterval = 0.1;         % sampling interval (seconds)
     tmax = 520;              % max simulation time (seconds)
     launch_delay = 20;       % delay for target acquisition
     D = 50;                  % diameter in centimeters

     wel = (43*(pi/180))/(tToff-troll);    % rotation in elev (rads/sec)
     waz = (15*(pi/180))/(tToff-troll);    % rotation in azimuth (rads/sec)
    
     minstep = .001;          % minimun step size
     numsamp =tmax/minstep;   % number of samples
     xinit = [30 * 1000;      % Initial Missile x position (m)
               0;             % Initial Missile x velocity (m/s)
               0;             % Initial Missile x acceleration (m/s^2);
              40 * 1000;      % Initial Missile y position (m)
               0;             % Initial Missile y velocity (m/s)
               0;             % Initial Missile y acceleration (m/s^2);
               0;             % Initial Missile z position (m)
               0.001 ;        % Initial Missile z velocity (m/s)
               0] ;           % Initial Missile z acceleration (m/s^2)


     tar_g = 9.8;                 % gravity, meters/sec^2
     tar_T = 7*g;                 % missile acceleration
     tar_tToff = 120; % 100;      % time of thrust shut off (seconds)
     tar_troll = 25;  % 30;       % time of missile rollover(seconds)
     tar_cfric = 0.05;            % coefficient of friction

     tar_wel = (45*(pi/180))/(tToff-troll);    % rotation in elev (rads/sec)
     tar_waz = (-34.5*(pi/180))/(tToff-troll); % rotation in azimuth (rads/sec)

     yinit = [1000 * 1000;    % Initial Missile x position (m)
               0;             % Initial Missile x velocity (m/s)
               0;             % Initial Missile x acceleration (m/s^2);
              800 * 1000;     % Initial Missile y position (m)
               0;             % Initial Missile y velocity (m/s)
               0;             % Initial Missile y acceleration (m/s^2);
               0;             % Initial Missile z position (m)
               0.001 ;        % Initial Missile z velocity (m/s)
               0] ;           % Initial Missile z acceleration (m/s^2)
