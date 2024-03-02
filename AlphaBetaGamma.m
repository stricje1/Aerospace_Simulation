%**************************************************************************
%The following is the MATLAB@ program used in the tracking of the ballistic
%missile base trajectory .
% abg.m
% Dr. Jeffrey Strickland
%
% This program uses an Alpha_Beta_Gamma tracker to filter the sensor
% measurement noise from the ballistic missile base trajectory
% developed using SIMULINK. Random noise is added to the measurement
% process. Actual missile track is generated in FlatEarthMissile
% SIMULINK model.
%
% delta = 0.1 sec
% nloops = 100
% alpha = 0.6
%**************************************************************************

% Load base trajectory simulation workspace
     %clear all
     %load dat1;          % base trajectory developed in SIMULINK model
     missilevec = missilevec;

% Define the number of simulation loops
     nloops = 100;

% Define the sampling interval
     delta = 1;

% Define the number of samples
     [num_rows,num_cols] = size(missilevec);
     nsamples = 6140;

% Initialize sensor data
     Sensor_posit =[ 100 * 1000;  % sensor is 100 km'in x
                     100 * 1000;  % sensor is 100 km in y
                       0 * 1000]; % sensor is 100 km in z

     sigma_r = 10;       % 10 meters std dev in range i
     sigma_b = 1*pi/180; % 1 degree std dev in azimuth
     sigma_e = 1*pi/180; % 1 degree std dev in elevation

% Define F matrix (TRANSITION MATRIX) for discrete time
% target motion, x (ktl) = F(k) *x(k) + G

     f_sub = [1, delta, (delta^2)/2;
                0, 1, delta;
                0, 0, 1];

     F = [ f_sub,    zeros(3), zeros(3);
           zeros(3), f_sub,    zeros(3);
           zeros(3), zeros(3), f_sub  ];

% Define G matrix
     G = - g * [0;
                0;
                0;
                0;
                0;
                0;
               (delta^2)/2;
                delta;
                0] ;

% Define the H matrix (MEASUREMENT MATRIX), assuming that the
% x, y, an z missile positions are observed directly; z(k) = H(k)*x(k)

     H = [1,0,0,0,0,0,0,0,0;
          0,0,1,0,0,0,0,0,0;
          0,0,0,0,0,0,1,0,0];

% Define alpha, beta, gamma tracker parameters
     alpha =  0.6;
     beta =   2*(2-alpha) - 4*sqrt(1-alpha);
     gamma = (beta^2)/(2*alpha);
     nu =     1;

     K_abg =  [alpha,                 0,                       0;
               beta/(nu*delta),       0,                       0;
               gamma/((nu*delta)^2),  0,                       0;
               0 ,                    alpha,                   0;
               0 ,                    beta/(nu*delta),         0;
               0 ,                    gamma/((nu*delta)^2),    0;
               0 ,                    0,                      alpha;
               0 ,                    0,                  beta/(nu*delta);
               0 ,                    0,            gamma/((nu*delta)^2)];

% Define initialization parameters

     d_sub = [ 1,             0,   0,  0,           0,   0,  0;
               3/(2*delta),   0,   0, -2/delta,     0,   0,  1/(2*delta);
               1/(delta^2),   0,   0, -2/(delta^2), 0,   0,  1/(delta^2)];

     D = [d_sub, zeros(3,2);
          zeros(3,1), d_sub,zeros(3,1);
          zeros(3,2), d_sub];

%******************* End of Initialization outside loops ******************
%**************************************************************************
% Loop over the target motion/measurement simulation
%**************************************************************************
 for kk = 1: nloops
tic;
kk;
        % define empty output matricies
        % measurement positions (cartesian) w/error
          zout_true_n = [] ;

        % distance error between measurement and true position
          error_true = [] ;

        % Kalman estimated trajectory
          ABG_track = [];

        % error between Kalman track and actual track
          track_error = [];

%**************************************************************************
% Loop through the simulation, generating target motion between
% sample times and taking measurements at each sample time,
% using 1 sensor
%**************************************************************************
     for ii = 1:nsamples
        % Process the measurement from Sensor
        % True missile position
          ztrue = [missilevec(2, ii) ;
                   missilevec(5, ii) ;
                   missilevec(8, ii)];

%**************************************************************************
% convert current position to polar coordinates and add
% sensor noise to the position, generating a noisy measurement
% from the sensor
%**************************************************************************
        % position relative to the sensor
          zrel = ztrue - Sensor_posit;
          r = sqrt (zrel(1)^2 + zrel(2)^2 + zrel(3)^2);

        % range from sensor
          b = atan2(zrel(2), zrel(1) );

        % bearing from sensor
          r_prime = sqrt(zrel(1)^2 + zrel(2)^2);

        % range in x-y plane
          e = atan2(zrel(3), r_prime);  % elevation from sensor

        % add noise to the measurement
          r_n = r + sigma_r * randn;
          b_n = b + sigma_b * randn;
          e_n = e + sigma_e * randn;

        % measurement in polar + noise
          z_polar_n = [r_n ;
                       b_n ;
                       e_n];
        % measbrement in cartesian coordinates + noise
          z_cart_true_n = [r_prime*cos(b_n);
          r_prime*sin(b_n);
          r_n*sin(e_n) ] + Sensor_posit;

          z_cart_rel_n = [r_prime*cos(b_n);
                          r_prime*sin(b_n);
                          r_n*sin(e_n)];

        % compute measurement error in cartesian coordinates
          zdiff = ztrue - z_cart_true_n;
          disterror = sqrt(zdiff'*zdiff);

        % Update the measurement array
        % true cartesian measurement + error
          zout_true_n = [zout_true_n, z_cart_true_n];

        % measurement error (between true measurement & true measurement w/
        % noise)
          error_true = [error_true, disterror];

          if ii > 2 % For intialization from the first 3 measurements

%**************************************************************************
% Prediction
%**************************************************************************
             % Initialization using the first 3 measurements
               if ii == 3
                    x_corr = D * [zout_true_n(:,3);
                                  zout_true_n(:,2) ;
                                  zout_true_n(:,1)] ;
               end; %if ii==3

               % ABG Filter prediction equations
                 x_predict = F * x_corr + G;

%**************************************************************************
% Correction
%**************************************************************************
             % Convert to relative position to compute RBE coordinates
               x_1 = x_predict(1) - Sensor_posit(1);
               x_4 = x_predict(4) - Sensor_posit(2);
               x_7 = x_predict(7) - Sensor_posit(3);

             % Convert prediction to Range, Bearing, Elevation coordinates
               r_hat = sqrt (x_1^2 + x_4^2 + x_7^2);
               b_hat = atan2(x_4, x_1);
               e_hat = atan2(x_7, sqrt(x_1^2 + x_4^2));

             % Determine expected measurement
               z_cart_exp_rel = [r_hat*cos(b_hat) *cos(e_hat);
                                 r_hat*cos(e_hat) *sin(b_hat);
                                 r_hat*sin(e_hat)] ;
               z_cart_exp_true = z_cart_exp_rel + Sensor_posit;

             % Observed minus expected measurements
               z_tilde_c = z_cart_rel_n - z_cart_exp_rel;

             % Correction equations
               x_corr = x_predict + K_abg * z_tilde_c;

             % Alpha_Beta_Gamma track positions and difference between ABG
             % and actual track position and actual target position
               zout_ABG_track = H * x_corr;
               track_diff = ztrue - zout_ABG_track;
               tracferror = [track_error,
                             sqrt(track_diff'*track_diff)];

             % Update ABG track trajectory array
               ABG_track = [ABG_track, zout_ABG_track];
          end; % if ii>2
     end; % for ii = 1:nsamples
     if kk == 1, % create first output
          zoutmean_true = zout_true_n;
          mean_ABG_track = ABG_track;
          merror_track = track_error;
          merror = error_true;
     else % create output after 1st run
          zoutmean_true = zoutmean_true + zout_true_n;
          mean_ABG_track = mean_ABG_track + ABG_track;
          merror_track = merror_track + track_error;
          merror = merror + error_true;
     end; % if kk ==I, else
toc;
end; % for kk = 1:nloops

%**************************************************************************
% Compute Means
%**************************************************************************
  zoutmean_true = zoutmean_true/nloops;
  mean_ABG_track = mean_ABG_track/nloops;
  merror = merror/nloops;               % mean error between
                                        % measurement and true position
  merror_track = merror_track/nloops;   % mean error between
                                        % EKF estimated position
                                        % and true position

%**************************************************************************
% Plot results
%**************************************************************************
figure (1)
     measurement = zoutmean_true/1000;                 % convert to km
     ABG = mean_ABG_track/1000;                        % convert to km
     missile_track = missilevec (:,1:nsamples) /1000;  % convert to km
    plot3(missile_track(2,:),missile_track(5,:),missile_track(8,:),'g-',...
     Sensor_posit(1)/1000, Sensor_posit(2)/1000,...
     Sensor_posit(3)/1000, 'rx');
     
     axis ( [0, 150,0,150, 0, 150] ) ;
     title('Ba1listic Missile Base Trajectory - 120 seconds');
     xlabel ('X (km)'), ylabel ('Y (km)'), zlabel('Z (km)'), grid;
     print -deps c4f1c

figure (2)
     plot3(missile_track(2,:),missile_track(5,:),missile_track(8,:),'b-',...
     measurement(1,:), measurement(2,:), measurement(3,:), 'r-',...
     Sensor_posit(1)/1000, Sensor_posit(2)/100, Sensor_posit(3)/1000,'rx');
     %axis( [0, 150,0,150, Or 150] ) ;
     title('Ba1listic Missile Base Trajectory with Measurement Noise - 120 seconds');
     xlabel ('X (km)'), ylabel ('Y (km)'), zlabel('Z (km)'), grid;
     print -deps c4f2c

figure (3)
     plot3(missile_track(2,:),missile_track(5,:),missile_track(8,:),'b-',...
     ABG(1,1:nsamples-2), ABG(2,1:nsamples-2), ABG(3,1:nsamples-2),'r-',...
     Sensor_posit(1)/1000,Sensor_posit(2)/1000,Sensor_posit(3)/1000, 'rx');
     axis ( [0, 150,0,150, 0, 150] ) ;
     xlabel('X (km)'), ylabel('Y (km )') , zlabel('Z (km)'),grid;
     title('Ba1listic Missile Base Trajectory and ABG Trajectory - 120 seconds');
     print -deps c4f3c

figure (4)
     start_pt = 1;
     stop_pt = 401;
     zoom_missile = ((start_pt + 1) : (stop_pt));
     zoom_Kalman = (start_pt : stop_pt-2);
     plot3(missile_track(2,zoom_missile), missile_track(5,zoom_missile),...
     missile_track(8, zoom_missile),'b-',...
     ABG(1,zoom_Kalman), ABG(2,zoom_Kalman), ABG(3,zoom_Kalman), 'r-') ;
     axis ( [30,60,30,60,0,60] ) ;
     xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)'),grid;
     title(['ZOOM - ABG Trajectory Initial',num2str( (stop_pt - start_pt)/10),'Seconds']);
     print -deps c4f4c

figure (5)
     start_pt = 1;
     stop_pt = 601;
     zoom_missile = ((start_pt + 1 ):(stop_pt));
     zoom_Kalman = (start_pt : stop_pt-1);
     plot3(missile_track(2,zoom_missile), missile_track(5,zoom_missile),...
     missile_track(8, zoom_missile))%,...
     %ABG(1,zoom_Kalman), ABG(2,zoom_Kalman), ABG(3,zoom_Kalman),'r-');
     %axis ( [30,60,30,60, 0, 60] ) ;
     xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)'),grid;
     title(['ZOOM - ABG Trajectory Initial',num2str( (stop_pt - start_pt)/lO),'Seconds']);
     print -deps c4f5c

figure (6)
     start_pt = 1;
     stop_pt = 801;
     zoom_missile = ((start_pt + 1 ) : (stop_pt ));
     zoom_Kalman = (start_pt : stop_pt-l);
     plot3(missile_track(2,zoom_missile), missile_track(5,zoom_missile),...
     missile_track(8, zoom_missile) ,'g_',...
     ABG(1,zoom_Kalman), ABG (2,zoom_Kalman), ABG (3,zoom_Kalman), 'r-');

     axis([30,60,30,60,0,60]);
     xlabel('X (km)'), ylabel ('Y (km)'), zlabel('Z (km)'), grid;
     title(['ZOOM - ABG Trajectory Initial',num2str((stop_pt - start_pt)/lO),'Seconds']);
     print -deps c4 f 6c

figure (7)
     time = missilevec(1,:);
     diff_ABG_base = [ABG(1,:) - missile_track(2,3:nsamples);
                      ABG(2,:) - missile_track(5,3:nsamples);
                      ABG(3,:) - missile_track(8,3:nsamples)];

     plot(time(1:nsamples), merror, 'g-',...
          time(3:nsamples), sqrt(diff_ABG_base(1,:).^2 +...
          diff_ABG_base(2,:).^2 + diff_ABG_base(3,:).^2),'b','LineWidth',2);

     xlabel('Time (seconds)') , ylabel('Error (meters)') , grid;
     title('ABG Distance Error vs. Time');
     legend('Mean Distance Error','ABG Distance Error');
     print -deps c4f7c

figure (8)
     p1ot(time(1:nsamp1es),merror,'g-',time(3:nsamp1es),merror_track,'r-');
     xlabel('Time (seconds)') , ylabel ('Mean Error (meters)') , grid; 
     title('Mean Distance Error in Measurements vs Time');
     %(1,num2str(nloops),'runs', nurn2str(nsamples),'data points)']),grid;
     print -deps c4f8c
