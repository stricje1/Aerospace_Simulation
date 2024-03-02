%The following is the MATLAB" program used in the tracking of the ballistic
%missile base trajectory .
%**************************************************************************
% imm.m
% Dr. Jeffrey Strickland
%
% This program generates a Kalman filter missile track using IMM with
% 2 models: an accelerating model and a ballistic model.
% The filter is initialized is from the missile launch position.
% Random noise is added in the sensor measurement process.
% Actual missile track is generated in FlatEarthMissle SIMULINK model.
%
%**************************************************************************
% Load simulation workspace
     %clear all
     load tbm_dat;
     missilevec = missilevec';
% Define the number of simulation loops
     nloops = 100;
% Define the sampling interval
     delta =.1;
% Define the number of samples
     [num_rows,num_cols] = size(missilevec);
     nsamples = 92;
% Define q^2
     q_sqr = 10;

% Initialize sensor data
       Sensor_posit =[100 * 1000;  % sensor is 100 km in x
                      100 * 1000;  % sensor is 100 km in y
                        0 * 1000]; % sensor is 0 km in z

       sigma_r = 10;               % 10 meters std dev in range
       sigma_b = 1*pi/180;         % 1 degree std dev in azimuth
       sigma_e = 1*pi/180;         % 1 degree std dev in elevation

       R = diag([sigma_r^2,        % covariance matrix for uncorrelated
                 sigma_b^2,        % range and bearing measurements
                 sigma_e^2]);

     % Define the H matrix (MEASUREMENT MATRIX) for the accelerating
     % model

       H = [1,0,0,0,0,0,0,0,0;
            0,0,1,0,0,0,0,0,0;
            0,0,0,0,0,0,1,0,0];

%**************************************************************************
% ACCELERATING MODEL
%**************************************************************************
     % Define G matrix
       G_accel = -g * [0;
                       0;
                       0;
                       0;
                       0;
                       0;
                      (delta^2)/2;
                       delta;
                       0];

     % Initialize Q, the covariance of the plant noise
       Q_sub_a = [(delta^5)/20, (delta^4)/8, (delta^3)/6;
                  (delta^4)/8,  (delta^3)/3, (delta^2)/2;
                  (delta^3)/6 , (delta^2)/ 2, delta ];

       Q_accel = q_sqr * [Q_sub_a,  zeros(3), zeros(3) ;
                          zeros(3), Q_sub_a,  zeros(3) ;
                          zeros(3), zeros(3), Q_sub_a];

     % Define F matrix (TRANSITION MATRIX) for discrete time
     % accelerating model.
       f_sub_a = [1,   delta, (delta^2)/2;
                  0,   1,      delta;
                  0,   0,      1 ] ;

       F_accel = [f_sub_a,  zeros(3), zeros(3) ;
                  zeros(3), f_sub_a,  zeros(3) ;
                  zeros(3), zeros(3), f_sub_a] ;

%**************************************************************************
% BALLISTIC MODEL
%**************************************************************************
% Define G matrix
       G_ball = -g * [0;
                      0;
                      0;
                      0;
                      0;
                      0;
                      (delta^2)/ 2;
                      delta;
                      0];

% Detemine Q for the Ballistic model
       Q_sub_b = [(delta^3)/ 3, (delta^2)/2 , 0;
                  (delta^2)/ 2,  delta,       0;
                   0 ,           0 ,          0];

       Q_ball = q_sqr * [Q_sub_b,  zeros(3), zeros(3) ;
                         zeros(3), Q_sub_b,  zeros(3) ;
                         zeros(3), zeros(3), Q_sub_b] ;

% Define F matrix (TRANSITION MATRIX) for discrete time
% ballistic model.
       f_sub_b = [1, delta, 0;
                  0, 1, 0;
                  0, 0, 0];

       F_ball = [f_sub_b,  zeros(3), zeros(3);
                 zeros(3), f_sub_b,   zeros(3);
                 zeros(3), zeros(3),  f_sub_b ];

%***************** End of Initialization outside loops ********************
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
       K_track = [] ;
       K_accel = [] ;
     % error between Kalman track and actual track
       track_error = [];
%**************************************************************************
% This block is used for the initialization process. Initialize
% from launch position.
%**************************************************************************
       x_corr_accel = missilevec(2:10,1);
       P_corr_accel = eye(9) * 10^4;

       x_corr_ball = missilevec(2:10,1);
       P_corr_ball = eye(9) * 10^4;

     % Initial likelihoods for states.
       mu_init=[1;0];
       mu = mu_init;
       mu_1 = mu(1);
       mu_2 = mu(2);

%**************************************************************************
% Loop through the simulation, generating target motion between
% sample times and taking measurements at each sample time,
% using 1 sensor
%**************************************************************************
  for ii = 2:nsamples
     % Process the measurement from Sensor
     % True missile position
       ztrue = [missilevec(2,ii) ;
                missilevec(5,ii) ;
                missilevec(8,ii)];
%**************************************************************************
% convert current position to polar.coordinates and add
% sensor noise to the position, generating a noisy measurement
% from the sensor.
%**************************************************************************

     % position relative to the sensor
       zrel = ztrue - Sensor_posit;
       r = sqrt(zrel(1)^2 + zrel(2)^2 + zrel(3)^2); % range from sensor
       b = atan2(zrel(2), zrel(1)); % bearing from sensor
       r_prime = sqrt(zrel(1)^2 + zrel(2)^2); % range in x-y plane
       e = atan2(zrel(3), r_prime);  % elevation from sensor

     % add noise to the measurement
       r_n = r + sigma_r * randn;
       b_n = b + sigma_b * randn;
       e_n = e + sigma_e * randn;

     % measurement in polar + noise
       z_polar_n = [r_n;
                    b_n;
                    e_n];

     % measurement in cartesian coordinates + noise
       z_cart_rel_n = [r_prime*cos(b_n);
                       r_prime*sin(b_n);
                       r_n*sin(e_n)   ];

       z_cart_true_n = z_cart_rel_n + Sensor_posit;

% computk measurement error in cartesian coordinates
       zdiff = ztrue - z_cart_true_n;
       disterror = sqrt(zdiff'*zdiff);
     % Update the measurement array
     % true cartesian measurement + error
       zout_true_n = [zout_true_n, z_cart_true_n];
     % measurement error (between true measurements) error-true = 
     % [error-true, disterror] ;
%*******************************************************************************
% Prediction
%*******************************************************************************
% Probabilities of changing state, Markov chain transition
       p1 = 1;
       p2 = 0.5;
       alt = 50e3;
       h = z_cart_true_n(3);

       prob_accel = -p2*(1/(1+ exp(-.0005*(h-alt))) - (1+p1));
       prob_ball = 1 - prob_accel;
       rho = [prob_accel, prob_ball;
              0 , 1 ];

     % Accelerating Model
       cbar = rho' * mu;
       if cbar < 10^(-8) % prevents cbar_1 from blowing up
          cbar_1 = 10^(-8);
       else
          cbar_1 = cbar(1) ;
       end;
       cbar_2 = cbar(2);

       rho_11 = rho(1,1);
       rho_21 = rho(2,1) ;
       rho_12 = rho(1,2);
       rho_22 = rho(2,2);

       x_hat_01 = x_corr_accel * ((rho_11*mu_1)/cbar_1) +...
       x_corr_ball * ( (rho_21*mu_2)/cbar_1 ) ;

       xtilde_11 = x_corr_accel - x_hat_01;
       xtilde_21 = x_corr_ball - x_hat_01;

       mu_11 = rho_11 * mu_1 / cbar_1;
       mu_21 = rho_21 * mu_2 / cbar_1 ;

       P_hat_01 = mu_11 * (P_corr_accel +...
                  xtilde_11*xtilde_11') +...
                  mu_21 * (P_corr_ball +...
                  xtilde_21*xtilde_21');

     % Kalman Filter Prediction Equations for Accelerating model
       x_predict_accel = F_accel * x_hat_01 + G_accel;
       P_predict_accel = F_accel * P_hat_01 * F_accel' + Q_accel;

     % Ballistic Model
       x_hat_02 = x_corr_accel * ((rho_12*mu_1)/cbar_2)+ ...
                  x_corr_ball  * ((rho_22*mu_2)/cbar_2) ;

       xtilde_12 = x_corr_accel - x_hat_02;
       xtilde_22 = x_corr_ball  - x_hat_02;

       mu_12 = rho_12 * mu_1 / cbar_2;
       mu_22 = rho_22 * mu_2 / cbar_2 ;

       P_hat_02 = mu_12*(P_corr_accel + xtilde_12*xtilde_12')+...
                  mu_22*(P_corr_ball  + xtilde_22*xtilde_22');

     % Kalman Filter Prediction Equations for Ballistic model
      x_predict_ball = F_ball * x_hat_02 + G_ball;
      P_predict_ball = F_ball * P_hat_02 * F_ball' + Q_ball;

%**************************************************************************
% Correction
%**************************************************************************
%*
% Accelerating Model
     % Convert to relative position to compute polar coordinates
       x_1 = x_predict_accel(1) - Sensor_posit(1);
       x_4 = x_predict_accel(4) - Sensor_posit(2);
       x_7 = x_predict_accel(7) - Sensor_posit(3);

     % Convert prediction to polar coordinates
       r_hat_a = sqrt(x_1^2 + x_4^2 + x_7^2);
       b_hat_a = atan2 (x_4, x_1) ;
       e_hat_a = atan2(x_7, sqrt(x_1^2 + x_4^2)) ;

     % Determine expected measurement
       z_polar_hat_a = [r_hat_a;
                        b_hat_a;
                        e_hat_a];

     % Observed minus expected measurements
       z_tilde_a = z_polar_n - z_polar_hat_a;

% The gradient of H evaluated at the most recent estimate
Hk_r2c1 = -x_4/(x_1^2 + x_4^2);
Hk_r2c4 =  x_1/(x_1^2 + x_4^2);
Hk_r3c1 = (-x_1*x_7)/( (sqrt(x_1^2 + x_4^2))*(x_1^2 + x_4^2 + x_7^2) ) ;
Hk_r3c4 = (-x_4*x_7)/( (sqrt(x_1^2 + x_4^2))*(x_1^2 + x_4^2 + x_7^2) ) ;
Hk_r3c7 = (sqrt (x_1^2 + x_4^2+ x_7^2)/(x_1^2 + x_4^2 + x_7^2));

% Determine H matrix
       Hk_a = [x_1/r_hat_a, 0, 0, x_4/r_hat_a, 0, 0, x_7/r_hat_a, 0, 0;
               Hk_r2c1,     0, 0, Hk_r2c4,     0, 0, 0,           0, 0;
               Hk_r3c1,     0, 0, Hk_r3c4,     0, 0, Hk_r3c7,     0, 0];

     % Compute Kalman Gain
       K_accel = P_predict_accel*Hk_a'/(Hk_a *P_predict_accel * Hk_a' + R);

     % Kalman Filter Correction equations for Acclerating Model
       x_corr_accel = x_predict_accel + K_accel * z_tilde_a;
       P_corr_accel = (eye(9) - K_accel*Hk_a )* P_predict_accel;
   % Ballistic Model
     % Convert to relative position to compute polar coordinates
       x_1 = x_predict_ball(1) - Sensor_posit(1);
       x_3 = x_predict_ball(4) - Sensor_posit(2);
       x_5 = x_predict_ball(7) - Sensor_posit(3);

     % Convert prediction to polar coordinates
       r_hat_b = sqrt(x_1^2 + x_3^2 + x_5^2);
       b_hat_b = atan2(x_3, x_1);
       e_hat_b = atan2 (x_5, sqrt(x_1^2 + x_3^2));

     % Determine expected measurement
       z_polar_hat_b = [r_hat_b;
                        b_hat_b;
                        e_hat_b];
     % Observed minus expected measurements
       z_tilde_b = z_polar_n - z_polar_hat_b ;
% The gradient of H evaluated at the most recent estimate
Hk_r2c1 = -x_3/(x_1^2 + x_3^2);
Hk_r2c4 =  x_1/(x_1^2 + x_3^2) ;
Hk_r3c1 = (-x_1*x_5)/((sqrt(x_1^2 + x_3^2)) * (x_1^2 + x_3^2 + x_5^2));
Hk_r3c4 = (-x_3*x_5)/((sqrt(x_1^2 + x_3^2)) * (x_1^2 + x_3^2 + x_5^2));
Hk_r3c7 = (sqrt( x_1^2 + x_3^2+x_7^2)/(x_1^2 + x_3^2 + x_5^2));
     % Detkrmine H matrix
       Hk_b = [x_1/r_hat_b, 0, 0, x_3/r_hat_b , 0, 0, x_5/r_hat_b , 0, 0;
               Hk_r2c1,     0, 0, Hk_r2c4,      0, 0, 0             0, 0;
               Hk_r3c1,     0, 0, Hk_r3c4,      0, 0, Hk_r3c7,      0, 0];
     % Compute Kalman Gain
          K_ball = P_predict_ball * Hk_b'/(Hk_b*P_predict_ball*Hk_b' + R);
     % Kalman Filter Correction equations for the Ballistic Model
          x_corr_ball = x_predict_ball + K_ball * z_tilde_b;
          P_corr_ball = (eye(9) - K_ball*Hk_b ) * P_predict_ball;
%**************************************************************************
% Update mode probabilities
%**************************************************************************
     m = 3;
     S_1 = Hk_a * P_predict_accel * Hk_a' + R;
     lambda_1 = (exp(-(z_tilde_a)'/(S_1)* z_tilde_a/2))/(sqrt((2*pi)^m *...
                 det(S_1))) ;
     S_2 = Hk_b * P_predict_ball * Hk_b' + R;
     lambda_2 = ( exp(-(z_tilde_b)'/(S_2)* z_tilde_b/2))/...
                ( sqrt((2*pi)^m * det(S_2)) ) ;
     c = lambda_1 * cbar_1 + lambda_2 * cbar_2;
     mu_1 = lambda_1 * cbar_1/c;
     mu_2 = lambda_2 * cbar_2/c;

%**************************************************************************
% Produce Combined Estimates
%**************************************************************************
     x_corr = mu_1 * x_corr_accel + mu_2 * x_corr_ball;
     P_corr = mu_1*(P_corr_accel+(x_corr_accel -...
              x_corr)*(x_corr_accel-x_corr)')+...
              mu_2* (P_corr_ball + (x_corr_ball -...
              x_corr)* (x_corr_ball-x_corr)');

%**************************************************************************
     % Kalman track positions and difference between Kalman
     % and actual track position and actual target position
       zout_K_track = H*x_corr;
       track_diff = ztrue - zout_K_track;
       track_error = [track_error, sqrt(track_diff'*track_diff )] ;
     % Update KF track trajectory array
       K_track = [K_track, zout_K_track];
  end; % for ii = 2:2O:nsamples
%**************************************************************************
     if kk == 1, % create first output
          zoutmean_true = zout_true_n;
          mean_K_track = K_track;
          merror_track = track_error;
          merror = error_true;
     else % create output after 1st run
          zoutmean_true = zoutmean_true + zout_true_n;
          mean_K_track = mean_K_track + K_track;
          merror_track = merror_track + track_error;
          merror = merror + error_true;
     end; % if kk ==I, else
toc;
end; % for kk = 1:nloops
%**************************************************************************
% Compute Means
%**************************************************************************
 zoutmean_true = zoutmean_true/nloops;
 mean_track = mean_K_track/nloops;
 merror = merror/nloops;              % mean error between
                                      % measurement and true position
  merror_track = merror_track/nloops; % mean error between
                                      % EKF estimated position
                                      % and true position

%**************************************************************************
% Plot results
%**************************************************************************
figure(1)
  measurement = zoutmean_true/1000; % convert to km
  Kalman_track = mean_K_track/1000; % convert to km
  missile_track = missilevec(:,1: nsamples) /1000; % convert to km
  plot3(missile_track (2,:) ,missile_track(5,:) , missile_track(8,:),...
  Sensor_posit(1)/1000, Sensor_posit(2)/1000, Sensor_posit(3)/1000, 'r^');
  %axis( [0,150,0,150,0,150]);
  title('Ballistic Missile Base Trajectory - 120 seconds');
  xlabel('x - km'), ylabel('y - km'), zlabel('z - km'), grid;
  print -deps ch5f1a

figure(2)
  plot3(missile_track(2,:),missile_track(5,:), missile_track(8,:),...
  measurement(1,:),measurement(2,:),measurement(3,:),...
Sensor_posit(1)/1000, Sensor_posit(2)/1000, Sensor_posit(3)/1000, 'r^');
  %axis ( [0 150,0,150,0,150] ) ;
  title('Ba1listic Missile Base Trajectory with Measurement Noise - 120 seconds');
  xlabel('x - km'), ylabel('y - km'), zlabel('z - km'), grid;
  print -deps ch5f2a

figure(3)
  plot3(missile_track(2,:),missile_track(5,:),missile_track(8,:),...
     Kalman_track(1,:),Kalman_track(2,:),Kalman_track(3,:), 'r-');
  %axis ( [0, 150,0,150, 0, 150] ) ;
  xlabel ( 'x - km' ) , ylabel ( ' y - km' ) , zlabel( ' z - km' ) ,grid;
  title('Ba1listic Missile Base Trajectory and IMM Trajectory - 120 seconds');
  print -deps ch5f3a

figure(4)
  start_pt = 1;
  stop_pt = 401;
  zoom_missile = ((start_pt +1) : (stop_pt ));
  zoom_Kalman = (start_pt : stop_pt-1) ;
  plot3(missile_track (2,1:zoom_missile), missile_track(5,1:zoom_missile),...
  missile_track(8,1:zoom_missile),'bo-',...
  Kalman_track(1,1:zoom_Kalman),...
  Kalman_track(2,1:zoom_Kalman) , Kalman_track(3,1:zoom_Kalman), 'r^-' ) ;
  %axis([30,60,30,60,00,60]) ;
  xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)'),grid;
  title(['ZOOM - IMM Trajectory Initial',num2str((stop_pt - start_pt))/10,' Seconds']);
  print -deps ch5f4a

figure(5)
  start_pt = 1;
  stop_pt = 601;
  zoom_missile = ((start_pt +1 ) : (stop_pt ));
  zoom_Kalman = (start_pt : stop_pt-1) ;
  plot3(missile_track (2,1:zoom_missile) ,missile_track(5,1:zoom_missile),...
  missile_track(8,1:zoom_missile), 'bo-',...
  Kalman_track(1,1:zoom_Kalman),...
  Kalman_track(2,1:zoom_Kalman) , Kalman_track(3,1:zoom_Kalman), 'r^-' );
  %axis([30,60,30,60,0,60] ) ;
  xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)' ) ,grid;
  title(['ZOOM - IMM Trajectory Initial ',num2str((stop_pt - start_pt))/10,' Seconds']) ;
  print -deps ch5f5a

figure(6)
  start_pt = 1;
  stop_pt = 801;
  zoom_missile = ((start_pt +1) : (stop_pt));
  zoom_Kalman = (start_pt : stop_pt-1);
  plot3(missile_track (2,1:zoom_missile) ,missile_track(5,1:zoom_missile),...
  missile_track(8,1:zoom_missile), 'bo-',...
  Kalman_track(1,1:zoom_Kalman),...
  Kalman_track(2,1:zoom_Kalman) , Kalman_track(3,1:zoom_Kalman), 'r^-' ) ;
  %axis ( [30,60,30,60,0 ,6 0] ) ;
  xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)'),grid;
  title(['ZOOM - IMM Trajectory Initial ',num2str((stop_pt - start_pt)/10),' Seconds']);
  print -deps ch5f6a

figure (7)
  time = missilevec(1,:);
  diff_IMM_base = [Kalman_track(1,:) - missile_track(2,2:nsamples);
                   Kalman_track(2,:) - missile_track(5,2:nsamples);
                   Kalman_track(3,:) - missile_track(8,2:nsamples)];
  plot(time (2:nsamples), merror, 'g-',...
      time(2:nsamples), 1000*sqrt(diff_IMM_base(1,:).^2 +...
      diff_IMM_base(2,:).^2 + diff_IMM_base(3,:).^2), 'r-');
  xlabel('Time (seconds)'), ylabel('Error (meters)'), grid;
  title('IMM Distance Error vs. Time');
  legend('Mean Distance Error', 'IMM Distance Error');
  print -deps c5f7a

figure(8)
  plot(time(2:nsamples), merror,'g-', time(2:nsamples), merror_track,'r-');
  xlabel('Time (seconds) '),ylabel('Mean Error(meters)'),grid;
  title('Mean Distance Error in Measurements vs Time');
  %(' , num2str(nloops),'runs,', num2str(nsamples),' data_points)']),grid;
  print -deps c5f8a
save imm100