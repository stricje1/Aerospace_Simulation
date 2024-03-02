%The following is the MATLAB@ program used in the tracking of the ballistic
%missile base trajectory .
%**************************************************************************
% efk.m
% Dr. Jeffrey S. Strickland
%
% This program uses an EKF to filter the sensor measurement noise
% from the ballistic missile base trajectory developed using
% SIMULINK Random noise is added in the sensor
% measurement process. Actual missile track is generated in
% FlatEarthMissle SIMULINK model.
%
% INPUT
% missilevec: state vector = [x,Vx,Ax, y,Vy, Ay, z,Vz,Az] '
%
% OUTPUT
% mean_K_track Kzilman estimated positions
%**************************************************************************
% Load simulation workspace
     %clear all
     %load tbm_dat;
     missilevec = targetvec;
  % Define the number of simulation loops
     nloops = 10;
  % Define the sampling interval
     delta = .1;
  % Define the number of samples
     nsamples = 6140;

  % Initialize sensor data
     Sensor_posit =[100 * 1000;   % sensor is 100 km in x
                     100 * 1000;   % sensor is 100 km in y
                       0 * 1000];  % sensor is 100 km in z

     sigma_r = 10;                 % 10 meters std dev in range
     sigma_b = 1*pi/180;           % 1 degree std dev in azimuth
     sigma_e = 1*pi/180;           % 1 degree std dev in elevation

     R = diag ( [sigma_r^2;        % covariance matrix for uncorrelated
                 sigma_b^2;       % range and bearing measurements
                 sigma_e^2]);

  % Define F matrix (TRANSITION MATRIX) for discrete time
  % target motion, x(kt1) = F(k)*x(k) + G
     f_sub = [1, delta, (delta^2)/2;
               0,   1,      delta   ;
               0,   0,        1    ];

     F = [ f_sub,    zeros(3), zeros(3) ;
           zeros(3), f_sub,    zeros(3) ;
           zeros(3), zeros(3),  f_sub  ];

  % Define G matrix
     G = -g * [0;
               0;
               0;
               0;
               0;
               0;
              (delta^2)/2;
               delta;
               0] ;

  % Define the H matrix (MEASUREMENT MATRIX), assuming that the
  % x, y, an z missile positions are observed directly; z(k) = H(k) *x(k)
     H = [1,0,0,0,0,0,0,0,0;
          0,0,1,0,0,0,0,0,0;
          0,0,0,0,0,0,1,0,0];

  % Initialize Q, the covariance of the plant noise
  % q^2 = scale factor to system noise covariance matrix Q,
  % used to account for unmodeled target maneuver acceleration.
     q_sqr = 10;

     Q_sub = [ (delta^5)/20,(delta^4)/8, (delta^3)/6;
              (delta^4)/8, (delta^3)/3, (delta^2)/2;
              (delta^3)/6, (delta^2)/2,  delta ]   ;

     Q = q_sqr*[Q_sub, zeros(3), zeros(3);
                zeros(3), Q_sub, zeros(3);
                zeros(3), zeros(3), Q_sub ];

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
     K_track = [] ;
     K_accel = [] ;

  % error between Kalman track and actual track
     track_error = [] ;

%**************************************************************************
% This block is used for the initialization process. Initialize
% from a SWAG
%**************************************************************************
     x_swag = missilevec(2:10,1) ;
     x_swag(9) = 6*g;
     p_swag = eye(9) * 10^4;
     x_corr = x_swag;
     P_corr = p_swag;
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
% convert current position to polar coordinates and add
% sensor noise to the position, generating a noisy measurement
% from the sensor.
%**************************************************************************
  % position relative to the sensor
     zrel = ztrue - Sensor_posit;
  % range from sensor
     r = sqrt(zrel(1)^2 + zrel(2)^2 + zrel(3)^2) ;

  % bearing from sensor
     b = atan2(zrel(2), zrel(1));

  % range in x-y plane
     r_prime = sqrt(zrel(1)^2 + zrel(2)^2);

  % elevation from sensor
     e = atan2(zrel(3), r_prime);

  % add noise to the measurement
     r_n = r + sigma_r * randn;
     b_n = b + sigma_b * randn;
     e_n = e + sigma_e * randn;

  % measurement in polar + noise
     z_polar_n = [r_n;
                  b_n;
                  e_n];

  % measurement in cartesian coordinates + noise
     z_cart_true_n = [r_prime*cos(b_n);
                      r_prime*sin(b_n);
                      r_n*sin(e_n)] + Sensor_posit;

     z_cart_rel_n = [r_prime*cos(b_n);
                     r_prime*sin(b_n);
                     r_n*sin(e_n)];

  % compute measurement error in cartesian coordinates
     zdiff = ztrue - z_cart_true_n;
     disterror = sqrt(zdiff'*zdiff);
  % Update the measurement array
  % true cartesian measurement + error
     zout_true_n = [zout_true_n, z_cart_true_n];

  % measurement error (between true measurements)
     error_true = [error_true, disterror] ;

%**************************************************************************
% Prediction
%**************************************************************************
  % Kalman Filter prediction equations
     x_predict = F * x_corr + G;
     P_predict = F * P_corr * F' + Q;

%**************************************************************************
% Correction
%**************************************************************************
  % Convert to relative position to compute RBE coordinates
     x_1 = x_predict(1) - Sensor_posit(1);
     x_4 = x_predict(4) - Sensor_posit(2);
     x_7 = x_predict(7) - Sensor_posit(3);

  % Convert prediction to Range, Bearing, Elevation coordinates
     r_hat = sqrt(x_1^2 + x_4^2 + x_7^2);
     b_hat = atan2 (x_4, x_1);
     e_hat = atan2(x_7, sqrt(x_1^2 + x_4^2)) ;

  % Determine expected measurement
     z_polar_hat = [r_hat ;
                    b_hat ;
                    e_hat];

  % Observed minus expected measurements
     z_tilde = z_polar_n - z_polar_hat;

  % The gradient of H evaluated at the most recent estimate
     Hk_r2c1 = -x_4/(x_1^2 + x_4^2);
     Hk_r2c4 =  x_1/(x_1^2 + x_4^2);
     Hk_r3c1 = (-x_1*x_7)/((sqrt(x_1^2 + x_4^2))*(x_1^2 + x_4^2 + x_7^2));
     Hk_r3c4 = (-x_4*x_7)/((sqrt(x_1^2 + x_4^2))*(x_1^2 + x_4^2 + x_7^2));
     Hk_r3c7 = (sqrt(x_1^2 + x_4^2))/(x_1^2+ x_4^2 + x_7^2);

       % Determine H matrix
          Hk = [x_1/r_hat, 0, 0, x_4/r_hat, 0, 0, x_7/r_hat, 0, 0;
                Hk_r2c1,   0, 0, Hk_r2c4,   0, 0, 0 ,        0, 0;
                Hk_r3c1,   0, 0, Hk_r3c4,   0, 0, Hk_r3c7,   0, 0];

            % Compute Kalman Gain
               K = P_predict * Hk'/(Hk * P_predict * Hk' + R );

            % Correction equations
               x_corr = x_predict + K * z_tilde;
               P_corr = (eye(9) - K*Hk)*P_predict*(eye(9) - K*Hk)'+ K*R*K';
            % Kalman track positions and difference between Kalman
            % and actual track position and actual target position
               zout_K_track = H*x_corr;

               track_diff = ztrue - zout_K_track;
               track_error = [track_error, sqrt(track_diff'*track_diff)];

            % Update KF track trajectory array
               K_track = [K_track, zout_K_track] ;

            % Estimated accelerations
               accel_out = [x_corr(3,:);
                            x_corr(6,:);
                            x_corr(9,:)];

           % Update KF acceleration array
               K_accel = [K_accel, accel_out];

  end;                               % for ii = 2:nsamples

%**************************************************************************
     if kk == 1, % create first output
          zoutmean_true = zout_true_n;
          mean_K_track = K_track;
          merror_track = track_error;
          merror = error_true;
     else                                    % create output after 1st run
          zoutmean_true = zoutmean_true + zout_true_n;
          mean_K_track = mean_K_track + K_track;
          merror_track = merror_track + track_error;
          merror = merror + error_true;
     end;                                    % if kk ==I, else
toc;
end;      % for kk = 1:nloops

%**************************************************************************
% Compute Means
%**************************************************************************
     zoutmean_true = zoutmean_true/nloops;
     mean_K_track  = mean_K_track/nloops;
           merror  = merror/nloops;          % mean error between
                                             % measurement and true position
     merror_track = merror_track/nloops;     % mean error between
                                             % EKE estimated position
                                             % and true position

%**************************************************************************
% Plot results
%**************************************************************************
figure (1)
     measurement = zoutmean_true/1000;                 % convert to km
     Kalman_track = mean_K_track/1000;                 % convert to km
     missile_track = missilevec(:,1:nsamples)/1000;    % convert to km
     plot3(missile_track(2,1:nsamples),missile_track(5,1:nsamples),missile_track(8,1:nsamples),'b-',...
           Sensor_posit(1)/1000, Sensor_posit(2)/1000,...
           Sensor_posit(3)/1000, 'rx');
    % axis([0, 150,0,150, 0, 150]);
     title('Ba1listic Missile Base Trajectory - 120 seconds');
     xlabel ('X (km) '), ylabel ('Y (km)'), zlabel('Z (km)') , grid;
     %print _deps c3p1s1

figure(2)
   plot3(missile_track(2,1:nsamples),missile_track(5,1:nsamples),missile_track(8,1:nsamples),'b-',...
     measurement(1,:), measurement(2,:), measurement(3,:), 'm-',...
     Sensor_posit(1)/1000,Sensor_posit(2)/1000, Sensor_posit(3)/1000,'rx');
     %axis ([0, 150,0,150, 0, 150]);
     title('Bal1istic Missile Base Trajectory with Measurement Noise');
     xlabel ('X (km)') , ylabel ('Y (km)') , zlabel ('Z (km)'),grid;

figure(3)
   plot3(missile_track(2,1:6139),missile_track(5,1:6139),missile_track(8,1:6139),'b-',...
       Kalman_track(1,:),Kalman_track(2,:),Kalman_track(3,:), 'm-',...
       Sensor_posit(1)/1000,Sensor_posit(2)/1000,Sensor_posit(3)/1000,'rx');
     %axis([0,150,0,150,0,150]);
     xlabel( 'X (km)') , ylabel ('Y (km)') , zlabel ('Z (km) ') ,grid;
     title('Bal1istic Missile Base Trajectory and EKF Trajectory - 120 seconds ');
     %print _deps c3p1s3

figure (4)
     start_pt = 1;
     stop_pt = 10;
     zoom_missile = ((start_pt + 1 ) : (stop_pt));
     zoom_Kalman= (start_pt : stop_pt-1);
     plot3(missile_track(2,1:nsamples),missile_track(5,1:nsamples),...
           missile_track(8,1:nsamples),'b-',...
           Kalman_track(:,1), Kalman_track(:,2),...
           Kalman_track(:,3), 'r-');
     %axis([0,30,0,10,0,60]);
     xlabel('X (km)') , ylabel('Y (km)'), zlabel('Z (km)'),grid;
     title(['ZOOM - EKF Trajectory Initial ',num2str((stop_pt - start_pt)),' Seconds']);
     %print -deps c3p1s4

figure(5)
     time = missilevec (:,1);
     time = missilevec (:,1);
     diff_k_base = [(Kalman_track(1,:)) - missile_track(2,2:nsamples);
                    (Kalman_track(2,:)) - missile_track(5,2:nsamples);
                    (Kalman_track(3,:)) - missile_track(8,2:nsamples)];
     plot(missilevec(1,1:6139), merror, 'b-' ,...
          missilevec(1,1:6139), 1000*sqrt(diff_k_base(1,1:6139).^2),'r-');
     xlabel('Time (seconds)'), ylabel('Error (meters)'), grid;
     title('EKF Distance Error vs. Time');
     legend('Mean Distance Error','EKF Distance Error');
     %print -deps c3p1s5

    plot(xinit(1),xinit(4))
