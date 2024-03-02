% MATLAB" CODE FOR PLOTTING MISSILE SIMULATION
% The following is the MATLAB" program used to plot the output of the
% SIMULINK model, FlatEPtMissileSim.m
%**************************************************************************
% MissileSimPlotsFlatEarth.m
%
% Dr. Jeffrey S. Strickland
%
% This file plots the results of the SIMULINK missile simulation
%**************************************************************************
  % Define Variables
     t  = missilevec(1,:);
     x  = missilevec(2,:);
     vx = missilevec(3,:);
     ax = missilevec(4,:);
     y  = missilevec(5,:);
     vy = missilevec(6,:);
     ay = missilevec(7,:);
     z  = missilevec(8,:);
     vz = missilevec(9,:);
     az = missilevec(10,:);

     x_km = x/1000;
     y_km = y/1000;
     z_km = z/1000;

     sxy = vx.^2 + vy.^2;
     speed = sqrt(sxy + vz.^2) ;
     ssxy = sqrt(sxy) ;
     dist = sqrt(x.^2 + y.^2);
     az = atan2(vy,vx)*180/pi;
     el = atan2(vz,ssxy)*180/pi;
     xaccel = ax/9.8;
     yaccel = ay/9.8;
     zaccel = az/9.8;
     total_accel = sqrt(xaccel.^2 + yaccel.^2 + zaccel.^2);
     
     
     tar_t  = targetvec(1,:);
     tar_x  = targetvec(2,:);
     tar_vx = targetvec(3,:);
     tar_ax = targetvec(4,:);
     tar_y  = targetvec(5,:);
     tar_vy = targetvec(6,:);
     tar_ay = targetvec(7,:);
     tar_z  = targetvec(8,:);
     tar_vz = targetvec(9,:);
     tar_az = targetvec(10,:);

     tar_x_km =  tar_x/1000;
     tar_y_km =  tar_y/1000;
     tar_z_km =  tar_z/1000;

     tar_sxy =  tar_vx.^2 +  tar_vy.^2;
     tar_speed = sqrt(tar_sxy +  tar_vz.^2);
     tar_ssxy = sqrt(tar_sxy);
     tar_dist = sqrt(tar_x.^2 +  tar_y.^2);
     tar_az = atan2(tar_vy, tar_vx)*180/pi;
     tar_el = atan2(tar_vz, tar_ssxy)*180/pi;
     tar_xaccel =  tar_ax/9.8;
     tar_yaccel =  tar_ay/9.8;
     tar_zaccel = tar_az/9.8;
     tar_total_accel = sqrt(tar_xaccel.^2+ tar_yaccel.^2+ tar_zaccel.^2);
     
     rho = 1.75228763*exp(-missilevec(8,:)/6705.6);

%**************************************************************************
% MISDIS: Hit-Miss Data
%**************************************************************************
%miss_distance(:,:)=([missilevec(1,:)/1000-[sqrt(abs([missilevec(2,:).^2+missilevec(5,:).^2+missilevec(8,:).^2]/1000-[targetvec(2,:).^2+targetvec(5,:).^2+targetvec(8,:).^2]/1000))]]);


%for ii = 2:6141
%    if miss_distance(2,ii-1)/1000 == 0;
%        intercept_time = missilevec(1,ii-1);
%        miss_distance_abs = miss_distance(2,ii-1)/1000;
%        hit_coordinates=[missilevec(2,round(intercept_time)),...
%            missilevec(5,round(intercept_time)),...
%            missilevec(8,round(intercept_time))]/1000;
%    elseif miss_distance(2,ii-1)/1000 <= .6;
%        intercept_time = missilevec(1,ii-1);
%        miss_distance_abs = miss_distance(2,ii-1)/1000;
%        hit_coordinates=[missilevec(2,round(intercept_time)),...
%            missilevec(5,round(intercept_time)),...
%            missilevec(8,round(intercept_time))]/1000;
%        break
%    elseif miss_distance(2,ii-1)/1000 <= 10;
%        intercept_time = missilevec(1,ii-1);
%        miss_distance_abs = miss_distance(2,ii-1)/1000;
%        hit_coordinates=[missilevec(2,round(intercept_time)),...
%            missilevec(5,round(intercept_time)),...
%            missilevec(8,round(intercept_time))]/1000;
%        
%    else    
%        intercept_time = 'no intercept';
%        miss_distance_abs = 'undetermined';
%        hit_coordinates = 'not hit';
%    end    
%        
%end

%intercept_time; 
%miss_distance_abs;
%hit_coordinates;
     

%**************************************************************************
% Plot Data
%**************************************************************************

figure (1)
     plot (x_km, z_km, ' b-',...
         tar_x_km,tar_z_km,'r-') ;
     axis square; grid on;
     xlabel('X (km)'),ylabel('Z (km)');
     title('Missile Z vs. X Plot');
     print -deps ch2fg2a

figure (2)
     plot (x_km, y_km, ' b-',...
         tar_x_km,tar_y_km,'r-') ;
     axis square; 
     grid on;
     xlabel('X (km)'), ylabel('Y (km)');
     title ('Missile Y vs. X Plot' ) ;
     print -deps ch2fg2b

figure (3)
     plot(t, (dist/1000), 'r-', 'LineWidth',2 ) ;
     axis square
     grid on
     ylabel('Down Range Dist (km)'), xlabel('Time (seconds)');
     title('Down Range Distance vs Time');
     print -deps ch2fg2c

figure ( 4 )
     plot(t, z_km, 'b-',...
         t,tar_z_km, 'r-') ;
     axis square; grid on;
     ylabel('Missile Altitide (km)'), xlabel('Time (seconds)');
     title('Missile Altitude vs Time (kilometers)');
     print -deps ch2fg2d

figure (5)
     plot(t,speed, 'r-', 'LineWidth',2 ) ;
     axis square,
     grid on
     ylabel('Missile Speed (m/s)'), xlabel('Time (seconds)');
     title('Missile Speed vs Time');
     print -deps ch2fg2e

figure (6)
     plot(t,az, 'r-') ;
     title('Missile Azimuth Heading vs Time');
     print -deps ch2fg2f

figure (7)
     plot(t,el, 'r-', 'LineWidth',2 ) ;
     title('Missi1e Elevation Angle vs Time');
     print -deps ch2fg2g

figure (8)
     plot(dist, z, 'r-' , 'LineWidth',2 ) ;
     axis square; grid on;
     title('Down Range Distance vs Height');
     print -deps ch2fg2h

figure (9)
     plot3(x, y, z, 'b-',...
         tar_x,tar_y,tar_z, 'r-') ;
     axis square ;
     grid on;
     ylabel ('Y (m)') , xlabel ('X (m)') , zlabel ('Z (m)');
     title('Three Dimensional Missile Trajectory in meters');
     print -deps ch2fg2i

figure (10)
     plot3(x_km,y_km,z_km,'b-',...
         tar_x_km,tar_y_km,tar_z_km, 'r-');
     grid on
     axis square; 
     ylabel('Y (km)'), xlabel('X (km)'), zlabel('Z (km)');
     title('Three Dimensional Missile Trajectory in kilometers');
     print -deps ch2fg2j

figure (11)
     plot3(x(1:2200), y(1:2200) ,z(1:2200) ,'b-',...
         tar_x(1:2200), tar_y(1:2200) ,tar_z(1:2200) ,'r-');
     axis square
     grid on
     ylabel('Y (m) '), xlabel('X (m)'), zlabel ('Z (m)');
     title('Missile Trajectory _ Initial 120 Seconds in meters');
     print -deps ch2fg2k

figure (12)
     plot3(x_km(1:2200), y_km(1:2200), z_km(1:2200) ,'r-', 'LineWidth',2 );
     axis square; grid on;
     ylabel('Y (km)'), xlabel('X (km)'), zlabel('Z (km)');
     title(' Missile Trajectory _ Initial 120 Seconds in kilometers');
     print -deps ch2fg21

figure (13)
     plot(t,xaccel, 'r-', 'LineWidth',2 ) ;
     ylabel ( 'gs ' ) , xlabel( 'Time (seconds) ' ) , grid;
     title('Missile Acceleration in X vs Time');
     print -deps ch2fg2m

figure (14)
     plot(t, yaccel, 'r-', 'LineWidth',2 ) ;
     ylabel ( ' gs ' ) , xlabel ( ' Time (seconds) ' ) , grid;
     title('Missile Acceleration in Y vs Time');
     print -deps ch2fg2n

figure (15)
     plot(t, zaccel, 'r-', 'LineWidth',2 ) ;
     ylabel(' gs ') , xlabel ('Time (seconds)') , grid;
     title('Missile Acceleration in Z vs Time');
     print -deps ch2fg20

figure (16)
     plot(t, total_accel, ' r- ', 'LineWidth',2 ) ;
     ylabel (' gs '), xlabel ('Time (seconds)') , grid;
     title('Missile Acceleration vs Time');
     print -deps ch2fg2p
     
figure (17)
plot3(x_km,y_km,z_km,'b-',...
         tar_x_km,tar_y_km,tar_z_km, 'r-',...
         missilevec(2,1)/1000, missilevec(5,1)/1000, missilevec(8,1)/1000,'bo-',...
         targetvec(2,1)/1000,targetvec(5,1)/1000,targetvec(8,1)/1000,'r^-');
     grid on
     axis square; 
     ylabel('Y (km)'), xlabel('X (km)'), zlabel('Z (km)');
     title(['intercept time= ',num2str(intercept_time)]);
     title({['Intercept Time = ',num2str(intercept_time),' seconds'],...
         ['Miss Distance = ',num2str(miss_distance_abs),' meters']});

figure (18)
plot3(x_km,y_km,z_km,'b-',...
  tar_x_km,tar_y_km,tar_z_km, 'r-',...
  missilevec(2,1)/1000, missilevec(5,1)/1000, missilevec(8,1)/1000,'bo-',...
  targetvec(2,1)/1000,targetvec(5,1)/1000,targetvec(8,1)/1000,'r^-',...
  missilevec(2,3090)/1000,missilevec(5,3090)/1000,missilevec(8,3090)/1000,'mo-');
     grid on
     axis square; 
     ylabel('Y (km)'), xlabel('X (km)'), zlabel('Z (km)');
     title(['intercept time= ',num2str(intercept_time)]);
     title({['Intercept Time = ',num2str(intercept_time),' seconds'],...
         ['Miss Distance = ',num2str(miss_distance_abs),' meters']}) ;    
 
figure (19)
   mv=[missilevec(2,:)/1000;missilevec(5,:)/1000;missilevec(8,:)/1000];
   tv=[ targetvec(2,:)/1000; targetvec(5,:)/1000; targetvec(8,:)/1000];
   theta(:,:)=acos(dot(mv,tv));
   plot(missilevec(1,:),imag(theta(:,:)*(pi/180)));
   ylabel ('Angle (radians)'), xlabel ('Time (seconds)') , grid;
     title('Closing Angle between Missile and Target');
