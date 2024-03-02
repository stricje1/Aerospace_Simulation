
   
%define constants 
R=82.0578/(10^3); %gas constant with units of m^3*atm/kg-mol/K
velocity(:,:) = sqrt(missilevec(3,:).^2+missilevec(6,:).^2+missilevec(9,:).^2);
mu = 1;

 for jj = 1:373,
  if missilevec(8,jj)*1000 < 0                % Travel inside the Earth is Viscous
          rho(jj,:) = 10^2;
     elseif missilevec(8,:)*1000 < 9144    % Altitudes below 9144 meters
          rho(jj,:) = 1.22557*exp(-1000*missilevec(8,:)/9144);
     else                % Altitudes above 9144 meters
          rho(jj,:) = 1.75228763*exp(-1000*missilevec(8,:)/6705.6);
  end

Re(1,jj)=rho(1,jj)*velocity(1,jj)'*D/mu;
end
Re(:)

% Re(:,:)=(velocity(1:373)/rho(1:373))*10e5;
 
%Define flat earth vector   

%FEV(:,ii)=[40*1000; 0; 0; 800*1000; 0; 0; 100000+i*1000; 0.001; 0];

for ii = 1:3100
    if ii==1
        FEV(:,ii)=[80*1000; 0; 0; 1000*1000; 0; 0; 0; 0.001; 0];
    else    
        FEV(:,ii)=FEV(:,ii-1)+[47.2; 0; 0; 219.9; 0; 0; 161.2; 0.0; 0];
    end    
end

% Plot
figure (20)
quiver3(30000,40000,0,0,0,500000)
hold all
plot3(missilevec(2,1:3100),missilevec(5,1:3100),missilevec(8,1:3100))
grid on
hold off
xlabel ( 'x' ) , ylabel( 'y' ) , zlabel( 'z' ) , grid;
     title('Missile Trajectory and Flat');

figure (21)
plot3(FEV(1,:),FEV(4,:),FEV(7,:))
hold all
plot3(missilevec(2,1:3100),missilevec(5,1:3100),missilevec(8,1:3100))
hold off
grid on

%Angle of attack
for kk = 1:3100
dot_x(kk,:)= dot([FEV(1,kk),FEV(4,kk),FEV(7,kk)]/10000,[missilevec(2,kk),missilevec(5,kk),missilevec(8,kk)]/10000)*pi/180;
end

%figure (22)
%plot(dot_x(1:3100),Re(:,:))

figure (23)
plot(missilevec(1,1:3100),dot_x(1:3100)*pi/180)
ylabel ( 'Angle of Attack (raidans)' ) , xlabel( 'Time (seconds)' ) , grid;
     title('Angle of Attaack vs Time');
%dot([FEV(1,1:4200),FEV(4,1:4200),FEV(7,1:4200)],[missilevec(2,1:4200),missilevec(5,1:4200),missilevec(8,1:4200)])

%figure (24)
%plot3(FEV_1,Missilevec_1,Dot_1)

for ij = 1:3100
    cl(ij)=300/(.5*rho(ij)*velocity(ij).^2'*.2);
end
cl(1:173)
figure (25)
plot(dot_x(1:3000)*pi/180,cl(1:3000).^(-1))
ylabel ( 'Coefficient of Lift' ) , xlabel( 'Angle of Attack (raidans)' ) , grid;
     title('Coefficient of Lift vs Angle of Attaack');

for ii = 1:373
    cd(ii)=(.072/Re(ii).^2'-(1670/Re(ii)'));
end
cd(:)
figure (26)
plot(missilevec(1,80:373),cd(80:373))
ylabel ( 'Coefficient of Drag' ) , xlabel( 'Angle of Attack (raidans)' ) , grid;
     title('ACoefficient of drag vs Angle of Attaack');

figure(29)
   plot3(missile_track(2,1:nsamples),missile_track(5,1:nsamples),missile_track(8,1:nsamples),'b-',...
     measurement(1,:), measurement(2,:), measurement(3,:), 'm-',...
     Sensor_posit(1)/1000,Sensor_posit(2)/1000, Sensor_posit(3)/1000,'rx');
     %axis ([0, 150,0,150, 0, 150]);
     title('Bal1istic Missile Base Trajectory with Measurement Noise');
     xlabel ('X (km)') , ylabel ('Y (km)') , zlabel ('Z (km)'),grid;
   hold all
   plot3(x_km,y_km,z_km,'b-');
   hold all
   plot3(Kalman_track(1:3090,1), Kalman_track(1:3090,2),...
           Kalman_track(1:3090,3), 'r-')
   hold off    
   
   figure(28)
   plot3(missile_track(2,1:nsamples),missile_track(5,1:nsamples),missile_track(8,1:nsamples),'b-',...
     measurement(1,:), measurement(2,:), measurement(3,:), 'm-',...
     Sensor_posit(1)/1000,Sensor_posit(2)/1000, Sensor_posit(3)/1000,'rx');
     %axis ([0, 150,0,150, 0, 150]);
     title('Bal1istic Missile Base Trajectory with Measurement Noise');
     xlabel ('X (km)') , ylabel ('Y (km)') , zlabel ('Z (km)'),grid;
   hold all
   plot3(x_km,y_km,z_km,'b-');
   hold all
   plot3([200;0;0.0003],[700;0;0],[500;0;0],'r-')
   
  
   for ii=1:missilevec(1,3909)
       qq(ii)=(missilevec(2,ii))*exp(-ii);
   end
   plot(missilevec(1,3909),qq)     