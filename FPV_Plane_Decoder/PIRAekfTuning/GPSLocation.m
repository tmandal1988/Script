% Name GPSLocation

%      AP=[8.557692462510719e+05  -4.842955753230331e+06  4.048121128128226e+06];%Car Test1 Org

% AP=[8.557546255905735e+05  -4.842951687411269e+06  4.048122354534138e+06];%Car Test2 Org

AP=[8.557546841855366e+05  -4.842953610053775e+06  4.048123629969336e+06];

    Dis=6369948.151050708;
    %Lat=39.0933006;
    %Lon=-80.47024365;
    Lat=asin(AP(3)/Dis)*(180/pi);
    Lon=180/pi*asin(AP(2)/sqrt(AP(1)^2+AP(2)^2));
    R=inv(x2t([AP';1;0;(90-Lat)*pi/180;Lon*pi/180;0],'rpy'));
    I=eye(9);

