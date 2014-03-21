%hist(vel)

% brake: the braking input at a given time, between values of 0 and ~650 corresponding to pressure
% throttle: gas input to the car, given as a percentage 
% delta: steering input for the angle of the steering wheel
% # car_offset_rt: distance of the car from the right curb 
% # frontObj,rightObj,leftObj: processed radar data where rows are (1) number of objects detected, (2) distance to nearest object, (3) detection angle to nearest object, (4) velocity of nearest object, and (5) heading angle of nearest object.
% # heading: heading angle of the car, relative to the gobal reference frame 
% # roadBndLeft#X: x-coordinate of road boundary on the left of the car from sensor distance 0,1,2, or 3, denoted by #, which are 10m apart
% # roadBndLeft#Y: follows above for the y-coordinate
% # roadBndRight#X/Y: same as above for the x and y coordinates for the right boundary
% # vel: velocity of the car in m/s
% # xData: x position in the global frame
% # yData: y position in the global frame
load sample_sim_data.mat
close all;

t0 = 1;
t1 = t0;
te = length(xData);
dt = 500/te;
figure('units','normalized','outerposition',[0 0 0.5 1]);
for t1 = [t0:10:te]
    pause(0.01);
    clf;
   % axes('Position',[.005 .005 .99 .99],'xtick',[],'ytick',[],'box','on','handlevisibility','off')
    subplot('position', [0 0 1 1]);
    plot(xData, yData, 'k-', 'LineWidth', 1)
    hold on;
    plot([-200:dt:300-dt], throttle/2+350, 'k-');
    plot([-200:dt:300-dt], delta/5+450, 'b-');
    plot([-200:dt:300-dt], brake/650*50+400, 'r-');
    plot((-200+dt*t1).*[1 1], [300, 500], 'g-', 'LineWidth', 2);
    plot([-175, -175], [200 200+throttle(t1)], 'k-', 'LineWidth', 8)
    plot([-150, -150], [200 200+brake(t1)/650*100], 'r-', 'LineWidth', 8)
    cx = [-75, 250]; r= 50;
    th = [-260:260]./180*pi();
    [x y] = pol2cart(th, r);
    plot(x+cx(1), y+cx(2), 'b-');
    th = pi/2+[0:sign(delta(t1)):delta(t1)]./180*pi();
    [x y] = pol2cart(th, r);
    plot(x+cx(1), y+cx(2), 'b-', 'LineWidth', 8);
    
    
    
    plot(xData(t0:t1), yData(t0:t1), 'k-', 'LineWidth', 1.5);
    plot(xData(t1), yData(t1), 'b.', 'MarkerSize', 30)
    quiver(xData(t1), yData(t1), vel(t1)*sind(heading(t1)), vel(t1)*cosd(heading(t1)), 'r.', 'LineWidth', 2)
%     fo
    z = frontObj(:,t1);
    if(z(1)>0)
        x = xData(t1)+sind(heading(t1)+z(3))*z(2);
        y = yData(t1)+cosd(heading(t1)+z(3))*z(2);
        plot(x, y, 'r.', 'MarkerSize', 20)
    end
    z = rightObj(:,t1);
    if(z(1)>0)
        x = xData(t1)+sind(heading(t1)+z(3)+90)*z(2);
        y = yData(t1)+cosd(heading(t1)+z(3)+90)*z(2);
        plot(x, y, 'r.', 'MarkerSize', 20)
    end
    z = leftObj(:,t1);
    if(z(1)>0)
        x = xData(t1)+sind(heading(t1)+z(3)-90)*z(2);
        y = yData(t1)+cosd(heading(t1)+z(3)-90)*z(2);
        plot(x, y, 'r.', 'MarkerSize', 20)
    end
    LBfx = [roadBndLeft0X(t1) roadBndLeft1X(t1)  roadBndLeft2X(t1)  roadBndLeft3X(t1)];
    LBfy = [roadBndLeft0Y(t1) roadBndLeft1Y(t1)  roadBndLeft2Y(t1)  roadBndLeft3Y(t1)];
    s = car_offset_rt(t1);
    RBfx = [xData(t1)+sind(heading(t1)+90)*s roadBndRight0X(t1) roadBndRight1X(t1)  roadBndRight2X(t1)  roadBndRight3X(t1)];
    RBfy = [yData(t1)+cosd(heading(t1)+90)*s roadBndRight0Y(t1) roadBndRight1Y(t1)  roadBndRight2Y(t1)  roadBndRight3Y(t1)];
    plot(LBfx, LBfy, 'm-')
    plot(RBfx, RBfy, 'm-')
    xlim([-200, 300])
    ylim([-200 500]);
    drawnow();    
end
%%
t1 = 16500;
hold on;
quiver(xData(t0:10:t1), yData(t0:10:t1), sind(heading(t0:10:t1)).*vel(t0:10:t1)/10, cosd(heading(t0:10:t1)).*vel(t0:10:t1)/10, 1, 'r.')


%%

close all;

quiver([t0:10:t1]', repmat(0, length(t0:10:t1),1), 0.*vel(t0:10:t1)/10, 1.*vel(t0:10:t1)/10, 1, 'r.')