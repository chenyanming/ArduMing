delete(instrfindall)
clear all
s = serial('/dev/tty.usbmodem1411');
set(s,'BaudRate',115200);
set(s,'Terminator','CR/LF');
fopen(s);




time = now;
voltage = 0;
count = 0;
%% Set up the figure 
figureHandle = figure('NumberTitle','off',...
    'Name','Voltage Characteristics',...
    'Color',[1 1 1],'Visible','off');%color outside the axis area

% Set axes
axesHandle = axes('Parent',figureHandle,...
    'YGrid','on',...
    'YColor',[0 0 0],...%color of the y axis 
    'XGrid','on',...
    'XColor',[0 0 0],...%color of the x axis
    'Color',[1 1 1]);%color inside the axis area

hold on;

plotHandle = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[0 0 1]);%color of line into the plot!
%plotHandle1 = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[1 1 0]);%color of line into the plot!
plotHandle2 = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[1 0.5 0.5]);
plotHandle3 = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[1 0 0]);
plotHandle4 = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[	0.58	0.40	0.35]);
%plotHandle5 = plot(axesHandle,time,voltage,'Marker','.','LineWidth',1,'Color',[	0.58	0	0.35]);

xlim(axesHandle,[min(time) max(time+0.001)]);

% Create xlabel
xlabel('Time','FontWeight','bold','FontSize',14,'Color',[0 0 0]);

% Create ylabel
ylabel('Pitch in Angle','FontWeight','bold','FontSize',14,'Color',[0 0 0]);

% Create title
title('Real Time Data','FontSize',15,'Color',[0 0 0]);





legend('Pitch','Throttle 1', 'Throttle 3', 'Pitch Setpoint');

for count = 1:2000

    %a(count,:) = fscanf(s, '%f')
    output{count,:} = fgets(s); %Get one line string, store into cell array ,the "count" row: ouput{count,:}
    output{count} = deblank(output{count});%delete the blank of head and tail
    split(count,:) = regexp(output{count}, '\s+', 'split');% Split the ouput by one or more blanks, store into array, the "count" row: split(count,:)
   
    %
    % The pitch setpoint
    %
    pitch_setpoint_cell(count) = split(count,2);
    pitch_setpoint_double_tmp = str2double(pitch_setpoint_cell(count));% String to double
    pitch_setpoint_double(count) = str2double(pitch_setpoint_cell(count));% String to double    
    
    
    %
    % The pitch
    %
    pitch_cell(count) = split(count,3);% Store the third char of split(count,3) into pitch_cell(count)
    pitch_double_tmp = str2double(pitch_cell(count));% String to double
    pitch_double(count) = str2double(pitch_cell(count));% String to double
  
    %
    % The pitch pid output
    %
    %pitch_pid_cell(count) = split(count,4);
    %pitch_pid_double_tmp = str2double(pitch_pid_cell(count));
    %pitch_pid_double(count) = str2double(pitch_pid_cell(count));  
    
    %
    % Throttle1
    %
    throttle1_cell(count) = split(count,5);
    throttle1_double_tmp = str2double(throttle1_cell(count));
    throttle1_double(count) = str2double(throttle1_cell(count));   

 
    %
    % Throttle3
    %
    throttle3_cell(count) = split(count,6);
    throttle3_double_tmp = str2double(throttle3_cell(count));
    throttle3_double(count) = str2double(throttle3_cell(count)); 
  
    %
    % GyroX
    %
    %GyroX_cell(count) = split(count,7);
    %GyroX_double_tmp = str2double(GyroX_cell(count));
    %GyroX_double(count) = str2double(GyroX_cell(count)); 
      
    time(count) = count;
    
    set(plotHandle,'YData',pitch_double,'XData',time); 
    %set(plotHandle1,'YData',pitch_pid_double,'XData',time);
    set(plotHandle2,'YData',throttle1_double,'XData',time);
    set(plotHandle3,'YData',throttle3_double,'XData',time);
    set(plotHandle4,'YData',pitch_setpoint_double,'XData',time);
    %set(plotHandle5,'YData',GyroX_double,'XData',time);
    
    set(figureHandle,'Visible','on'); 
    
    datetick('x',0);
    pause(0.000001);
    
end

fclose(s)
delete(s)

%figure(1);

%n=1:1:100;
%u=pitch_double_tmp(n,1)
%plot(n,u)





