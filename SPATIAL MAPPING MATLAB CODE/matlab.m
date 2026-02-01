%{
    Elisabeth Mark
    COMPENG 2DX3
    Project Deliverable 2
%}

% clearing workspace
clear;

% finding available serial ports
ports = serialportlist("available");
serial_port = serialport(ports(1), 115200, 'Timeout', 120); % connect to serial port COM3 for UART communication with board
flush(serial_port);

% Parameters for measurements
depth_total = 3; % number of depth steps (x-axis)
num_measurements = 64; % measurements per rotation - 64*5.625 = 360
measurements = zeros(0,3); %empty matrix to hold data

cur_point = 0;

%going through all points 
while (cur_point <= depth_total * num_measurements-1) % Read until all data recieved
    % Read data from serial port
    serial_data = readline(serial_port);
    
    %break loop if there is nothing there
    if (isempty(serial_data))
        break;
    end

    %get and store data in matrix
    temp = parse(serial_data);

    %return columns of data that has relevant info
    temporary_matrix = temp([2 3 4]);
    
    disp(temporary_matrix);

    if (temporary_matrix(1) == 0) 
        % bad measurement from ToF
    else
        %appends valid data to the measurements matrix
        measurements = vertcat(measurements, temporary_matrix); %#ok<AGROW> 
    end
   
    cur_point = cur_point + 1;
end

measurement_data = measurements.'; % Transpose the data into a new N * M matrix

% Convert polar to Cartesian coordinates
[x, y, z] = pol2cart(measurement_data(2,:).*(pi/180), measurement_data(1,:), measurement_data(3,:));
cart_data = [z; y; x]; % Flipped X and Z

% convert z and y to meters
cart_data(2,:) = cart_data(2,:)./1000;
cart_data(3,:) = cart_data(3,:)./1000;

%for converting x data to 10 cm
cart_data(1,:) = cart_data(1,:).*10;

% Create 3D scatter plot
figure;
scatter3(cart_data(1,:), cart_data(2,:), cart_data(3,:), 'filled');
hold on;

% Connect the points with the same angle and depth difference of 1
for d = 1:depth_total
    
    % connect ring loops
    offset = (d-1)*num_measurements;

    % plot points in pairs to connect with straight lines
    for cur_point = 1:num_measurements-1
        plot3(cart_data(1,offset+cur_point:offset+cur_point+1), cart_data(2,offset+cur_point:offset+cur_point+1), cart_data(3,offset+cur_point:offset+cur_point+1), 'k-');
    end
    % Final point in the ring
    plot3([cart_data(1,offset+1), cart_data(1,offset+num_measurements)], [cart_data(2,offset+1), cart_data(2,offset+num_measurements)], [cart_data(3,offset+1), cart_data(3,offset+num_measurements)], 'k-');
end


for d = 1:depth_total-1
    for cur_point = 1:num_measurements
        % connect adjacent rings
        p1 = (d-1)*num_measurements + cur_point;
        p2 = d*num_measurements - cur_point + num_measurements+1;
        plot3([cart_data(1,p1), cart_data(1,p2)], [cart_data(2,p1), cart_data(2,p2)], [cart_data(3,p1), cart_data(3,p2)], 'k-');
    end
end

% Output 
hold off;
title('Spatial Visualization');
xlabel('X (Depth) (cm)');
ylabel('Y (Width) (m)');
zlabel('Z (Height) (m)');
grid on;

% Function to parse data from the UART line
function parsed_data = parse(n)
    
    % n will be the UART string
    string_in = n;
    
    % Print out the incoming string (just for validation)
    disp("Raw Data: "+ string_in);

    % Parse variables separated by commas
    parsed = sscanf(string_in, '%f,%f,%f,%f,%f');
    
    % Store parsed variables in a variable
    check_bit = parsed(1);
    distance = parsed(2);
    angle = parsed(3)*5.625/8; % Steps to deg
    depth = parsed(4);
    spad_num = parsed(5);
    
    % Return the parsed data as a matrix
    parsed_data = [check_bit, distance, angle, depth, spad_num];

end
