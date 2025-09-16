function [time, velocity] = getdata(filename)
    % Read the data from the Excel file
    data =  readtable(filename);
    
    % Extract the time and velocity vectors
    time = table2array(data(:, 1)); % Assuming time is in the first column
    velocity = table2array(data(:, 2)); % Assuming velocity is in the second column
end