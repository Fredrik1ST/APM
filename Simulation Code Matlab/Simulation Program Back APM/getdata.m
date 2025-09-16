function [data1, data2] = getdata(filename)
    % Read the data from the Excel file
    data =  readtable(filename);
    
    % Extract the time and velocity vectors
    data1 = table2array(data(:, 1)); % Assuming time is in the first column
    data2 = table2array(data(:, 2)); % Assuming velocity is in the second column
end