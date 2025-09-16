
%Find Transferfunction of the APM
% max_speed = 36.11111; % m/s
file_name_TF = 'transferfunc_values';
[input, output] = getdata(file_name_TF);
data= [input, output];
np = 2;

%Transferfunc
sys = tfest(data(:, 1:2),np);
tf(sys)