function [Vd_data, d_segments] = decide_eliteStrategy(sex_input, distance_input, interval_bool)
% UNTITLED Summary of this function goes here
%   Detailed explanation goes here

   % Check sex
        if strcmpi(sex_input, 'male')
            % Check distance for males
            if distance_input <= 100
                filename = 'pace__elite_100m_male.xlsx';
            elseif distance_input <= 400
               filename = 'pace__elite_400m_male.xlsx';
            elseif distance_input <= 800
                filename = 'pace__elite_800m_male.xlsx';
            elseif distance_input <= 1500
                filename = 'pace__elite_1500m_male.xlsx';
            elseif distance_input <= 5000
                filename = 'pace__elite_5000m_male.xlsx';
            else
                filename = 'pace__elite_10000m_male.xlsx';
            end
        elseif strcmpi(sex_input, 'female')
            % Check distance for females
            if distance_input <= 100
                filename = 'pace__elite_100m_female.xlsx';
            elseif distance_input <= 400
                filename = 'pace__elite_400m_female.xlsx';
            elseif distance_input <= 800
                filename = 'pace__elite_800m_female.xlsx';
            elseif distance_input <= 1500
                filename = 'pace__elite_1500m_female.xlsx';
            elseif distance_input <= 5000
                filename = 'pace__elite_5000m_female.xlsx';
            else
                filename = 'pace__elite_10000m_male.xlsx';
            end
        else
            result = 'Invalid sex input';
        end
   [Vd_data, d_segments] = getdata(filename); % ([m/s, m]) 
end