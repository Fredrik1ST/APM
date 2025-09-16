function [v] = acceleration_phase( t, ti, Vi, Vd, k, s)
% returns the velocity during acceleration
%
% Inputs:
% t         = Time                                            (s)
% t_prev    = Previous time                                   (s)
% Vi        = Initial velocity                                (m/s)
% Vd        = Desired velocity                                (m/s)
% n_d       = Sigmoid power parameter for acceleration        (unitless)
% k         = Sigmoid steepness parameter for acceleration    (1/s)
% s         = Sigmoid shift parameter                         (s)
% p         = Current position                                (m)
%
% Outputs:
% v_ref       = Reference velocity                            (m/s)
%
% Author:    Camilla Kvamme
% Date:      05.02.2024
%
% The following formulas are inspired from the paper titled "Path Generation
% for High-Performance Motion of ROVs Based on a Reference Model",
% by Daniel de A. Fernandes, Asgeir J. Sørensen, and Décio C. Donha.
% This paper was published in the journal "Modeling, Identification and Control" (Volume 36, Number 2, Pages 81-101) in the year 2015.
% The DOI for reference is 10.4173/mic.2015.2.2, and it was published by the Norwegian Society of Automatic Control.   


    % Sigmoid function
    sigmoid = 1/(1 + exp(-k * ((t-ti)-s)));
    
    % Sigmoid-based transition between velocities
    V_sigmoid = Vi + (Vd - Vi) * sigmoid;

    % % Find velocity 
    v = V_sigmoid; 

end

