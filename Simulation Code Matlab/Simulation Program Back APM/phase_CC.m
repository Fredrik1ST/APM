function [v_ref, switch_var] = phase_CC(t, ti, Vi, Vd,  k_a, k_d, s_a, s_d, v_ref, switch_var)
% returns the velocity of the APM
%
% Inputs:
% t           = Current time                                    (s)
% t_prev      = Previous time                                   (s)
% Ts          = Sampling time                                   (s)
% Vi          = Desired initial velocity                        (m/s)
% Vd          = Desired velocity                                (m/s)
% k_a         = Sigmoid steepness parameter for acceleration    (1/s)
% k_d         = Sigmoid steepness parameter for deceleration    (1/s)
% s_a         = Sigmoid shift parameter for acceleration
% s_d         = Sigmoid shift parameter for deceleration
% p           = Current position                                (m)
% v_ref       = Current velocity                                (m/s)
% switch_var  = Switch parameter
%
% Outputs:
% v_ref       = Reference velocity                              (m/s)
% p_ref       = Reference position                              (m)
% switch_var  = Switch parameter
%
% Author:    Camilla Kvamme
% Date:      05.02.2024
%
% The following formulas are inspired from the paper titled "Path Generation
% for High-Performance Motion of ROVs Based on a Reference Model",
% by Daniel de A. Fernandes, Asgeir J. Sørensen, and Décio C. Donha.
% This paper was published in the journal "Modeling, Identification and Control" (Volume 36, Number 2, Pages 81-101) in the year 2015.
% The DOI for reference is 10.4173/mic.2015.2.2, and it was published by the Norwegian Society of Automatic Control.

    % Reference model
    switch switch_var
        case 1 % First phase: acceleration
            [v_ref] = acceleration_phase( t, ti, Vi, Vd, k_a, s_a);
        case 2
           % Second phase: constant velocity
            v_ref = Vd;
        case 3 % Third phase: deceleration  
            [v_ref] = acceleration_phase( t, ti, Vi, Vd, k_d, s_d); 

        case 4 % Forth phase: constant position    
            v_ref = 0;
    
        otherwise
            % Handle any other cases if needed
            display('Error');
    end
    
end
