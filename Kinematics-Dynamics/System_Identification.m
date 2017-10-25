%% Parameter values
% Electric circuit
p = 6;            % Number of poles of the ac motor
R = 378e-3;       % Stator Resistance - Ohm
Ld = 3.427e-3;    % Inductance in the other H
Lq = 3.334e-3;    % Inductance in stator H
Ke = 1.13;        % back emf

% Mechanical crank shaft
Jt = 57e-4;       % Total inertia     
Kt = 1.39;        % Torque constant
ba = 0.1;         % viscous damping
Tl = 1e-4;        % Torque load

% Kinematics 
l = 80;           % length of the rod
r = 22.5;         % radius of crank
q = 9;            % ofset

% Double mass
M = 10;           % Mass of the load
Jm = 6.375e-04;   % Inertia of the motor
ks = 48.75;       % spring stiffnes
kb = 4.875;       % joint stiffnes
bl = 0.1;         % damping in the spring

% PI velocity 
c1_v = 7.813;
c2_v = 4.867;
c3_v = 1;
c4_v = 100.4;

% PI current
c1_i = 30;
c2_i = 1.05e+04;
c3_i = 1;
c4_i = 510;


%% Establishing data set
prompt = 'Current or velocity? (Current = 1/Velocity = 2/Position = 3)';
kvob = input(prompt);

[y,u,t_sample] = input_output(kvob);
z = iddata(y,u,t_sample,'Name','system_data');
%% Optimisation
% Three options are avaibale
if kvob == 1 % Current
    z.InputName = {'Current_d','Currnet_q'};
    z.InputUnit =  {'A','A'};
    z.OutputName = {'Current_d','Currnet_q'};
    z.OutputUnit = {'A','A'};

    FileName      = 'equations_system_i';                                                    
    Order         = [2 2 4];                                                                  
    Parameters    = [c1_i; c2_i; c4_i; R; Ld; Lq];                                              
    InitialStates = [0; 0; 0; 0];                                                             
    Ts            =  0;                                                                       

    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts,...
        'Name', 'current_model');

    set(nlgr, 'InputName', {'Current_d','Currnet_q'}, 'InputUnit',...
        {'A','A'},'OutputName', {'Current_d','Currnet_q'},...
        'OutputUnit', {'A','A'},'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {0,0,0,1,1});
    nlgr.Parameter(1).Minimum = 0;
    nlgr.Parameter(2).Minimum = 0;
    nlgr.Parameter(3).Minimum = 0;
    nlgr = setinit(nlgr, 'Fixed', {false false false false});
    disp('We are optimising');

    opt = nlgreyestOptions('Display', 'on');
    opt.SearchOption.MaxIter = 10;
    nlgr_opt = nlgreyest(z, nlgr, opt);
  
elseif kvob == 2 % Velocity
    z.InputName = {'Velocity ref'};
    z.InputUnit =  {'rad/s'};
    z.OutputName = {'Velocity feedback'}; 
    z.OutputUnit = {'rad/s'};
    z.Tstart = t_sample;
    z.TimeUnit = 's';

    FileName      = 'equations_system_v';                                                      
    Order         = [1 1 7];                                                                  
    Parameters    = [c1_v; c2_v; c3_v; c4_v; Kt; ba; Jt];                                        
    InitialStates = [0; 0; 0; 0; 0; 0; 0];                                                     
    Ts            =  0;                                                                        
    
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts,...
        'Name', 'velocity_model');

    set(nlgr, 'InputName', {'Velocity ref'}, 'InputUnit', {'rad/s'},...
              'OutputName', {'Velocity feedback'}, 'OutputUnit',...
              {'rad/s'}, 'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {0,0,0,0,0,0,0});
    nlgr.Parameter(1).Minimum = 0;
    nlgr.Parameter(4).Minimum = 0;
    nlgr.Parameter(5).Minimum = 0;
    nlgr.Parameter(6).Minimum = 0;
    nlgr.Parameter(7).Minimum = 0;
    nlgr = setinit(nlgr, 'Fixed', {false false false false false...
        false false});
    disp('We are optimising');

    opt = nlgreyestOptions('Display', 'on');
    opt.SearchOption.MaxIter = 10;
    nlgr_opt = nlgreyest(z, nlgr, opt);

else % position
    z.InputName = {'Position_ref'};
    z.InputUnit =  {'mm'};
    z.OutputName = {'Position_feed'}; 
    z.OutputUnit = {'mm'};
    z.Tstart = t_sample;
    z.TimeUnit = 's';

    FileName      = 'GSSApos';                                                                     
    Order         = [1 1 9];                                                                       
    InitialStates = [0; 0; 0; 0; 0; 0; 0];                                                         
    Parameters    = [r; l; q; M; kb; ks; bl];                                                           
    Ts            = 0;                                                                             
    
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts,...
        'Name', 'postion_validation');

    set(nlgr, 'InputName', {'Position_ref'}, 'InputUnit', {'mm'},...
              'OutputName', {'Position_feed'}, 'OutputUnit', {'mm'},...
              'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {0,0,0,0,0,0,0});
    nlgr = setinit(nlgr, 'Fixed', {false false false false false...
        false false});
    nlgr.Parameter(3).Minimum = 0;
    nlgr.Parameter(4).Minimum = 0;
    nlgr.Parameter(5).Minimum = 0;
    disp('We are optimising');

    opt = nlgreyestOptions('Display', 'on'); 
    opt.SearchOption.MaxIter = 10;
    nlgr_opt = nlgreyest(z, nlgr, opt);    
end

getpar(nlgr_opt)

%% Validation
disp('We are compering');
compare(z, nlgr);