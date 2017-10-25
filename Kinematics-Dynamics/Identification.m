%% Known motor parameters and gearbox
Jm = 1.1472;      % Inertia of AC motor - gm^2
p = 12;         % Number of poles of the ac motor
R = 0.378;      % Stator Resistance - Ohm
Tr = 27.5;      % Transmision ratio of the gearbox 
lambda = 0.11;  % Flux of motor
Lq = 3.334e-3;  % Inductance in stator
Ld = 3.427e-3;  % Inductance in stator
Jg = 0.07;      % Inertia of gearbox - gm^2
Jc = 23.56;     % Inertia of crank - gm^2
Ke = 1.131;     % EMF constant - V/(rad/s)
Kt = 1.395;     % Torque constant - Nm/Arms
bm = 0.01;      % Viscous damping(friction) coefficient - Nm
bg = 0.01;      % Internal damping of gearbox 


%% Let's creat the non-linear model
prompt = 'Which mode would you like?(curent=1/velocity=2/PMSM=3)';
i = input(prompt);

prompt = 'White noise or sine?(White=1/Sin=2)';
k = input(prompt);

if i == 1 % Current ************************************************************************************
    [y,u,t_sample] = input_output(i,k);
    z = iddata(y,u,t_sample,'Name','current_data');
    
    z.InputName = {'Voltage_d','Voltage_q','Velocity'};
    z.InputUnit =  {'V','V','rad/s'};
    z.OutputName = {'Current_q'};
    z.OutputUnit = {'A'};
    z.Tstart = 0;
    z.TimeUnit = 's';
    
%     figure('Name', [z.Name ': Voltage-d input -> Current output']);
%     plot(z(:, 1, 1)); 
%     figure('Name', [z.Name ': Voltage-q input -> Current output']);
%     plot(z(:, 1, 2)); 
%     figure('Name', [z.Name ': Velocity input -> Current output']);
%     plot(z(:, 1, 3));

    FileName      = 'equations_current';                                    % File describing the model structure.
    Order         = [1 3 2];                                                % Model orders [ny nu nx].
    Parameters    = [Ld; Lq; R; p; Ke];                                         % Initial parameters.
    InitialStates = [0; 0];                                                 % Initial initial states.
    Ts            =  0;                                                     % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'current_model');

    set(nlgr, 'InputName', {'Voltage_d','Voltage_q','Velocity'}, 'InputUnit', {'V','V','rad/s'},...
              'OutputName', 'Current_q', 'OutputUnit', 'A',...
              'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {0,0,0,1,0});

elseif i == 2 % Velocity *******************************************************************************
    [y,u,t_sample] = input_output(i,k);
    z = iddata(y,u,t_sample,'Name','velocity_data');
    
    z.InputName = {'Current_q'};
    z.InputUnit =  {'A'};
    z.OutputName = {'Velocity'};
    z.OutputUnit = {'rad/s'};
    z.Tstart = 0;
    z.TimeUnit = 's';
    
%     figure('Name', [z.Name ': Voltage-d input -> Current output']);
%     plot(z(:, 1, 1)); 
%     figure('Name', [z.Name ': Voltage-q input -> Current output']);
%     plot(z(:, 1, 2)); 

    FileName      = 'equations_velocity';                                   % File describing the model structure.
    Order         = [1 1 2];                                                % Model orders [ny nu nx].
    Parameters    = [Tr; Kt; Jm; Jg; Jc; bm; bg];                           % Initial parameters.
    InitialStates = [0.985; 0];                                             % Initial initial states.
    Ts            =  0;                                                     % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'velocity_model');

    set(nlgr, 'InputName', {'Current_q'}, 'InputUnit', {'A'},...
              'OutputName', {'Velocity'}, 'OutputUnit', {'rad/s'},...
              'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {1,0,1,0,0,1,1});
elseif i == 3 % PMSM ***********************************************************************************
    [y,u,t_sample] = input_output(i,k);
    z = iddata(y,u,t_sample,'Name','velocity_data');
    
    z.InputName = {'Voltage_d','Voltage_q'};
    z.InputUnit =  {'V','V'};
    z.OutputName = {'Velocity'};
    z.OutputUnit = {'rad/s'};
    z.Tstart = 0;
    z.TimeUnit = 's';
   
%     figure('Name', [z.Name ': Voltage-d input -> Current output']);
%     plot(z(:, 1, 1)); 
%     figure('Name', [z.Name ': Voltage-q input -> Current output']);
%     plot(z(:, 1, 2)); 

    FileName      = 'dynamic_equations';                                                        % File describing the model structure.
    Order         = [1 2 4];                                                                    % Model orders [ny nu nx].
    Parameters    = [Ld; Lq; R; p; Tr; Jm; Jg; Jc; bm; bg; Kt; Ke];                             % Initial parameters.
    InitialStates = [0; 0; 0.985; 0];                                                           % Initial initial states.
    Ts            =  0;                                                                         % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'velocity_model');

    set(nlgr, 'InputName', {'Voltage_d','Voltage_q'}, 'InputUnit', {'V','V'},...
              'OutputName', {'Velocity'}, 'OutputUnit', {'rad/s'},...
              'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {0,0,0,1,1,1,0,0,0,0,0,0});
elseif i == 4 % The whole system with PI's
    [y,u,t_sample] = input_output(i,k);
    z = iddata(y,u,t_sample,'Name','system_data');
    
    z.InputName = {'Voltage_d','Voltage_q'};
    z.InputUnit =  {'V','V'};
    z.OutputName = {'Velocity'};
    z.OutputUnit = {'rad/s'};
    z.Tstart = 0;
    z.TimeUnit = 's';
   
%     figure('Name', [z.Name ': Voltage-d input -> Current output']);
%     plot(z(:, 1, 1)); 
%     figure('Name', [z.Name ': Voltage-q input -> Current output']);
%     plot(z(:, 1, 2)); 

    FileName      = 'dynamic_equations';                                                        % File describing the model structure.
    Order         = [1 2 4];                                                                    % Model orders [ny nu nx].
    Parameters    = [Ld; Lq; R; p; Tr; Jm; Jg; Jc; bm; bg; Kt; Ke];                             % Initial parameters.
    InitialStates = [0; 0; 0.985; 0];                                                           % Initial initial states.
    Ts            =  0;                                                                         % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'velocity_model');

    set(nlgr, 'InputName', {'Voltage_d','Voltage_q'}, 'InputUnit', {'V','V'},...
              'OutputName', {'Velocity'}, 'OutputUnit', {'rad/s'},...
              'TimeUnit', 's');

    nlgr = setpar(nlgr, 'Fixed', {0,0,0,1,1,1,0,0,0,0,0,0});
end
%% Estimating parameters of non-linear current model
opt = nlgreyestOptions('Display', 'on');
opt.SearchOption.MaxIter = 120;
nlgr_opt = nlgreyest(z, nlgr, opt);
 
compare(z, nlgr_opt);