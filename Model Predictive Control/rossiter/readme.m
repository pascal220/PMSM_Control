SOFTWARE INSTRUCTIONS for readers of 
Model-based predictive control: a practical approach,  by J.A. Rossiter


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
OVERVIEW

These files are intended as a support to the book authored by J A Rossiter to enable
students to investigate three MPC algorithms. They should also form useful 
templates for algorithm modifications (there are too many possible algorithms to provide
code for them all). For the sake of simplicity, there is very little error catching, so 
it is assumed that the files are called with the correct syntax. 

The files are provided free of charge and as such no guarantee is given as
to their behaviour nor are they intended to be comprehensive. However, USERS are 
invited to contact the author if they discover either bugs or wish to suggest 
useful improvements. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
EDITTING THE FILES AND REQUIREMENTS

The files are simple MATLAB and hence can be editted very easily by the USER to 
formulate the precise scenario or plots desired. Most files do not use any 
MATLAB toolboxes but the exceptions are:
dlqr.m      (for LQMPC algorithm).     Needs replacing by suitable command if removed.
quadprog.m  (for constraint handling). Can be removed to simulate unconstrained case.
dlyap.m     (for LQMPC algorithm)   In ssmpc_costfunction.m (needed for constraint handling)    

WARNING: To avoid excessive screen dumps, the displays from quadprog.m are switched off hence 
the user will not be informed about infeasibility. The edit to undo this is transparent - 
remove argument 'opt' from appropriate line in '*_simulate.m'.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
EXAMPLE FILES (self contained script files):

Example script files are provided to illustrate how to use the programs. These
can be editted in a transparent manner to modify the model, constraints etc.
to use these, simply type the filename in the workspace.

example1_siso       GPC   (Transfer function and T-filter)
example1_mimo       GPC   (Matrix fraction description and T-filter)
example2_siso       LQMPC (State space - siso. Integral action via observer)
example2_mimo       LQMPC (State space - mimo. Integral action via observer)
example3_siso       IMGPC (Independent model, state space. Integral action via offset)
example3_siso       IMGPC (Independent model, state space. Integral action via offset)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SYNTAX ADOPTED IN THE PROGRAMS

(MFD models): See example files if you are unsure about this.
If A(z) = Ao + A1z^{-1}+ ... + An z^{_n},    Ai matrices, then use 
            A = [Ao,A1,...,An]
The code is written assuming a unit delay in the model, as normal in discrete
systems. Therefore process model numerators are written as
           B = [B1,B2,...] that is do not include the first coefficient as zero. 

Toeplitz and hankel matrices:    See caha.m       

STATE SPACE MODELS: It is assumed that all models are strictly proper so D=0.
      uses matrices  A, B, C, D to represent
          x(k+1) = A x(k) + B u(k);    y =C x + D u   (Assumes D=0 however)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DETAILED CODE FOR THREE ALGORITHMS

GPC ALGORITHM (Transfer function models - SISO and MIMO case. 
                     Integral action by incremental model)
1.  mpc_simulate.m     forms constrained closed-loop simulations and nominal control law
                            (GPC - without a T-filter). From figure 1
2.  mpc_simulate_tfilt.m forms constrained closed-loop simulations and nominal control law
                            (GPCT - with a T-filter).   From figure 4
3.  mpc_predmat.m      forms the prediction matrices (without a T-filter)
4.  mpc_predtfilt.m    modifies prediction matrices to include a T-filter
5.  mpc_constraints.m  forms the constraint matrices
6.  mpc_law.m          forms the unconstrained GPC control law from prediction matrices
7.  example1_siso.m    illustrates typical data entry,  simulations and
                            control law calculation for the SISO case (GPC and GPCT)
8.  example1_mimo.m    illustrates typical data entry, a simulations and control law
                            calculation for the MIMO case (GPC and GPCT)
                            
HINTS: To remove constraint handling edit mpc_simulate: remove quadprog and replace
       by line 'unconstrained control law (already in code). Should be obvious.
       To remove constraints from process, set limits large. Can remove from code 
       but this will take longer.
       
       
LQMPC ALGORITHM:    (State space models - SISO and MIMO case. With costing
                      on absolute inputs not increments and integral action via observer.)
1. ssmpc_simulate   Forms closed-loop simulations with constraint handling. From figure 1
2. ssmpc_observor   Forms a default and simplified observer design. Allows for integral
                    action assuming observability.
3. ssmpc_predclp    forms closed-loop prediction matrices assuming a given feedback.
4. ssmpc_costfunction  forms the cost-function dependence on perturbation c, 
                            assumes optimal feedback
5. ssmpc_constraints   forms the constraint matrices, inputs and states (not input rates)
6. example2_siso    illustrates typical data entry,  simulations and control law 
                        calculation for the SISO case                        
7. example2_mimo    illustrates typical data entry,  simulations and control law 
                        calculation for the MIMO case                        

HINTS: To remove constraint handling edit ssmpc_simulate: remove quadprog and set c=0. 
       Should be obvious.  To remove constraints from process, set limits large. Can 
       remove from code but this will take longer.
       If the plots are not converging well, check observabilty first.
       
       
       
IMGPC ALGORITHM:    (Independent state space models - SISO and MIMO case with costing
                    on absolute inputs not increments. Integral action via offset term)
1. imgpc_simulate   Forms closed-loop simulations with constraint handling. From figure 1
2. imgpc_predmat    Forms prediction matrices based on absolute inputs and an offset
                    correction between model and process.
3. imgpc_costfunction  forms the cost-function dependence on perturbation future inputs 
                        and the control law parameters
4. imgpc_constraints   forms the constraint matrices, inputs and input rates.
5. example3_siso    illustrates typical data entry,  simulations and control law 
                        calculation for the SISO state spaces case                        
6. example3_mimo    illustrates typical data entry,  simulations and control law 
                        calculation for the MIMO state space case                        

HINTS: To remove constraint handling edit imgpc_simulate: remove quadprog and use control 
       law (in code); should be obvious.  To remove constraints from process, set limits 
       large. Can remove from code but this will take longer.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

       
Author: J.A. Rossiter  (email: J.A.Rossiter@shef.ac.uk)