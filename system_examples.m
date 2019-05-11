%% Basic usage of System
hold off; close all; clear; clc;
a = [0,1,0;0,0,1;-8,-14,-7];
b = [0;0;1];
c = [1,1,0];
d = 0;

% initialize a system with state space matrices
sys = System(a,b,c,d);

% print out the A, B, C, and D matrices
sys.display_ss


% get transfer function from system
display('TF');
[num, den] = sys.tf


%Check controllability
% This will return 1 if true or 0 if false
% Also, it will print a statement of whether or not the system is
% controllable

sys.is_controllable


% get Diagonal Controllable Form matrices
display('DCF Matrices:');
[a,b,c,d] = sys.get_dcf

% Get a new System object whose ss variables are 
% in diagonal canonical form:
display('System in DCF:')
dcf_sys = sys.to_dcf;
dcf_sys.display_ss;


% get Controller Canonical Form matrices
display('CCF Matrices:');
[a,b,c,d] = sys.get_ccf

% Get a new System object whose ss varaibles
% are in controller canonical form:
display('System in CCF');
ccf_sys = sys.to_ccf;
ccf_sys.display_ss;

% Initialize a System using a TF
display('Get TF from System');
[num, den] = sys.tf
tf_initialized_sys = System(num, den);
% Make sure that the system is the same as the original system
display('Ensure TF is the same as the TF used to initialize it');
[num2, den2] = tf_initialized_sys.tf
% Checks

% Get eigenvalues of system
display('System eigenvalues');
sys.eigenvalues

% Get Tdcf from the system
display('Tdcf:')
t_dcf = sys.tdcf

% Get controllability matrix from the system
display('Controllability Matrix:');
p = sys.controllability_matrix

% Get inverse Pccf of the system
display('inv(Pccf):');
i_ppcf = sys.ipccf

% Get controllability gramian from the system
display('Controllability Gramian:');
sys_cont_gram = sys.cont_gram

% Get Servocontroller CL system from original OL system
desired_eigenvalues = [-5, -6, -7, -8];
servo_sys = sys.servocontroller(desired_eigenvalues);
display('Servocontrolled system');
servo_sys.display_ss

% Input determination, CL design, and servocontroller design all shown
% below in detail

%check if tf changes if you change a

sys.a(3,3) = -8;
display('Ensure tf matches change to a matrix');
[num, den] = sys.tf


% Get Tccf, Tocf, Q, P, Qocf, Pccf

a = [8,-5,10;0,-1,1;-8,5,-9];
b = [-1;0;1];
c = [1,-2,4];
d = 0;

new_sys = System(a,b,c,d);
display('Get Tccf, Tocf, Q, Qocf, P, Pccf');
tccf = new_sys.tccf
tocf = new_sys.tocf
q = new_sys.q
qocf = new_sys.qocf
p = new_sys.p
pccf = new_sys.pccf

% Get OCF matrices
display('OCF matrices');
[a_ocf, b_ocf, c_ocf, d_ocf] = new_sys.get_ocf

% Make new sys whose matrices are in OCF form, but of the same TF
display('OCF form');
ocf_sys = new_sys.to_ocf;
ocf_sys.display_ss


% Check if is observable
display('Check Observability:')
new_sys.is_observable;

% Get Controllability Gramian
display('Controllability Gramian:')
new_sys.obs_gram

% Get system in ccf
display('CCF Form:');
ccf_sys = new_sys.to_ccf;
ccf_sys.display_ss;

% Get eigenvalues
display('Eigenvalues:');
eigenvalues = new_sys.eigenvalues

% Get system in dcf
display('DCF Form. Notice it matches eigenvalues above.:');
dcf_sys = new_sys.to_dcf;
dcf_sys.display_ss;


% Show how to get dual system and its relationship to controllability and
% observability

a = [1,2;0,1]; b = [1;0]; c = [1,1]; d = 0;

sys = System(a,b,c,d);
sys.is_controllable % returns true
sys.is_observable % returns false
dual_sys = sys.to_dual;
dual_sys.display_ss % shows A,B,C, and D matrices
dual_sys.is_controllable % returns false
dual_sys.is_observable % returns true
%% Use System to find an input to get a required output given time and desired input/outputs

hold off; close all; clear; clc;

a = [0,1,0;0,0,1;-8,-14,-7];
b = [0;0;1];
c = [1,1,0];
d = 0;

% initialize system with a,b,c,d matrices
sys = System(a,b,c,d);

out_init = [0;1;0];
out_final = [1;1;1];
t_init = 0;
t_final = 1;

% Calculate required input signal given the conditions
u = sys.find_input(out_init, out_final, t_final, t_init);
vpa(u,3)
u_eqn = @(t)34.3*exp(t) - 19.4.*exp(2.0.*t) + 1.65.*exp(4.0.*t);
tvals = [0:0.01:1];
uvals = u_eqn(tvals);

figure(1)
plot(tvals, uvals)
title('Input Values')

% Set initial conditions of the system before simulation
% so that the simulation matches the calculated input initial conditions
sys = sys.set_x0(out_init);
% Simulate the states
[yout, t, xout] = sys.simulate(uvals, tvals);

% Plot the states
figure(2)
plot(t, xout(:,1))
hold on
plot(t, xout(:,2))
plot(t, xout(:,3))
legend('x1','x2','x3')
title('State Values')



%% Use System to make a Closed Loop system.


hold off; close all; clear; clc;

% Define OL System
a = [-15,-14,-24;-14,-14,-24;15,15,25];
b = [1;1;-1];
c = [1,1,2];
d = 0;

% Design a state-variable feedback controller 
% Desired poles = -2 += j2, -5

des_poles = [-2 + j*2, -2 - j*2, -5];

% Make open loop system
ol_sys = System(a,b,c,d);


%Show open loop poles
ol_poles = eig(ol_sys.a)

% Make closed loop system
% Note that this doesn't change ol_sys
% It returns a new System which we capture with cl_sys
cl_sys = ol_sys.control(des_poles);

% Display cl system and pole locations
cl_sys.display_ss
cl_poles = eig(cl_sys.a)


% Show k vector values
k = ol_sys.get_control_gain(des_poles)


% PLOT CL VS OL STATES
t = [0:0.01:10];
u = ones(size(t));

% Use `simulate` method to get y,t and c values.
% The internal function being used is `lsim`
[yc,tc,xc] = cl_sys.simulate(u,t);
[yo, to, xo] = ol_sys.simulate(u,t);
figure(1)
subplot(311); plot(tc,xc(:,1), to, xo(:,1), 'r--'); grid();
title('CL v. OL x_1');
ylabel('\it x_1');
legend('CL', 'OL');
subplot(312); plot(tc,xc(:,2), to, xo(:,2), 'r--'); grid();
ylabel('\it x_2');
title('CL v. OL x_2');
legend('CL', 'OL');
subplot(313); plot(tc,xc(:,3), to, xo(:,3), 'r--'); grid();
title('CL v. OL x_3');
ylabel('\it x_3');
xlabel('\it time (sec)');
legend('CL', 'OL');

% PLOT CL VS OL OUTPUT
figure(2)
plot(tc, yc)
hold on
plot(to, yo)
title('CL v. OL OUT');
legend('CL', 'OL')



% Modify the controller for steady-state tracking, put fourth pole @ s =
% -10

des_poles = [des_poles -10];

% Create servocontroller system for SS tracking
% Again notice that ol_sys is not changed, but it returns
% a new object which we capture with `tracked_sys`
tracked_sys = ol_sys.servocontroller(des_poles);

% Display CL system poles
poles = eig(tracked_sys.a)

%Display system and k matrix:
tracked_sys.display_ss

% This step is just to get k so I can display it, it is not necessary to 
% compute the servocontroller system. That is done via the line:
%tracked_sys = ol_sys.servocontroller(des_poles);
[k,an,bn] = ol_sys.servocontroller_gain(des_poles)
k

%Simulate the tracked system
[yt, tt, xt] = tracked_sys.simulate(u,t);


% Part D
% Plot SS Tracked System

figure(3)
subplot(311); plot(tt,xt(:,1)); grid();
title('x_1');
ylabel('\it x_1');

subplot(312); plot(tt,xt(:,2)); grid();
ylabel('\it x_2');
title('CL v. OL x_2');

subplot(313); plot(tt,xt(:,3)); grid();
title('x_3');
ylabel('\it x_3');
xlabel('\it time (sec)');


% PLOT CL VS OL OUTPUT
figure(4)
plot(tt, yt)
title('OUTPUT');

% To see ss tracking match with ol sys instead of input value, 
% you can change last b matrix value to 0.1 (OL yss value)

ol_track_sys = tracked_sys;
ol_track_sys.b = [0;0;0;0.1];

[ytt, ttt, xtt] = ol_track_sys.simulate(u,t);

% Ensure poles are the same
poles = eig(ol_track_sys.a)

% Plot OL vs CL
figure(5)
subplot(311); plot(ttt,xtt(:,1), to, xo(:,1), 'r--'); grid();
title('CL v. OL x_1');
ylabel('\it x_1');
legend('CL', 'OL');
subplot(312); plot(ttt,xtt(:,2), to, xo(:,2), 'r--'); grid();
ylabel('\it x_2');
title('CL v. OL x_2');
legend('CL', 'OL');
subplot(313); plot(ttt,xtt(:,3), to, xo(:,3), 'r--'); grid();
title('CL v. OL x_3');
ylabel('\it x_3');
xlabel('\it time (sec)');
legend('CL', 'OL');

% PLOT CL VS OL OUTPUT
figure(6)
plot(ttt, ytt)
hold on
plot(to, yo)
title('CL v. OL OUT');
legend('CL', 'OL')