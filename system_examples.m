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

% Get tocf 

a = [1,1,0;0,2,1;1,0,0]; b = [1;2;1]; c= [1,2,0]; d = 0;
sys = System(a,b,c,d);
tocf = sys.tocf
% Check above
[ao,bo,co,do] = sys.get_ocf
[a,b,c,d] = sys.get_ssxform(tocf)
a_checks = ao == a
b_checks = bo == b
c_checks = co == c
d_checks = do == d

% Perform pole zero cancellation
zeros = [-3, -1]
poles = [-2, -3, -4]
num = poly(zeros);
den = poly(poles);
tfsys = System(num, den);
tfsys.is_reducable
reduced_sys = tfsys.reduce;
new_poles = reduced_sys.poles
new_zeros = reduced_sys.zeros
[nnum, nden] = reduced_sys.tf

% Use sym with controllability and observability matrices
syms c1 c2 c3

a = [1,0,0;0,0,0;0,0,-1];
c = [c1, c2, c3];
sys = System(a,[0;0;1], c, 0);
q = sys.q
eq = det(q) == 0

syms k1 k2

a = [-6,1;-5,0];
b = [1;k1];
c = [1, k2];
d = 0;


% k2 = 0;
sys = System(a,b,c,d);
p = sys.p
eq = det(p) == 0
solve(eq, k1)
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

%% Use System to evaluate controllability and observability of symbolic State Space Equations:
%% 4 problems below
%% Problem 1
hold off; close all; clear; clc;


syms c1 c2 c3

a = [1,0,0;0,0,0;0,0,-1];
c = [c1, c2, c3];

% Part A
% From the A matrix we can see that the states are
% decoupled, so any state decoupled from the output 
% (ie any element of c is zero) would cause the system
% to become unobservable.

display('Part A')
display('========= Intuition =======');
display('Since the states are decoupled, any element of the C matrix being 0 would cause unobservability');
display('===========================');
q = [c; c*a; c*a^2]

eqn = det(q) == 0;
unobservability_eqn = eqn

display('For observable solution, make det = 0');
unobservable = subs(eqn, [c1,c2,c3], [1,0,3])

display('For observable solution, make det != 0');
observable = subs(eqn, [c1,c2,c3], [1,2,3])
display('So, any value of c1, c2, or c3 equaling zero will cause unobservability');
% Check
display('=============== Check ===================')
c1 = [1,0,3];
if rank(obsv(a,c1)) == 2; disp('Success'); else disp('Fail'); end;
c1 = [1,2,3];
if rank(obsv(a,c1)) == 3; disp('Success'); else disp('Fail'); end;
c1 = [0,199,231];
if rank(obsv(a,c1)) == 2; disp('Success'); else disp('Fail'); end;
c1 = [827373,199,0];
if rank(obsv(a,c1)) == 2; disp('Success'); else disp('Fail'); end;
display('Check SAT');
display('-----------------------------------------------------------');
% Part B
display('Part B');
display('Any values of c1, c2, and c3 when none of them are zero cause observability.');
display('Above shows this with c1,c2,c3 = 1,2,3');
display('Another Exmaple: c1 = 2, c2 = 4, c3 = 6:');
c = [2,4,6];
q = obsv(a,c);
if rank(q) == 3; disp('Success - Obsevable'); else; disp('Fail - Not Observable'); end;
display('Proving that c1,c2,c3 = 2,4,6 results in an observable system');

%% Problem 2

hold off; close all; clear; clc;

syms k1 k2

a = [-6,1;-5,0];
b = [1;k1];
c = [1, k2];
d = 0;
sys = System(a,b,c,d);
% Part A
% Is the system controllable for all vaues of k1?
% k2 = 0;
display('Part A');
display('Is the system controllable for all values of k1? (k2 = 0)');
p = [b, a*b]
eq = det(p) == 0
uncontrollable_values = solve(eq, k1)
display('So, the system is uncontrollable for k = 1,5');
% Check:
p = ctrb(a, [1;1]);
if rank(p) < 2; display('Check 1 success'); else display('Check 1 failure'); end
p = ctrb(a, [1;5]);
if rank(p) < 2; display('Check 2 success'); else display('Check 2 failure'); end
p = ctrb(a, [1;2]);
if rank(p) < 2; display('Check 3 failure'); else display('Check 3 success'); end
display('----------------------------------------------------');
% Part B
% Is the system observable for all values of k2?
% k1 = 0
display('Part B');
display('Is the system observable for all vaues of k2? (k1 = 0)');

q = sys.q
eq = det(q) == 0
unobservable_values = solve(eq, k2)
display('So, the system is unobservable for k = -1,-0.2');
% Check:
p = obsv(a, [1, -1]);
if rank(p) < 2; display('Check 1 success'); else display('Check 1 failure'); end
p = obsv(a, [1,-0.2]);
if rank(p) < 2; display('Check 2 success'); else display('Check 2 failure'); end
p = obsv(a, [1,-0.5]);
if rank(p) < 2; display('Check 3 failure'); else display('Check 3 success'); end
display('------------------------------------------------');
% Part C
% For the values of K2 that make the system uncontrollable
% what is the transfer function of the system?
display('Part C');
display('For the values of K1 that made the system uncontrollable, what is the TF of the system?');
bc1 = double(subs(b, k1, 1));
bc2 = double(subs(b, k1, 5));
cc = double(subs(c, k2, 0));

sys_c1 = System(a,bc1,cc,d);
sys_c2 = System(a,bc2,cc,d);
[num1, den1] = sys_c1.tf;
tf_k1_1 = tf(num1, den1)
[num2, den2] = sys_c2.tf;
tf_k1_5 = tf(num2, den2)
% check
disp('Check');
if ~sys_c1.is_controllable && ~sys_c2.is_controllable; display('Check SAT'); else display('Check FAILED'); end;

display('----------------------------------------------');
% Part D 
% For the values of K2 that make the system unobervable
% what is the transfer funciton of the system?
display('Part D');
display('For the values of K2 that made the system unobservable, what is the TF of the system?');
bd = double(subs(b, k1, 0));
cd1 = double(subs(c, k2, -1));
cd2 = double(subs(c, k2, -0.2));

sys_d1 = System(a,bd, cd1, d);
sys_d2 = System(a,bd, cd2, d);
[num1, den1] = sys_d1.tf;
[num2, den2] = sys_d2.tf;

tf_k2_neg1 = tf(num1, den1)
tf_k2_negpoint2 = tf(num2, den2)
% Check
display('Check:');
if ~sys_d1.is_observable && ~sys_d2.is_observable; display('Check SAT'); else display('Check FAILED'); end

display('==============================Results================================');
display('So, the TFs for the unobservable and uncontrollable values of K1 and K2 ended up being the same');


%% Poblem 3
hold off; close all; clear; clc;

syms I m l ddphi phi ddx dx g M b u

eqn1 = (I + m*l^2) * ddphi - m*g*l*phi == m*l *ddx;
eqn2 = (M+m) * ddx + b * dx - m*l*ddphi == u;

ddphi_sol = solve(eqn1, ddphi);
ddx_sol = solve(eqn2, ddx);
ddx_sol_solo = subs(ddx_sol, ddphi, ddphi_sol);
ddphi_sol_solo = subs(ddphi_sol, ddx, ddx_sol);

ddx_final = simplify(solve(ddx == ddx_sol_solo, ddx))
ddphi_final = simplify(solve(ddphi == ddphi_sol_solo, ddphi))

display('ddx state eqns:');
ddx_dx_component = subs(ddx_final, [u, phi, dx], [0,0,1])
ddx_phi_component = subs(ddx_final, [u, dx, phi], [0,0,1])
ddx_u_component = subs(ddx_final, [phi, dx, u], [0,0,1])

display('ddphi state eqns:');
ddphi_dx_component = subs(ddphi_final, [u, phi, dx], [0,0,1])
ddphi_phi_component = subs(ddphi_final, [u, dx, phi], [0,0,1])
ddphi_u_component = subs(ddphi_final, [dx, phi, u], [0,0,1])

% Make state equations: states [x1,x2,x3,x4]' = [x,dx,phi,dphi]'

asym = [0,1,0,0;...
    0,ddx_dx_component,ddx_phi_component, 0;...
    0,0,0,1;...
    0, ddphi_dx_component, ddphi_phi_component, 0]


bsym = [0; ddx_u_component; 0; ddphi_u_component]

% For two outputs
display('For angle and cart position outputs');
c = [1,0,0,0; 0,0,1,0]
d = [0;0]

% For the pendulum angle ouptut only:
display('For pendulum angle only');
c_angle = [0,0,1,0]
d_angle = 0

% For the cart position output only:
display('For cart position only');
c_cart = [1,0,0,0]
d_cart = 0

% Substitute in values
Mval = 0.5;
mval = 0.2;
bval = 0.1;
lval = 0.3;
gval = 9.8;
Ival = 0.006;

a = double(subs(asym, [M, m, b, l, g, I], [Mval, mval, bval, lval, gval, Ival]))
b = double(subs(bsym, [M, m, b, l, g, I], [Mval, mval, bval, lval, gval, Ival]))

symSys = System(asym, bsym, c, d);
symTF = simplify(symSys.ss2tf);

sys_2outs = System(a,b,c,d);
sys_angleout = System(a,b,c_angle, d_angle);
sys_cartout = System(a,b,c_cart, d_cart);

% Check observability for just being able to measure cart position
display('Check observability and controllability for just cart pos. output');
sys_cartout.is_controllable;
sys_cartout.is_observable;
zeros = sys_cartout.zeros
poles = sys_cartout.poles
display('---------------------------------------');

% Check observability for just being able to measure pendulum angle
display('Check observability and controllability for just angle output');
sys_angleout.is_controllable;
sys_angleout.is_observable;
[num, den] = sys_angleout.tf;
original_tf = tf(num, den)
zeros = sys_angleout.zeros
poles = sys_angleout.poles
% Not observable, so find minimum realization
display('Reduce System');
if sys_angleout.is_reducable; display('System is Reducable'); end
reduced_sys = sys_angleout.reduce;
[num, den] = reduced_sys.tf;
reduced_tf = tf(num, den)
reduced_sys.display_ss
zeros = reduced_sys.zeros
poles = reduced_sys.poles
display('---------------------------------------');


% Check observability for being able to measure both outputs
display('Check Controllability and Observability for an output of both cart position and pendulum angle');
sys_2outs.is_controllable;
sys_2outs.is_observable;
[num, den] = sys_2outs.tf;
zeros = roots(num(1,:))
zeros2 = roots(num(2,:))
poles = sys_2outs.poles

%% Problem 4
hold off; close all; clear; clc;
syms beta gamma alpha

% Part A

% System 1 Controllability Mtarix = B = (alpha - beta).
% Therefore system 1 is contollable if alpha != beta
% System 1 Observability Matrix = 1, therefore it is observable
display('System 1')
sys1 = System([-beta], [alpha-beta], [1], 0);
p = sys1.p
q = sys1.q
controllability_alpha = solve(det(p) == 0, alpha)
det_q = det(q) % Is always 1 


% System 2 Controllability Matrix = B = 1, therefore it is controllable
% System 2 Observability matrix = C = 1, therefore it is observable

display('System 2')
sys2 = System([-alpha], [1], [1], 0);
p = sys2.p % det(p) is always 1 
q = sys2.q % det(q) is always 1
display('------------------------------------------------');


% Part B
display('Part B - Systems in series, CCF');
a = [0,1;-beta*gamma, -(beta+gamma)]; 
b = [0;1];
c = [alpha,1];

sym_sys = System(a,b,c,0);
q = sym_sys.q
observability_eqn = 0 == det(q)
alpha_unobservable_vals = solve(observability_eqn, alpha)
beta_unobservable_vals = solve(observability_eqn, beta)
gamma_unobservable_vals = solve(observability_eqn, gamma)
display('Will be unobservable if alpha = beta OR alpha = gamma');
% Check
display('Test above conclusion');
subs(observability_eqn, [alpha, gamma], [10,10])
subs(observability_eqn, [alpha, beta] , [2.44, 2.44])
display('Test SAT');

p = sym_sys.p
controllability_eqn = 0 == det(p)
display('Always controllable')
display('Checks with being in Controller Canonical Form');
display('--------------------------------------------------');


% Part C
display('Part C: Both Systems in series; OCF');

a = [0, -beta*gamma; 1, -(beta+gamma)];
b = [alpha ; 1];
c = [0,1];
d = 0;

sys_ocf = System(a,b,c,d);

p = sys_ocf.p
q = sys_ocf.q

controllability_eqn = 0 == det(p)
observability_eqn = 0 == det(q)

uncontrollable_alphas = solve(controllability_eqn, alpha)
uncontrollable_betas = solve(controllability_eqn, beta)
uncontrollable_gammas = solve(controllability_eqn, gamma)
display('So, the system is uncontrollable when alpha = beta OR alpha = gamma')
display('Check:')
a_t = double(subs(a, [beta, gamma], [5, 1]));
b_t1 = double(subs(b, alpha, 5));
b_t2 = double(subs(b, alpha, 1));
b_t3 = double(subs(b, alpha, 3));
systest = System(a_t, b_t1, c,d);
systest.is_controllable;
systest.b = b_t2;
systest.is_controllable;
systest.b = b_t3;
systest.is_controllable;
display('Tests SAT');

display('Observability does not depend on any variables, so the system is always observable');
display('This checks with the system being in OCF as well as the results of the system in CCF');
display('---------------------------------------------------------------------------------------------');

% Part D

display('Find minimum realization for situaitons when system was uncontrollable/unobservable');
display('This occurred when alpha = beta OR alpha = gamma');

b_gsubs = subs(b, alpha, gamma);
b_bsubs = subs(b, alpha, beta);
a_bsubs = subs(a, beta, alpha);
sys_a2g = System(a,b_gsubs,c,d);
sys_a2b = System(a, b_bsubs, c,d);
sys_b2a = System(a_bsubs, b, c,d);
tfsys1 = sys_a2b.ss2tf
tfsys2 = sys_a2g.ss2tf
tfsys3 = sys_b2a.ss2tf

tfsys1_simplified = simplify(sys_a2b.ss2tf)
tfsys2_simplified = simplify(sys_a2g.ss2tf)
tfsys3_simplified = simplify(sys_b2a.ss2tf) 
display('Note that tfsys3 is the same as tfsys1 because the same mathematical substitution was made');
display('The simplified version is 1 / (s + lambda), where lambda = the variable not equal to the other two');
% Check
display('=============Check===================')
display('let alpha = beta = 2.34, gamma = 3')
display('This aligns with sys_a2b which yielded a reduced form of 1/(s+gamma)');
aval = 2.34;
bval = 2.34;
gval = 3;


at = double(subs(a, [beta, gamma], [bval, gval]));
bt = double(subs(b, alpha, aval));
syst = System(at,bt,c,d);
syst.is_controllable;
red_sys = syst.reduce;
red_tf = red_sys.to_tf
display('Got expected value of 1/(s+gamma) = 1/(s+3)');
display('Check SAT');
