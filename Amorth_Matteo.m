clear
close all
clc

%% Setup yalmip

yalmip('clear');
y_settings = sdpsettings('solver', 'mosek', 'verbose', 0);

%% Variables

gravity = 9.81;
drone_mass = 0.5;
drone_inertia = 0.05;

th_force = sdpvar(1,1);
control_torque = sdpvar(1,1);
velocity = sdpvar(1,1);
angular_v = sdpvar(1,1);

f_g_less = th_force - drone_mass * gravity;

%% System definition

a = zeros(4,4);
b = zeros(4,2);
c = zeros(2,4);
d = zeros(2,2);

a(1,2) = 1;
a(3,4) = 1;

b(2,1) = 1/drone_mass;
b(4,2) = 1/drone_inertia;

c(1,1) = 1;
c(2,3) = 1;

[m, ~] = size(a);
[~, n] = size(b);
stability_system = ss(a,b,c,d);


%% Check if the system is controllable/reachable

disp('1. Check if the system is controllable/reachable');

% see ctrb documentation to understand the Output
if ((rank(ctrb(stability_system)) - m) == 0)
    disp('System is controllable/reachable');
else
    disp('System is not controllable/reachable');
end

disp('------------------------------------------------');
%% Controller

alpha = 1;
x = sdpvar(n,m);
w = sdpvar(m,m);
k = 200;
zero_v = 1e-3; 

% eye(x) = identity matrix of x rows and columns

constraints = [ w >= eye(m) * zero_v;
                a * w + b * x + (a * w + b * x)' <= -2 * alpha * w;
                [eye(m) * k, x'; x, eye(n)] >= eye(m + n) * zero_v];
                
solvesdp(constraints, [], y_settings);

w_value = value(w);
x_value = value(x);
p_value = inv(value(w));
k_value = value(x) * inv(value(w));

disp('2. Full-state static feedback control');

disp('The gain matrix K is:');
disp(k_value);

disp('Real part of eigenvalues:');
disp(mat2str(real(eig(stability_system.A + stability_system.B * k_value)')));
disp('------------------------------------------------');

%% perturbations system

disp('3. Space with perturbations');

% new matrices definition'

% New dynamic
a = zeros(6,6);
a(1,2) = 1;
a(2,3) = 1;
a(4,5) = 1;
a(5,6) = 1;

b = zeros(6,2);
b(3,1) = 1/drone_mass;
b(6,2) = 1/drone_inertia;

c = zeros(2,6);
c(1,2) = 1;
c(2,5) = 1;

% d is the same as previously defined
d = zeros(2,2);

e = zeros(6,2);
e(3,1) = 1/drone_mass;
e(6,2) = 1/drone_inertia;

f = zeros(2,2);

% Plant dimensions
[m,~] = size(a); 
[~,n] = size(b); 
[p,~] = size(c);
[~,o] = size(f);

% reset yalmip

yalmip('clear')
y_settings = sdpsettings('solver', 'mosek', 'verbose', 0);

% new var definition
w = sdpvar(m,m);
x = sdpvar(n,m);
rho = sdpvar(1,1);
gamma = sdpvar(1,1);
zero_v = 1e-3; 

gamma_optimization = [];
gain_optimization = [];

for k = 10:10:1400
    constraints = [w >= rho * eye(m);

              rho >= zero_v;

              [k * rho * eye(m),        x';
                  x,          k * rho * eye(n)] >= zero_v * eye(m + n);

              [a * w + b * x + (a * w + b * x)',      e,     (c * w + d * x)';
                                e',                   -gamma * eye(o),         f';
                      c * w + d * x,                      f,        -gamma * eye(p)] <= -zero_v * eye(m + o + p)];
                  
                  
     solvesdp(constraints, gamma, y_settings);
     
     [primal_res, dual_res] = check(constr);
     
     if( all([primal_res; dual_res] > -zero_v) )
         gamma_optimization(end+1) = value(gamma);
         gain_optimization(end+1) = k;
     else
         disp('-----------------------------------------------------------------');
         disp('Problem is infeasible'); 
     end
    
end

% -- Plot optimality curve'
figure(1)
area(gain_optimization, gamma_optimization, 'FaceColor',[0.3010 0.7450 0.9330], 'EdgeColor',[0 0.4470 0.7410]);
xlabel('Bound on |K|');
ylabel('Minimum value of \gamma');
axis([0 gain_optimization(end) 0 gamma_optimization(1)+1/4]);

%% gamma < 0.1, alpha = 1, |K| < 200 (3.2)
% -- Impose alpha = 1, gamma < 0.1 (for example gamma = 0.09) 
alpha = 1;
k_new = 200;
%gamma=0.09;

constraints = [  w >= rho * eye(m);

                rho >= zero_v;

                [k_new*rho*eye(m),         x';
                 x,          k_new * rho * eye(n)] >= zero_v * eye(m + n);

                [a * w + b * x + (a * w + b * x)',      e,     (c * w + d * x)';
                 e',                   -gamma * eye(o),         f';
                 c * w + d * x,                      f,        -gamma * eye(p)] <= -zero_v * eye(m + o + n);

                 a * w + b * x + (a * w + b * x)' <= -2 * alpha * w;

                 gamma <= 0.1 - zero_v];

% -- Solve the problem
solutions = solvesdp(constraints, [], y_settings);

[first_r, second_r] = check(constraints);
disp('-----------------------------------------------------------------');
if( all([first_r; second_r] > -zero_v) )
    disp('The problem (with bounded parameters) is feasible.');
else
    disp('The problem (with bounded parameters) is infeasible.');
end

%% Colleague solution with k = 1200 (3.3)
k_ext = 1200;

constr = [w >= rho*eye(m);

          rho >= zero_v;

          [k_ext * rho * eye(m),         x';
                  x,          k_ext * rho * eye(m)] >= zero_v * eye(n + m);

          [a * w + b * x + (a * w + b * x)',      e,     (c * w + d * x)';
                            e',                   -gamma * eye(),         FF';
                  CC*W_p + DD*X_p,                      FF,        -gamma*eye(m)] <= -fake_zero*eye(n_p+d+m);

           AA*W_p + BB*X_p + (AA*W_p + BB*X_p)' <= -2*alpha*W_p;

           gamma <= 0.1 - fake_zero];

% -- Solve the problem
solvesdp(constr, gamma, opts);

[primal_res, dual_res] = check(constr);

if( all([primal_res; dual_res] > -fake_zero) )
    disp('The problem with colligue parameters is feasible.')
else
    disp('The problem with colligue parameters is infeasible.');
end

% Extract values
W_val = value(W_p);
X_val = value(X_p);
K_val = X_val / W_val;
gamma = value(gamma);
K_norm = norm(K_val);


disp('The gain matrix K is:')
disp(value(K_val));
disp('The norm of K is:')
disp(value(K_norm));
disp('The real part of the closed loop eigenvalues is:')
disp(value(real(eig(AA+BB*K_val)')));
disp('gamma is:')
disp(value(gamma));

%% Simulation of the system from initial conditios (3.4)
t_f = 15;
step_size = 1e-3;
t_sim = 0:step_size:t_f;

x_0 = [0, 1, 0.1, 0, 0.5 0]';

w = [2, 0.2]';

[t_ode_CL, x_ode_CL] = ode45( @(t,x)(AA*x + BB*K_val*x + EE*w), t_sim, x_0 ); 

z_CL = CC*x_ode_CL';

figure(3);
plot(t_sim, z_CL(1,:),'color','b','linewidth',1)
hold on
plot(t_sim, z_CL(2,:),'color','g','linewidth',1)
hold off
title('Feedback-control z and psi errors')
xlabel('Time [s]');
ylabel('Errors');
legend('z-error','psi-error');

