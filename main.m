
yalmip('clear')
clear all

%% Vehicle params
l_axes = 0.3;
l_front = 0.1;
b_width = 0.1;
l_rear = 0.1;


%Initial
z0 = [0;-2.5;0;pi/2;0];
zT = [0;2.5;0;pi/2;0];
obs = [0;0;1];   % circle obstacle [x,y,r]
zmax = [5;5;1;pi;pi/2];
zmin = [-5;-5;0;-pi;-pi/2];
T = 66;                 % Hyper param
dt = 0.1;   
umax = [1;pi/2];
umin = [-1;-pi/2];

x_min = zmin(1);
y_min = zmin(2);
v_min = zmin(3);
theta_min = zmin(4);
phi_min = zmin(5);

x_max = zmax(1);
y_max = zmax(2);
v_max = zmax(3);
theta_max = zmax(4);
phi_max = zmax(5);


%Targets
x_ref = zT(1);
y_ref = zT(2);
v_ref = zT(3);
theta_ref = zT(4);
phi_ref = zT(5);


%% Yalmip setup
nx = 5; % number of states
nu = 2; % number of inputs
u = sdpvar(repmat(nu,1,T),repmat(1,1,T)); % (nu x 1) x T
x = sdpvar(repmat(nx,1,T+1),repmat(1,1,T+1)); % (nx x 1) x (T+1)


objective = 0;
constraints = [];
for k = 1:T 
    % model constraints
    constraints = [constraints,
                    x{k+1}(1)    == x{k}(1) + dt*x{k}(3)*cos(x{k}(4)), % x = x + dt*v*cos(theta)
                    x{k+1}(2)    == x{k}(2) + dt*x{k}(3)*sin(x{k}(4)), % y = y + dt*v*sin(theta)
                    x{k+1}(3)    == x{k}(3) + dt*u{k}(1), % v = v + dt*a
                    x{k+1}(4)    == x{k}(4) + dt*x{k}(3)*tan(x{k}(5))/b_width, % theta = theta + dt*v*tan(phi)/b_width
                    x{k+1}(5)    == x{k}(5) + dt*u{k}(2), % phi = phi + dt*eta  
                    -umax <= u{k} <= umax,
                    zmin <= x{k} <= zmax
                  ];

    % Four corner points
    A_x=x{k}(1)+(l_axes+l_front).*cos(x{k}(4))-b_width.*sin(x{k}(4));
    B_x=x{k}(1)+(l_axes+l_front).*cos(x{k}(4))+b_width.*sin(x{k}(4));
    C_x=x{k}(1)-l_rear.*cos(x{k}(4))+b_width.*sin(x{k}(4));
    D_x=x{k}(1)-l_rear.*cos(x{k}(4))-b_width.*sin(x{k}(4));
    A_y=x{k}(2)+(l_axes+l_front).*sin(x{k}(4))+b_width.*cos(x{k}(4));
    B_y=x{k}(2)+(l_axes+l_front).*sin(x{k}(4))-b_width.*cos(x{k}(4));
    C_y=x{k}(2)-l_rear.*sin(x{k}(4))-b_width.*cos(x{k}(4));
    D_y=x{k}(2)-l_rear.*sin(x{k}(4))+b_width.*cos(x{k}(4));
    
    % Obstacle constraints              
    % (x-x0)^2+(y-y0)^2 >= r^2
    % evauluated at all four corners of car
    x0=obs(1);y0=obs(2);r=obs(3);
    constraints = [constraints, (A_x-x0)^2+(A_y-y0)^2>=r^2,
        (B_x-x0)^2+(B_y-y0)^2>=r^2,
        (C_x-x0)^2+(C_y-y0)^2>=r^2,
        (D_x-x0)^2+(D_y-y0)^2>=r^2];
    
    % Value constraints
    % xmin <= x <= xmax
    % evauluated at all four corners of car
    constraints = [constraints, y_min <= A_y <= y_max,
        y_min <= B_y <= y_max,
        y_min <= C_y <= y_max,
        y_min <= D_y <= y_max];
    constraints = [constraints, x_min <= A_x <= x_max,
        x_min <= B_x <= x_max,
        x_min <= C_x <= x_max,
        x_min <= D_x <= x_max];
end
% Terminal constraints
constraints = [constraints, x{T+1} == [x_ref;y_ref;v_ref;theta_ref;phi_ref]];
% Weighted objective with terminal MSE loss
objective = objective + 100*((x{T+1}(1) - x_ref)^2 + (x{T+1}(2) - y_ref)^2 + ...
                        (x{T+1}(3) - v_ref)^2) + ...
                        (x{T+1}(4) - theta_ref)^2 + (x{T+1}(5) - phi_ref)^2;


parameters_in = {x{1}};
solutions_out = {[u{:}], [x{:}]};
% Using IPOPT solver
controller = optimizer(constraints, objective,sdpsettings('solver','ipopt'),parameters_in,solutions_out);

% Input to controller
oldx = z0;

[solutions,diagnostics] = controller{{oldx}};    
if diagnostics == 1
        error('The problem is infeasible');
end

%use these for animation purposes
x_vals = [];
y_vals = [];
theta_vals = [];

X = solutions{2};
x_vals = X(1,:);
y_vals = X(2,:);
theta_vals = X(4,:);


%% Animation
x_vertices_ref = [x_ref + (l_axes+l_front)*cos(theta_ref) - b_width*sin(theta_ref);
                  x_ref + (l_axes+l_front)*cos(theta_ref) + b_width*sin(theta_ref);
                  x_ref - l_rear*cos(theta_ref) + b_width*sin(theta_ref);
                  x_ref - l_rear*cos(theta_ref) - b_width*sin(theta_ref);
                  x_ref + (l_axes+l_front)*cos(theta_ref) - b_width*sin(theta_ref);
                  ];
y_vertices_ref = [y_ref + (l_axes+l_front)*sin(theta_ref) + b_width*cos(theta_ref);
                  y_ref + (l_axes+l_front)*sin(theta_ref) - b_width*cos(theta_ref);
                  y_ref - l_rear*sin(theta_ref) - b_width*cos(theta_ref);
                  y_ref - l_rear*sin(theta_ref) + b_width*cos(theta_ref);
                  y_ref + (l_axes+l_front)*sin(theta_ref) + b_width*cos(theta_ref);
                  ]; 
figure()              
% Target Location
plot(x_vertices_ref, y_vertices_ref, 'r');
% Circle obstacle
rectangle('Position', [x0 - r, y0 - r, 2*r, 2*r], ...
          'Curvature', [1, 1], 'FaceColor', 'blue');
axis([-5 5 -5 5]);
hold on;

% Planned path
for j = 1:T
    x_vertices = [x_vals(j) + (l_axes+l_front)*cos(theta_vals(j)) - b_width*sin(theta_vals(j));
                  x_vals(j) + (l_axes+l_front)*cos(theta_vals(j)) + b_width*sin(theta_vals(j));
                  x_vals(j) - l_rear*cos(theta_vals(j)) + b_width*sin(theta_vals(j));
                  x_vals(j) - l_rear*cos(theta_vals(j)) - b_width*sin(theta_vals(j));
                  x_vals(j) + (l_axes+l_front)*cos(theta_vals(j)) - b_width*sin(theta_vals(j));
                  ];
    y_vertices = [y_vals(j) + (l_axes+l_front)*sin(theta_vals(j)) + b_width*cos(theta_vals(j));
                  y_vals(j) + (l_axes+l_front)*sin(theta_vals(j)) - b_width*cos(theta_vals(j));
                  y_vals(j) - l_rear*sin(theta_vals(j)) - b_width*cos(theta_vals(j));
                  y_vals(j) - l_rear*sin(theta_vals(j)) + b_width*cos(theta_vals(j));
                  y_vals(j) + (l_axes+l_front)*sin(theta_vals(j)) + b_width*cos(theta_vals(j));
                  ];
           
    plot(x_vertices, y_vertices, 'k');
    pause(dt);
    hold on;
end
