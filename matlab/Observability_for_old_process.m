close all;


a = 500;
fs = 1000;
Ts = 1/fs;
%% state system for constat velocity

A = [1,Ts;0,1]; %
B = [0;0];
C = [1,0];
D = 0;

%% state system for constat accel
A = [1,Ts,Ts^2/2;0,1,Ts;0,0,1];
B = [0;0;0];
C = [1,0,0];
D = 0;

%%
Obvs = [C;C*A;C*A*A]; 
rankO = rank(Obvs);


% Compute observability Gramian
n=3;
N = 100; % Number of steps for Gramian approximation
W_o = zeros(n); % Initialize Gramian matrix (n is the number of states)
for i = 0:N-1
    W_o = W_o + (A')^i * (C') * C * A^i;
end

%G = dlyap(A', C'*C); marginally stable!!!
[V, D] = eig(W_o);

%% for normal system:
% Extract eigenvalues (diagonal entries of D)
lambda1 = D(1,1);
lambda2 = D(2,2);
lambda3 = D(3,3);

% Extract eigenvectors (columns of V)
v1 = V(:,1);
v2 = V(:,2);
v3 = V(:,3);
%%
T = Obvs;
%T = sqrt(D)*V';
A_z = T*A/T;
Obz = [C/T; C/T*A_z; C/T*A_z*A_z];
W_oz = Obz'*Obz;


[Vz, Dz] = eig(W_oz);

% Extract eigenvalues (diagonal entries of D)
lambda1z = Dz(1,1);
lambda2z = Dz(2,2);
lambda3z = Dz(3,3);

% Extract eigenvectors (columns of V)
v1z = Vz(:,1);
v2z = Vz(:,2);
v3z = Vz(:,3);



theta = linspace(0, 2*pi, 50);
phi = linspace(0, pi, 50);

[Theta, Phi] = meshgrid(theta, phi);

% Parametric equations for the ellipsoid
X = cos(Theta) .* sin(Phi);
Y = sin(Theta) .* sin(Phi);
Z = cos(Phi);

% Scale and rotate the ellipsoid by the eigenvalues and eigenvectors
ellipsoid_points = [1/sqrt(lambda1z) * v1, 1/sqrt(lambda2z) * v2, 1/sqrt(lambda3z) * v3] * [X(:)'; Y(:)'; Z(:)'];
X_ellipsoid = reshape(ellipsoid_points(1,:), size(X));
Y_ellipsoid = reshape(ellipsoid_points(2,:), size(Y));
Z_ellipsoid = reshape(ellipsoid_points(3,:), size(Z));

% Plot the ellipsoid
figure;
surf(X_ellipsoid, Y_ellipsoid, Z_ellipsoid, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
hold on;

% Plot axes


% Labels
xlabel('X (Position)');
ylabel('Y (Velocity)');
zlabel('Z (Acceleration)');
title('3D Observability Ellipsoid');
axis equal;
grid on;
hold off;





%%
% Define the angle for the parametric equation of the ellipse
theta = linspace(0, 2*pi, 100);
% Parametric equations for the ellipse
ellipse_points = [cos(theta); sin(theta)];

% Scale the ellipse by the eigenvalues (λ1, λ2) and rotate by the eigenvectors
ellipse_scaled = [1/sqrt(lambda1) * v1, 1/sqrt(lambda2) * v2] * ellipse_points;

% Plot the ellipse
figure;
hold on;

% Plot the ellipse
plot(ellipse_scaled(1,:), ellipse_scaled(2,:), 'b-', 'LineWidth', 2);

% Labels
xlabel('position');
ylabel('velocity');
title('Observability ellipsoid of x0 and x1');

% Hold off to stop adding to the plot
hold off;

ellipse_scaled = [1/sqrt(lambda2) * v2, 1/sqrt(lambda3) * v3] * ellipse_points;

% Plot the ellipse
figure;
hold on;

% Plot the ellipse
plot(ellipse_scaled(1,:), ellipse_scaled(2,:), 'b-', 'LineWidth', 2);

% Labels
xlabel('velocity');
ylabel('acceleration');
title('Observability ellipsoid of x1 and x2');

figure;
hold on;
ellipse_scaled = [1/sqrt(lambda1z) * v1z, 1/sqrt(lambda2z) * v2z] * ellipse_points;

% Labels
xlabel('z0');
ylabel('z1');
title('Observability ellipsoid of z');

% Hold off to stop adding to the plot
hold off;

