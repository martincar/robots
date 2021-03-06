% Define global constants
constants

% get user input
O_d = input('Input end effector location: (eg. [924.87; 149.09; 20.32])\n(Enter 0 for default):\n');
temp_size = size(O_d);
% if dimensions of input are incorrect, try again
while (temp_size(1) ~= 3) || (temp_size(2) ~= 1) 
    if O_d == 0
        O_d = [924.87; 149.09; 20.32];
    else
        O_d = input('Incorrect dimensions in input, try again:\n');
    end
    temp_size = size(O_d);
end

k_d = input('Input approach vector k: (eg. [1;0;0])\n(Enter 0 for default):\n');
temp_size = size(k_d);
while (temp_size(1) ~= 3) || (temp_size(2) ~= 1)
    if k_d == 0
        k_d = [1;0;0];
    else
        k_d = input('Incorrect dimensions in input, try again:\n');
    end
    temp_size = size(k_d);
end

j_d = input('Input sliding vector j: (eg. [0;-1;0])\n(Enter 0 for default):\n');
temp_size = size(j_d);
while (temp_size(1) ~= 3) || (temp_size(2) ~= 1) || (dot(k_d, j_d) ~= 0)
    if j_d == 0
        j_d = [0;-1;0];
    elseif dot(k_d, j_d) ~= 0
        j_d = input('Sliding vector not orthogonal to approach vector, try again:\n');
    else
        j_d = input('Incorrect dimensions in input, try again:\n');
    end
    temp_size = size(j_d);
end

% Calculate and explicitly define all origins and frames using angle_to_matrix_jacobian.m
angle_to_matrix_home;

C_0 = [ 1 0 0;
        0 1 0;
        0 0 1];

O_0 = [0;0;0];

sys_0 = [   C_0        O_0;
            zeros(1,3)   1   ]; 

for i = 1:6
    
    eval(['sys_' num2str(i) ' = sys_' num2str(i-1) '*T' num2str(i) ';']);       % example: sys_1 = sys_0*T1
    
    eval(['C_' num2str(i) ' = sys_' num2str(i) '(1:3, 1:3);']);              % example: C_1 = sys_1(1:3, 1:3)
    
    eval(['O_' num2str(i) ' = sys_' num2str(i) '(1:3, 4);']);                % example: O_1 = sys_1(1:3, 4)

end

% finding theta1
% find home position vector from origin to wrist centre
u = (O_4 - O_0);

% find new wrist centre vector using desired approach vector and desired end effector location
O_4_new = O_d - d6*k_d; 
w = (O_4_new - O_0);

% use kahanP2 around k0 with the two wrist centre vectors to find theta 1
k_0 = C_0(:,3);
theta1 = KahanP2(k_0, u, w); % NOTE: currently only finds one solution for theta 1

% find transformation matrix from C0 to C1
T_11 = k_rot(theta1)*i_rot(-90);
T_12 = [0; 0; 0];
T1 = [T_11 T_12; zeros(1,3) 1];

% find C1, O1
sys_1_new = sys_0*T1;
C_1_new = sys_1_new(1:3, 1:3);
O_1_new = sys_1_new(1:3, 4);

% finding theta3
% use new wrist centre vector as c in kahan P4
% find i1, j1 to find k-j plane projection of new wrist centre vector
% (magnitudes)
i1 = C_1_new(:,1);
j1 = C_1_new(:,2);

% dot new 0_4 vector with i, j
c_i1 = dot( (O_4_new - O_0),  i1 );
c_j1 = dot( (O_4_new - O_0),  j1 );

% use hypotenuse of k-j plane projection components as c
c = sqrt( c_i1^2 + c_j1^2 );

% use origin0 to origin2 length as a
a = a2;

% use hypotenuse of io and jo directions from origin2 to origin4 (wrist centre) as b
b = sqrt(a3^2 + d4^2);

phi3 = atand(a3/d4);
theta3 = KahanP4(a,b,c) - phi3; % NOTE: currently only finds one solution for theta 3

% finding theta2 
% use wrist centre vector as w
u = O_4_new - O_1;
% split vector from origin2 to wrist centre into i1, j1 components
w_j1 = sqrt(a3^2 + d4^2) * sind(theta3 + phi3) * -j1;

o2v_i1 = sqrt(a3^2 + d4^2) * cosd(theta3 + phi3) * i1;
w_i1 = o2v_i1 + a2*i1;
w = w_j1 + w_i1;

k1 = C_1_new(:,3);
theta2 = KahanP2(k1, u, w);

theta1
theta2
theta3

% create function to return i rotation matrix from angle input (in degrees)
function f = i_rot(n)
    f = [1 0 0; 0 cosd(n) -sind(n); 0 sind(n) cosd(n)];
end

function f = k_rot(n)
    f = [cosd(n), -sind(n), 0; sind(n), cosd(n), 0; 0, 0, 1];
end


