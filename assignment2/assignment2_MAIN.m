% Define global constants
constants

% get user input
O_d = input('Input end effector location: (eg. [925; 149.1; 20.3])\n(Enter 0 for default):\n');
temp_size = size(O_d);
% if dimensions of input are incorrect, try again
while (temp_size(1) ~= 3) || (temp_size(2) ~= 1) 
    if O_d == 0
        O_d = [925; 149.1; 20.3];
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
while (temp_size(1) ~= 3) || (temp_size(2) ~= 1) || (dot(k_d, j_d) >= 1e-4)
    if j_d == 0
        j_d = [0;-1;0];
    elseif dot(k_d, j_d) >= 1e-4
        j_d = input('Sliding vector not orthogonal to approach vector, try again:\n');
    else
        j_d = input('Incorrect dimensions in input, try again:\n');
    end
    temp_size = size(j_d);
end

% Calculate and explicitly define all home origins and frames using angle_to_matrix_home script
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

% finding theta3
% % find new wrist centre vector using desired approach vector and desired end effector location
O_4_new = O_d - d6*k_d; 
w = (O_4_new - O_0);
arm_length_desired = norm(w);

%c_prime is the desired arm length projected onto home position arm plane. 
c_prime = sqrt(arm_length_desired^2 - d3^2);

b = sqrt(a3^2 + d4^2);
phi = atand(a3/d4); %phi is the angle resulting from the vertical offset in the arm 

theta3 = KahanP4(a2,b,c_prime) - phi;

% if theta3 is really small, set to 0 to avoid -0 values
if theta3 < 1e-10 && theta3 > -1e-10
   theta3 = 0;
end

% find theta 1 and theta 2
u = O_4_new - O_0;

% first theta 3 solution
% find transformation matrix of theta3 rotation with theta 1 and 2 being 0
T_11 = k_rot(theta3 + 90)*i_rot(90);
T_12 = [0; 0; -149.09] + k_rot(theta3 + 90)*[20.32; 0; 0];
T3_new = [T_11 T_12; zeros(1,3) 1;];

% find new system assuming only theta3 movement
sys_4_new = T1*T2*T3_new*T4*sys_0;
% extract location from system
v = sys_4_new(1:3, 4);

% set rotation matrices of j0 and k0
s = [0;0;1];
t = [0;1;0];

% do kahan P3 to find 2 sets of theta1 and theta 2 (left and right shoulder)
[theta1_left, theta2_left, theta1_right, theta2_right] = KahanP3(s,t,u,v);

% reverse theta1 direction because kahanP3 gives the opposite theta1 rotation
theta1_left = -1 * theta1_left;
theta1_right = -1 * theta1_right;

% round really small values to 0
if theta1_left < 1e-10 && theta1_left > -1e-10
   theta1_left = 0;
end

if theta2_left < 1e-10 && theta2_left > -1e-10
   theta2_left = 0;
end

if theta1_right < 1e-10 && theta1_right > -1e-10
   theta1_right = 0;
end

if theta2_right < 1e-10 && theta2_right > -1e-10
   theta2_right = 0;
end

arm_soln1 = [theta1_left; theta2_left; theta3];
arm_soln2 = [theta1_right; theta2_right; theta3];

% second theta 3 solution
% find transformation matrix of -theta3 rotation
T_11 = k_rot(-1 * theta3 + 90)*i_rot(90);
T_12 = [0; 0; -149.09] + k_rot(-1 * theta3 + 90)*[20.32; 0; 0];
T3_new = [T_11 T_12; zeros(1,3) 1;];

% find new system assuming only -theta3 movement
sys_4_new = T1*T2*T3_new*T4*sys_0;
% extract location from system
v = sys_4_new(1:3, 4);

% set rotation matrices of j0 and k0
s = [0;0;1];
t = [0;1;0];

% do kahan P3 to find 2 sets of theta1 and theta 2 (left and right shoulder)
[theta1_left, theta2_left, theta1_right, theta2_right] = KahanP3(s,t,u,v);

% reverse theta1 direction because kahanP3 gives the opposite theta1 rotation
theta1_left = -1 * theta1_left;
theta1_right = -1 * theta1_right;

% round really small values to 0
if theta1_left < 1e-10 && theta1_left > -1e-10
   theta1_left = 0;
end

if theta2_left < 1e-10 && theta2_left > -1e-10
   theta2_left = 0;
end

if theta1_right < 1e-10 && theta1_right > -1e-10
   theta1_right = 0;
end

if theta2_right < 1e-10 && theta2_right > -1e-10
   theta2_right = 0;
end

arm_soln3 = [theta1_left; theta2_left; -1 * theta3];
arm_soln4 = [theta1_right; theta2_right; -1 * theta3];

% make matrix of all solutions
arm_solns = [arm_soln1 arm_soln2 arm_soln3 arm_soln4];

% find number of solutions (number of columns in array)
arm_soln_size = size(arm_solns);
num_solns = arm_soln_size(2);

% for each arm solution
for i=1:num_solns
    % extract correct arm solution from arm solution matrix
    arm_soln = arm_solns(1:3, i);
    % create frame 3 for each arm solution
    c3 = create_c3(arm_soln);
    
    % find wrist solutions (input frame 3 and desired k and j
    wrist_solns_temp = wrist_kinematics(c3, k_d, j_d);
    
    % append the same arm solution on top of wrist solutions
    solutions_temp = [arm_soln arm_soln arm_soln arm_soln;
                      wrist_solns_temp;                   ];
    
    eval(['soln' num2str(i) '= solutions_temp;']);
end

% append all arm + wrist solutions together
all_solns = [soln1 soln2 soln3 soln4];

% remove all duplicate solutions
all_solns = transpose(unique(all_solns', 'rows'));

% remove any solutions with angles outside of joint limits
% find number of solutions (columns)
solns_size = size(all_solns);
num_solns = solns_size(2);

% create array of zeroes to append to
solns = zeros(6,1);

% for each solution
for i=1:num_solns
   % create array of boundaries for joint limits
   bot_bounds = [-160; -225; -135; -110; -100; -266];
   top_bounds = [160; 45; 135; 170; 100; 266];
   
   % set append flag to true by default
   append = 1;
   % for each bound
   for j=1:6
       % if an angle is outside of the bounds, set append flag to false
      if all_solns(j,i) < bot_bounds(j) || all_solns(j,i) > top_bounds(j)
         append = 0; 
      end
   end
   
   % if append flag true (no out of bounds angles)
   if append == 1
       % append current solution to final solutions matrix
       solns = [solns all_solns(:,i)];
   end
end

% remove zero column from solutions matrix
solns(:,1) = [];

% find amount of solns inside joint limits
solns_size = size(solns);
num_final_solns = solns_size(2);

disp('Results: each column is a unique solution within joint limits (theta1-6)')
disp(solns)
fprintf('%d solutions found.\n',num_final_solns)

% forward kinematics test
% for i=1:num_final_solns
%    forward_kin_test(solns(:,i)); 
% end

