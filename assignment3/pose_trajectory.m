% prompt user for desired pose
O_d = input('Input end effector location: (eg. [469.7; 149.1; 435])\n(Enter 0 for default):\n');
temp_size = size(O_d);
% if dimensions of input are incorrect, try again
while (temp_size(1) ~= 3) || (temp_size(2) ~= 1) 
    if O_d == 0
        O_d = [469.7; 149.1; 435];
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

%make a frame based on the inputed j_d and k_d
C_desired = [cross(j_d, k_d), j_d, k_d];

% starting pose and jacobian from forward kinematics
% C_0 = [0 0 1; 0 -1 0; 1 0 0];
% k_0 = [1; 0; 0];
% j_0 = [0; -1; 0];
% O_0 = [469.7; 149.1; 435];
% J_0 = [-149.1 435 -435 0 0 0; 469.7 0 0 60 0 0; 0 -469.7 39.7 0 -60 0; 
%       0 0 0 0 0 1; 0 1 -1 0 1 0; 1 0 0 1 0 0;];

initial_joint_vars = [0,0,90,0,90,0];
[C6, O6, J] = angle_to_matrix_jacobian(initial_joint_vars);

traj = O_d - O6;

distance = norm(traj);
%the time is 1s. Distance = 1/2*t*peak_velocity, because it's a triangle.
peak_velocity = distance/0.5;
traj_normalized_direction = traj/norm(traj);

%Solve for the rotation matrix given your starting and desired frame
Rot_mat = C_desired/C6;

%this gives
axis_of_rotation = [Rot_mat(3,2) - Rot_mat(2,3);Rot_mat(1,3) - Rot_mat(3,1);Rot_mat(2,1) - Rot_mat(1,2)];

%find a random vector that is not parallel with the axis of rotation
v = [1;1;1];
if ( cross(v,axis_of_rotation) == 0)
    v = [1;1;2];
end


 
%Find the angle by running the random vector through the rotation and
%finding the angle between the inital and rotated vector about the axis of
%rotation using KahanP2
theta = KahanP2(axis_of_rotation, v, Rot_mat*v); 

%~~~~~~~~~~~~~~~~tested up to here~~~~~~~~~~~~~~~~~~~~~~

%angular velocity profiles
peak_w = theta/0.5;
peak_w_dir = axis_of_rotation/norm(axis_of_rotation);



% for each 50Hz tick
for i=1:50
    % calculate time in seconds based on tick
    t = i*0.02;
   
    %"peak_velocity/0.5" is the slope of the v(t) graph because you have
    %"peak_velocity" rise over "0.5 seconds" run
    if t <= 0.5
        velocity = (peak_velocity/0.5)*t;
        acceleration = peak_velocity/0.5;
        
        angular_velocity = (peak_w/0.5)*t;
        angular_acceleration = peak_w/0.5;
    else
        velocity = (peak_velocity/0.5)*(1-t);
        acceleration = -peak_velocity/0.5;
        
        angular_velocity = (peak_w/0.5)*(1-t);
        angular_acceleration = -peak_w/0.5;
    end
    
    
end
