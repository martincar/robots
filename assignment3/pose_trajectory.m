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

% starting pose and jacobian from forward kinematics
% C_0 = [0 0 1; 0 -1 0; 1 0 0];
k_0 = [1; 0; 0];
j_0 = [0; -1; 0];
O_0 = [469.7; 149.1; 435];
% J_0 = [-149.1 435 -435 0 0 0; 469.7 0 0 60 0 0; 0 -469.7 39.7 0 -60 0; 
%       0 0 0 0 0 1; 0 1 -1 0 1 0; 1 0 0 1 0 0;];

traj = O_d - O_0;
  
% for each 50Hz tick
for i=1:50
    % calculate time in seconds based on tick
    t = i*0.02;
   
    if t <= 0.5
        % find displacement profile
        % find velocity profile
        % find acceleration profile
    else
        
    end
    
    
end