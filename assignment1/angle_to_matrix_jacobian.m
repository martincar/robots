% get user input from dialog box
prompt = {'Enter theta 1 (degrees):','Enter theta 2 (degrees):', 'Enter theta 3 (degrees):', 'Enter theta 4 (degrees):', 'Enter theta 5 (degrees):', 'Enter theta 6 (degrees):'};
dlg_title = 'Joint Angle Input';
num_lines = 1;
defaultans = {'0','0','0','0','0','0'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
% convert input strings to numbers
params = str2double(answer);

% for each joint angle
for i=1:6
    % find joint angle associated with iteration number
    theta = params(i);
    
    % create transformation matrix with name Q1 - Q6 based on iteration
    k_rot = [cosd(theta), -sind(theta), 0; sind(theta), cosd(theta), 0; 0, 0, 1];
    if i==1
        T_11 = k_rot*i_rot(-90);
        T_12 = [0; 0; 0];
    elseif i==2
        T_11 = k_rot*i_rot(180);
        T_12 = k_rot*[431.8; 0; 0];
    elseif i==3
        T_11 = [cosd(theta+90), -sind(theta+90), 0; sind(theta+90), cosd(theta+90), 0; 0, 0, 1]*i_rot(90);
        T_12 = [0; 0; -149.09] + [cosd(theta+90), -sind(theta+90), 0; sind(theta+90), cosd(theta+90), 0; 0, 0, 1]*[20.32; 0; 0];
    elseif i==4
        T_11 = k_rot*i_rot(90);
        T_12 = [0; 0; 433.07];
    elseif i==5
        T_11 = k_rot*i_rot(-90);
        T_12 = [0; 0; 0];
    elseif i==6
        T_11 = k_rot;
        T_12 = [0; 0; 60];
    end
    T_21 = [0 0 0];
    T_22 = 1;
    
    Ttop = [T_11 T_12];
    Tbot = [T_21 T_22];
    Ttemp = [Ttop; Tbot];
    eval(['T' num2str(i) '= Ttemp']);
    
    % create overall transformation matrix and plotting vectors
    if i==1
        % if first iteration, create x,y,z variables and overall
        % transformation matrix
        T = Ttemp;
        x = [0 T(1,4)];
        y = [0 T(2,4)];
        z = [0 T(3,4)];
        
        % create matrix for k representations of each transormation
        k0 = [0; 0; 1; 0];
        k1 = T*k0;
        k = [k0 k1];
    else
        % otherwise, multiply Q by current transformation, and append x y z
        % of each link origin 
        T = T*Ttemp;
        x = [x T(1,4)];
        y = [y T(2,4)];
        z = [z T(3,4)];
        
        % append k for each transformation to matrix
        ktemp = T*[0; 0; 1; 0];
        k = [k ktemp];
    end 
end

% create origins vector out of x, y, z vectors
origins = [x;y;z];
% create vector from end effector to each origin
for i=1:6
    if i==1
        v = origins(:,7) - origins(:,1);
    else
        v_i = origins(:,7) - origins(:,i);
        v = [v v_i];
    end
end
% remove last row and column of k vector (no end effector or vector
% representation)
k = k(1:3,1:6);

% take cross product of k and origin to end effector vectors
Jtop = cross(k,v);

% create jacobian
J = [Jtop; k];

% 3d plotting
s = plot3(x,y,z);
s.Marker = '*';
title('Joint Origins');
xlabel('i');
ylabel('j');
zlabel('k');
label_str = {'', 'origin0/1', 'origin2', 'origin3', 'origin4/5', '', 'origin6'};
text(x,y,z+1, label_str);

% output results to command window
disp('Results:');
disp('Input (theta1-6):');
disp(transpose(params));
disp('T:');
disp(T);
disp('J:');
disp(J);

% output k, j and location
disp('O_d:');
disp(T(1:3,4));
disp('k_d');
disp(T(1:3,3));
disp('j_d');
disp(T(1:3,2));

% create function to return i rotation matrix from angle input (in degrees)
function f = i_rot(n)
    f = [1 0 0; 0 cosd(n) -sind(n); 0 sind(n) cosd(n)];
end
