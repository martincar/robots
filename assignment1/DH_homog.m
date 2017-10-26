% get user input from dialog box
prompt = {'Enter theta (degrees):','Enter d:', 'Enter a:', 'Enter alpha (degrees):'};
dlg_title = 'DH Input';
num_lines = 1;
defaultans = {'0','0','0','0'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
% convert input strings to numbers
params = str2double(answer);

% save parameters as variables
theta = params(1,1);
d = params(2,1);
a = params(3,1);
alpha = params(4,1);

% translation
d = [a*cosd(theta); a*sind(theta); d];
% rotation using two matrices
Q1 = [cosd(theta), -sind(theta), 0; sind(theta), cosd(theta), 0; 0, 0, 1];
Q2 = [1, 0, 0; 0, cosd(alpha), -sind(alpha); 0, sind(alpha), cosd(alpha)];
% rotation as one matrix
Q = Q1*Q2;

% output results to command window
disp('The resulting Q and d are:');
disp('Q:');
disp(Q);
disp('d:');
disp(d);

disp('Q1:');
disp(Q1);
disp('Q2:');
disp(Q2);