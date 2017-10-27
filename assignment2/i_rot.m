% create function to return i rotation matrix from angle input (in degrees)
function f = i_rot(n)
    f = [1 0 0; 0 cosd(n) -sind(n); 0 sind(n) cosd(n)];
end