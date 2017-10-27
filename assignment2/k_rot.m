% create function to return k rotation matrix from angle input (in degrees)
function f = k_rot(n)
    f = [cosd(n), -sind(n), 0; sind(n), cosd(n), 0; 0, 0, 1];
end