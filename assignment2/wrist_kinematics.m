function wrist_solns = wrist_kinematics(c3, k_d, j_d)
    % use c3 to find k3, j3
    k3 = c3(:,3);
    j3 = c3(:,2);
    
    % use kahan P3 to find theta4 and theta5
    [theta4_1, theta5_1, theta4_2, theta5_2] = KahanP3(k3, -j3, k_d, k3);
    % reverse direction of theta4 as kahan p3 uses theta4 in the opposite direction
    theta4_1 = -1 * theta4_1;
    theta4_2 = -1 * theta4_2;
    
    % create c5 from theta 3 and theta4 solutions
    theta_vals = [theta4_1; theta5_1];
    c5_1 = create_c5(theta_vals, c3);
    j5_1 = c5_1(:,2);
    k5_1 = c5_1(:,3);
    
    theta_vals = [theta4_2; theta5_2];
    c5_2 = create_c5(theta_vals, c3);
    j5_2 = c5_2(:,2);
    k5_2 = c5_2(:,3);
    
    % solve for theta 6 solutions using kahan P2
    theta6_1 = KahanP2(k5_1, j5_1, j_d);
    theta6_2 = KahanP2(k5_2, j5_2, j_d);
    
    % find alternate theta 6 values due to large theta 6 rotation limit
    if theta6_1 < 0
        theta6_3 = theta6_1 + 360;
    else
        theta6_3 = theta6_1 - 360;
    end
    
    if theta6_2 < 0
        theta6_4 = theta6_2 + 360;
    else
        theta6_4 = theta6_2 - 360;
    end
    
    soln1 = [theta4_1; theta5_1; theta6_1];
    soln2 = [theta4_1; theta5_1; theta6_3];
    soln3 = [theta4_2; theta5_2; theta6_2];
    soln4 = [theta4_2; theta5_2; theta6_4];
    
    wrist_solns = [soln1 soln2 soln3 soln4];
end