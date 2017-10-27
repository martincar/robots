function c3 = create_c3(soln)
    theta1 = soln(1);
    theta2 = soln(2);
    theta3 = soln(3);
    
    T_11 = k_rot(theta1)*i_rot(-90);
    T_12 = [0; 0; 0];
    T1 = [T_11 T_12; zeros(1,3) 1];

    T_11 = k_rot(theta2)*i_rot(180);
    T_12 = k_rot(theta2)*[431.8; 0; 0];
    T2 = [T_11 T_12; zeros(1,3) 1];
    
    T_11 = k_rot(theta3+90)*i_rot(90);
    T_12 = [0; 0; -149.09] + k_rot(theta3+90)*[20.32; 0; 0];
    T3 = [T_11 T_12; zeros(1,3) 1];
    
    sys3 = T1*T2*T3;
    c3 = sys3(1:3, 1:3);
end