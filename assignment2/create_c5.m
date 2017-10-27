function c5 = create_c5(soln, c3)
    theta4 = soln(1);
    theta5 = soln(2);
    
    c34 = k_rot(theta4)*i_rot(90);
    c45 = k_rot(theta5)*i_rot(-90);
    
    c5 = c3 * c34 * c45;
end