function [theta1, phi1, theta2, phi2] = KahanP3(s,t,u,v)
    % normalize vectors
    s = s/norm(s);
    t = t/norm(t);
    u = u/norm(u);
    v = v/norm(v);
    
    % set up alpha beta solving matrices
    P = [ 1        dot(s,t);
          dot(t,s)    1     ];
    proj = [dot(u,s); dot(v,t)];
    
    % solve for alpha and beta
    scales = proj / P;
    alpha = scales(1);
    beta = scales(2);
    
    % find z using solved alpha, beta
    z = alpha*s + beta*t;
    
    % solve if solution exists
    if norm(z) > 1
        error('No solution for Kahan P3')
    else    
        % find both solutions for w vector
        w1 = z + sqrt(1 - norm(z)^2) * cross(s,t) / norm(cross(s,t));
        w2 = z - sqrt(1 - norm(z)^2) * cross(s,t) / norm(cross(s,t));
        
        theta1 = KahanP2(s,u,w1);
        phi1 = KahanP2(t,v,w1);
        
        theta2 = KahanP2(s,u,w2);
        phi2 = KahanP2(t,v,w2);
    end
end

