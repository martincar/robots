function angle = KahanP2(s,u,w)
% Input: Two vectors, axis of rotation between them
% output: Angle between the two vectors

    % normalize vectors
    s = s/norm(s);
    u = u/norm(u);
    w = w/norm(w);
    
    % apply cross products
    s_cross_neg = cross(s, (u-w));
    s_cross_pos = cross(s, (u+w));
    
    % apply kahan P2
    angle = 2 * atand( norm(s_cross_neg) / norm(s_cross_pos) );
    % find sign of angle
    angle_sign = sign( dot(w, s_cross_neg) );
    
    % set angle with sign
    angle = angle * angle_sign;
    
end

