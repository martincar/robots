function angle = KahanP1(u,v)
% Input: Two vectors, axis of rotation between them
% output: Angle between the two vectors

    % normalize vectors
    u = u/norm(u);
    v = v/norm(v);
    
    % apply kahan P1
    angle = 2 * atand( norm(u - v) / norm(u + v) );
end

