function angle = KahanP4(a,b,c)
    if (a + b > c) && (c > abs(a-b))
        numerator = (a+b)^2 - c^2;
        denominator = c^2 - (a-b)^2;
        
        angle = 2 * atand( sqrt(numerator / denominator));
    else
        error('No Solution for Kahan P4')
    end
end

