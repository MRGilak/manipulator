function H = dh(theta, d, a, alpha)

    M_theta = [ cos(theta) -sin(theta)  0  0;
                sin(theta)  cos(theta)  0  0;
                0           0           1  0;
                0           0           0  1 ];

    M_d = [ 1 0 0 0;
            0 1 0 0;
            0 0 1 d;
            0 0 0 1 ];

    M_a = [ 1 0 0 a;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1 ];

    M_alpha = [ 1 0           0            0;
                0 cos(alpha) -sin(alpha)   0;
                0 sin(alpha)  cos(alpha)   0;
                0 0           0            1 ];

    H = M_theta * M_d * M_a * M_alpha;
end
