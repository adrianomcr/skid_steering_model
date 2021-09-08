function n = compute_normals(CM,m,g,a,b,F_last)


    A = [1, 1, 1, 1;
         -b, -b, b, b;
         -a, a, a, -a];
    B = [m*g; m*g*CM(2) + F_last(2)*CM(3); -m*g*CM(1) + F_last(1)*CM(3)];

    n = A'*inv(A*A')*B;

    %Check if one of the normals is negative
    [min_n, id_min] = min(n);
    if(min_n < 0)

        A(:,id_min) = [];
        n3 = A'*inv(A*A')*B;

        n_old = n;
        n = ones(4,1);
        n(id_min) = 0;
        for k = 1:1:4
            if (k==id_min)
                n(k) = 0;
            else
                n(k) = n3(1);
                n3(1) = [];
            end
        end  

    end

    
    

end %function



