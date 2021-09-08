function [F_1, F_2, F_3, F_4, n] = friction_joined_model(CM,m,g,a,b,kf,u_all,epsilon)



    A = [1, 1, 1, 1;
         -b, -b, b, b;
         -a, a, a, -a];
    A = A + [0, 0, 0, 0;
             CM(3)*kf*u_all(2,1)/(norm(u_all(:,1))+epsilon), CM(3)*kf*u_all(2,2)/(norm(u_all(:,2))+epsilon), CM(3)*kf*u_all(2,3)/(norm(u_all(:,3))+epsilon), CM(3)*kf*u_all(2,4)/(norm(u_all(:,4))+epsilon);
             CM(3)*kf*u_all(1,1)/(norm(u_all(:,1))+epsilon), CM(3)*kf*u_all(1,2)/(norm(u_all(:,2))+epsilon), CM(3)*kf*u_all(1,3)/(norm(u_all(:,3))+epsilon), CM(3)*kf*u_all(1,4)/(norm(u_all(:,4))+epsilon)];
    B = [m*g; m*g*CM(2); -m*g*CM(1)];
    
    
    %Compute normals
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
    
    
    %Compute forces
    F_1 = -kf*n(1)*u_all(:,1)/(norm(u_all(:,1)) + epsilon);
    F_2 = -kf*n(2)*u_all(:,2)/(norm(u_all(:,2)) + epsilon);
    F_3 = -kf*n(3)*u_all(:,3)/(norm(u_all(:,3)) + epsilon);
    F_4 = -kf*n(4)*u_all(:,4)/(norm(u_all(:,4)) + epsilon);

end