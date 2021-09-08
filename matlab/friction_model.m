function f = friction_model(vel,kf,n,epsilon)

    %Simple friction force
    f = -kf*n*vel/(norm(vel)+epsilon);
    
end % end function