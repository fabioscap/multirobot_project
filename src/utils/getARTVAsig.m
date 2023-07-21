function h_m = getARTVAsig(p_r,p_t,R_r, R_t, noise,m)
%getARTVAsig eq. 6 of Avalanche Victim Search via Robust Observers
arguments
    p_r % pose of the receiver
    p_t=[0;0;0] % pose of the transmitter
    R_r = eye(3) % orientation of the receiver
    R_t = eye(3) % orientation of the transmitter
    noise=true % enable/disable additive noise
    m=1.0 % amplitude of transmitter (see Avalanche Victim Search...)
end
    if length(p_r) == 2
        p_r = [p_r; 0];
    end 
    if length(p_t) == 2
        p_t = [p_t; 0];
    end
    % transform the receiver position in the transmitter frame
    T_r = [R_r p_r; 0 0 0 1];
    T_t = [R_t p_t; 0 0 0 1];

    T_rt = T_t * inv(T_r);

    x_rt = T_rt(1:3, 4);

    % compute the field in the transmitter frame
    % (eq. 2)
    A = [ 2*x_rt(1)^2 - x_rt(2)^2 - x_rt(3)^2;
          3*x_rt(1)*x_rt(2);
          3*x_rt(1)*x_rt(3);];

    h_t = ( m / ( 4*pi*norm(x_rt)^5 )) * A;

    % rotate the value in the receiver frame
    T_tr = inv(T_rt);
    h_n = T_tr(1:3, 1:3) * h_t;


    % add noise
    if noise
        % TODO implement noise
        error("not implemented");
    else
        h_m = h_n;
    end

    % maybe return directly the norm? it seems that
    % they use ||h_m|| in the estimation process
end

