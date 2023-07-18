function T = v2t(pose)
%v2t returns a 4x4 isometry from a pose vector 
% (rpy parametrization Rx*Ry*Rz)
    T = eye(4,4);
    if length(pose) == 3
        T(1:2, 4)  = pose(1:2);
        T(1:3,1:3) = eul2rotm([0,0,pose(3)],"ZYX");
    elseif lenght(pose) == 6
        T(1:3, 4) = pose(1:3);
        T(1:3,1:3) = eul2rotm([pose(4),pose(5),pose(6)],"ZYX");
    else
        error("v2t| pose lenght has to be 3 (2D) or 6 (3D)");
    end
end

