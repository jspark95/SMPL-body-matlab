function R = roddeg(V, J)
    if all(V == 0)
        R = eye(3);
    else
        joint_radians = deg2rad(V);
        R = eul2rotm(joint_radians, 'XYZ'); % 'ZYX'는 회전 순서, 결과 잘 안나오면 'XYZ'로 바꾸기
    end

    if (nargin > 1)
        if size(J, 2) == 3
            J = J';
        end
        
        R = [R J];
        R = [R; [0 0 0 1]];
    end
end