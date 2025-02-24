function smpl_pose_params = convertToSMPLPose(joint_angles)
    % joint_angles는 24x3 배열이며 각 행은 하나의 관절에 대한 [pitch, yaw, roll]을 나타냄 (도 단위)

    % 라디안으로 변환
    joint_radians = deg2rad(joint_angles);

    % 각 관절에 대한 쿼터니언 초기화
    axang = zeros(24, 3);

    % 각 관절에 대해 쿼터니언 계산
    for i = 1:size(joint_radians, 1)
        % 회전 행렬 생성
        R = eul2rotm(joint_radians(i, :), 'ZYX'); % 'ZYX'는 회전 순서

        tmp = rotm2axang(R);

        % 쿼터니언으로 변환
        axang(i, :) = tmp(1:3) * tmp(4);
    end

    % 쿼터니언 배열을 SMPL 포즈 매개변수로 재구성
    smpl_pose_params = axang;
end