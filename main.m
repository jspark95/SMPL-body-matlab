% ready_arguments
clear all;
load('model.mat');


% 추후 입력 파트에서 들어올 부분
beta = zeros(10,1); % shape param
theta = zeros(24,3); % pose param

% beta = readNPY('C:/Users/jspark/Desktop/smpl/betas.npy');
theta2 = readNPY('C:/Users/jspark/Desktop/smpl/pose.npy');
theta2 = reshape(theta2, 3,24)';
theta(17:20,:) = theta2(17:20,:);

%% shape setting
% betasize = size(shapedirs,3);
% shapesize = numel(shapedirs)/betasize;
% shapedirs_rv = zeros(shapesize, betasize);
% for i = 1:betasize
%     tmp = reshape(shapedirs(:,:,i), shapesize, 1);
%     shapedirs_rv(:,i) = tmp;
% end
% shape_disp = shapedirs_rv*beta;
% v_shaped = reshape(shape_disp, length(shape_disp)/3, 3) + v_temp;
v_shaped = squeeze(sum(permute(shapedirs, [3 1 2]) .* beta)) + v_temp;

% Joint Regressor
J = reg_J * v_shaped;

% Pose Regressor
pp = theta(2:end, :);
nPosedir = size(posedirs, 3); % 23 x 9 = 207
dTheta = zeros(nPosedir, 1);
for i = 1:length(pp)
    dTheta((i-1)*9+(1:9)) = reshape(rod2(pp(i, 1), pp(i, 2), pp(i, 3)) - eye(3), [1, 9]);
end
v_posed = squeeze(sum(permute(posedirs, [3 1 2]) .* dTheta)) + v_shaped;

%%% pose setting
%pp = theta(2:24,:);
%refine_theta = [];
%for i = 1:length(pp)
%     tmp_theta = rod2(pp(i,1),pp(i,2),pp(i,3));
%     tmp_theta = tmp_theta - eye(3);
%     refine_theta = [refine_theta; tmp_theta];
% end
% refine_theta = reshape(refine_theta', numel(refine_theta),1);
% thetasize = size(posedirs,3);
% posesize = numel(posedirs)/thetasize;
% posedirs_rv = zeros(posesize, thetasize);
% for i = 1:thetasize
%     tmp = reshape(posedirs(:,:,i), posesize, 1);
%     posedirs_rv(:,i) = tmp;
% end
% pose_disp = posedirs_rv*refine_theta;
% v_posed = reshape(pose_disp, length(pose_disp)/3, 3) + v_shaped;

%% lbs part
rot_J = zeros(4, 4, length(kintree));
rot_J(:,:,1) = rodrigues(theta(1,1), theta(1,2), theta(1,3), J(1,:));
for j = 2:length(kintree)
    childtmp = rodrigues(theta(j,1), theta(j,2), theta(j,3), (J(j,:)-J((kintree(1,j)+1),:)));
    rot_J(:,:,j) = rot_J(:,:,(kintree(1,j)+1))*childtmp;
end
% rot_J(:,:,1) = rot_J_global
rot_J_global = rot_J;
for i = 1:length(kintree)
    d_results = [zeros(4,3) (rot_J(:,:,i)*[J(i,:) 0]')];
    rot_J(:,:,i) = rot_J(:,:,i) - d_results;
end

%% new vertices
numofjoint = size(rot_J,3);
jmatrixsize = numel(rot_J)/numofjoint;
rot_J_rv = zeros(jmatrixsize, numofjoint);
for i = 1:numofjoint
    tmp = reshape(rot_J(:,:,i), jmatrixsize, 1);
    rot_J_rv(:,i) = tmp;
end
tmpT = rot_J_rv * weights';
T = zeros(4,4,size(tmpT,2));
for i = 1:size(tmpT,2)
    T(:,:,i) = reshape(tmpT(:,i),4,4);
end
rest_shape_h = [v_posed ones(length(v_posed),1)];
T1 = reshape(T(:,1,:), [size(T,1) size(T,3)]);
T2 = reshape(T(:,2,:), [size(T,1) size(T,3)]);
T3 = reshape(T(:,3,:), [size(T,1) size(T,3)]);
T4 = reshape(T(:,4,:), [size(T,1) size(T,3)]);
v = T1.*repmat(rest_shape_h(:,1),1,4)' + T2.*repmat(rest_shape_h(:,2),1,4)' + T3.*repmat(rest_shape_h(:,3),1,4)' + T4.*repmat(rest_shape_h(:,4),1,4)';
v(4,:) = [];

Jtr = rot_J_global(1:3,4,:); 
Jtr = reshape(Jtr, [size(Jtr,1) size(Jtr,3)])';

comp_v = readNPY('C:/Users/jspark/Desktop/smpl/py_mesh.npy');

