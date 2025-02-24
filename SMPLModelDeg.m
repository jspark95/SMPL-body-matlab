function [v, v_shaped, v_posed, rot_J_global] = SMPLModelDeg(Core, beta, theta)
% shapedirs, posedirs, regJoint, weights, kintree

%% Shape PCA
v_shaped = squeeze(sum(permute(Core.shapeDirs, [3 1 2]) .* beta)) + Core.meanVerts;

%% Joint Regressor
J = Core.regJoint * v_shaped;   

%% Pose Regressor
nPosedir = size(Core.poseDirs, 3); % 23 x 9 = 207
dTheta = zeros(nPosedir, 1);
for i = 2:length(theta) % Except Global Transformation
    dTemp = roddeg(theta(i,:)) - eye(3);
    dTheta((i-2)*9+(1:9)) = dTemp(:);
end
v_posed = squeeze(sum(permute(Core.poseDirs, [3 1 2]) .* dTheta)) + v_shaped;

%% Linear Blend Skinning
% Get Rotation Matrix along the Joint
% Get Rotation Matrix along the Joint
rot_J = zeros(4, 4, length(Core.kinTree)); % 4 x 4 x 24
rot_J(:, :, 1) = roddeg(theta(1, :), J(1,:));
for j = 2:length(Core.kinTree)
    childRot = roddeg(theta(j, :), (J(j, :) - J((Core.kinTree(1, j)+1), :)));
    rot_J(:,:,j) = rot_J(:, :, (Core.kinTree(1, j)+1)) * childRot;
end

rot_J_global = rot_J; % Global Rotation matrix of the Joint
for i = 1:length(Core.kinTree)
    rot_J(:, :, i) = rot_J(:, :, i) - [zeros(4,3) (rot_J(:,:,i) * [J(i,:) 0]')];
end

%% Make Skinned Vertices
nJoint = size(Core.blendWeights, 2); % 24
nVertex = size(Core.blendWeights, 1); % 6890

% rot_J [(4 x 4) x 24] * weights [(24 x 6890)']
% Jtr = squeeze(rot_J_global(1:3, 4, :)); 
T = reshape(reshape(rot_J, [16, nJoint]) * Core.blendWeights', [4, 4, nVertex]); % 4 x 4 x 6890

rest_shape_h = repmat([v_posed ones(length(v_posed),1)], 1, 1, 4); % 6890 x 4 x 4
v = squeeze(sum(permute(T, [3 2 1]) .* rest_shape_h, 2))';
v = v(1:3, :) ./ v(4, :); % Homogenous to Cartesian
v = v(1:3, :)'; % Final Src vertices set

end

