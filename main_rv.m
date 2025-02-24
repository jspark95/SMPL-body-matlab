% ready_arguments
clear all;
addpath('./npy_matlab');
load('m_model.mat'); % male template
% load('w_model.mat'); % female template

% 추후 입력 파트에서 들어올 부분

beta = zeros(10, 1);	% Shape param (10)
theta = zeros(24, 3);	% Pose param (1 Global + 23 Joints x 3)

% beta = readNPY('C:/Users/jspark/Desktop/smpl/betas.npy');
% theta = readNPY('C:/Users/jspark/Desktop/smpl/pose.npy');
% % beta = readNPY('betas.npy');
% % theta = readNPY('pose.npy');
% theta = reshape(theta, [3. 24])';

theta(14,:) = -0.5*[0.1434,0.4534,0.4064];
theta(15,:) = 0.5*[0.1434,0.4534,0.4064];
theta(17,:) = -[0.14,0.45,0.4064];
theta(18,:) = [0.14,0.45,0.4064];
%% Shape PCA
v_shaped = squeeze(sum(permute(shapedirs, [3 1 2]) .* beta)) + v_temp;

%% Joint Regressor
J = reg_J * v_shaped;

%% Pose Regressor
nPosedir = size(posedirs, 3); % 23 x 9 = 207
dTheta = zeros(nPosedir, 1);
for i = 2:length(theta) % Except Global
    dTemp = rod(theta(i,:)) - eye(3);
    dTheta((i-2)*9+(1:9)) = dTemp(:);
end
v_posed = squeeze(sum(permute(posedirs, [3 1 2]) .* dTheta)) + v_shaped;

%% Linear Blend Skinning
% Get Rotation Matrix along the Joint
rot_J = zeros(4, 4, length(kintree)); % 4 x 4 x 24
rot_J(:, :, 1) = rod(theta(1, :), J(1,:));
for j = 2:length(kintree)
    childRot = rod(theta(j, :), (J(j, :) - J((kintree(1, j)+1), :)));
    rot_J(:,:,j) = rot_J(:, :, (kintree(1, j)+1)) * childRot;
end

rot_J_global = rot_J;
for i = 1:length(kintree)
    rot_J(:, :, i) = rot_J(:, :, i) - [zeros(4,3) (rot_J(:,:,i) * [J(i,:) 0]')];
end

%% Make Skinned Vertices
nJoint = size(weights, 2); % 24
nVertex = size(weights, 1); % 6890

% rot_J [(4 x 4) x 24] * weights [(24 x 6890)']
T = reshape(reshape(rot_J, [16, nJoint]) * weights', [4, 4, nVertex]); % 4 x 4 x 6890

rest_shape_h = repmat([v_posed ones(length(v_posed),1)], 1, 1, 4); % 6890 x 4 x 4
v = squeeze(sum(permute(T, [3 2 1]) .* rest_shape_h, 2))';

v = v(1:3, :) ./ v(4, :); % Homogenous to Cartesian
v = v(1:3, :)';

Jtr = squeeze(rot_J_global(1:3, 4, :)); 

figure, dispFace(v, f, [.8 .8 .8]);

function R = rod(V, J)
    if all(V == 0)
        R = eye(3);
    else
        theta=sqrt(V(1)^2 + V(2)^2 + V(3)^2);
        omega=[0 -V(3) V(2); V(3) 0 -V(1); -V(2) V(1) 0];
        R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
    end

    if (nargin > 1)
        if size(J, 2) == 3
            J = J';
        end
        
        R = [R J];
        R = [R; [0 0 0 1]];
    end
end