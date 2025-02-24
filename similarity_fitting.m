% Similarity fitting
function [m, s, R, t] = similarity_fitting(source, target, s_lm, t_lm)
m = mean(source(s_lm,:));
Vsrc = source(s_lm,:) - m;
Vtar = target(t_lm, :) - mean(target(t_lm, :));
s = sqrt(sum(sum(Vtar.^2, 2)) / sum(sum(Vsrc.^2, 2))); % Scale
Vsrc = Vsrc * s;

M = zeros(3, 3);
for i=1:length(Vsrc)
    % Covariance Matrix
    M = M + (Vsrc(i, :)' * Vtar(i, :));
end

[U, ~, V] = svd(M);
R = V * U'; % Rotation

if (det(R) < 0)
    R(:, 3) = -R(:, 3);
    disp('Check to Reflection!');
end

t = target(t_lm, :) - (R* Vsrc')';
t = mean(t); % Translation
end