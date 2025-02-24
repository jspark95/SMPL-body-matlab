% new_s_lm = [];
% for i = 0:15
%     new_s_lm = [new_s_lm; src_joint(16-i).Position];
% end
% 
% aa_lm = [];
% for i = 1:7
%     aa_lm = [aa_lm; target_js1(i).Position];
% end
% 
% aaa_lm = [];
% for i = 0:14
%     aaa_lm = [aaa_lm; target_js2(15-i).Position];
% end
% 
% t_idx1 = zeros(7,1);
% for i = 1:7
% %     s_idx(i) = find(v_temp(:,1)==new_s_lm(i,1) & v_temp(:,2)==new_s_lm(i,2) & v_temp(:,3)==new_s_lm(i,3));
%     t_idx1(i) = find(points(:,1)==aa_lm(i,1) & points(:,2)==aa_lm(i,2) & points(:,3)==aa_lm(i,3));
% end
% 
% t_idx2 = zeros(15,1);
% for i = 1:15
% %     s_idx(i) = find(v_temp(:,1)==new_s_lm(i,1) & v_temp(:,2)==new_s_lm(i,2) & v_temp(:,3)==new_s_lm(i,3));
%     t_idx2(i) = find(points(:,1)==aaa_lm(i,1) & points(:,2)==aaa_lm(i,2) & points(:,3)==aaa_lm(i,3));
% end
% 
% new_s_lm2 = src_joint2.Position;
% find(v_temp(:,1)==new_s_lm(1,1) & v_temp(:,2)==new_s_lm(1,2) & v_temp(:,3)==new_s_lm(1,3))
% s_idx = zeros(17,1);
% t_idx = zeros(15,1);
% for i = 1:15
% %     s_idx(i) = find(v_temp(:,1)==new_s_lm(i,1) & v_temp(:,2)==new_s_lm(i,2) & v_temp(:,3)==new_s_lm(i,3));
%     t_idx(i) = find(points(:,1)==t_lm(i,1) & points(:,2)==t_lm(i,2) & points(:,3)==t_lm(i,3));
% end
% 
% new_s_lm2 = src_joint2.Position;
% find(v_temp(:,1)==new_s_lm(1,1) & v_temp(:,2)==new_s_lm(1,2) & v_temp(:,3)==new_t_lm(1,3))
% s_idx = zeros(17,1);

eps = 1;
t_idx = zeros(22,1);
for i = 1:22
%     tmp = find(points(:,1) < (new_t_lm(i,1)+eps) & points(:,1) > (new_t_lm(i,1)-eps) & new_t_lm(:,2) < (new_t_lm(i,2)+eps) & points(:,2) > (new_t_lm(i,2)-eps) & points(:,3) < (new_t_lm(i,3)+eps) & points(:,3) > (new_t_lm(i,3)-eps));
    tmp = find(points(:,1) < (new_t_lm(i,1)+eps) & points(:,1) > (new_t_lm(i,1)-eps));
    t_idx(i) = tmp(1);
end

% t_idx = zeros(22,1);
% for i = 1:22
%     aa = find(points(:,1)==new_t_lm(i,1) & points(:,2)==new_t_lm(i,2) & points(:,3)==new_t_lm(i,3));
%     t_idx(i) = aa(1);
% end

