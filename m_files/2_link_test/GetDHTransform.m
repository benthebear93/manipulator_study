function T = GetDHTransform( link_length_m, link_twist_rad, joint_offset_m, joint_angle_rad )

l = link_length_m;
a = link_twist_rad;
d = joint_offset_m;
th = joint_angle_rad;

T = [cosd(th)  -sind(th)*cosd(a)   sind(th)*sind(a)  l*cosd(th);
     sind(th)   cosd(th)*cosd(a)  -cosd(th)*sind(a)  l*sind(th);
          0            sind(a)           cosd(a)          d;
          0                0                0           1];


end

% T 행렬의 1행 4열은 x 좌표
% T 행렬의 2행 4열은 y 좌표
% T 행렬의 3행 4열은 z 좌표