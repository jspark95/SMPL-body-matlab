function A = deg2axisang(deg)
rad = deg2rad(deg);
rotm = eul2rotm(rad);
axang = rotm2axang(rotm);
A = zeros(3,1);
A(1) = axang(1) .* axang(4);
A(2) = axang(2) .* axang(4);
A(3) = axang(3) .* axang(4);
end