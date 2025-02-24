function R = rodrigues(vx, vy, vz, J)

if size(J,2) == 3
    J = J';
end

if vx==0&&vy==0&&vz==0
    R = eye(3);
else
    theta=sqrt(vx^2+vy^2+vz^2);
    omega=[0 -vz vy; vz 0 -vx; -vy vx 0];
    R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
end
R = [R J];
R = [R; [0 0 0 1]];
end
