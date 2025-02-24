maskIdx = [1:3188];
maskIdx = [8000:9000];
figure, dispFaceSep(obj.v, obj.f.v, [0, 0.8, 0], [0.8, 0, 0], maskIdx);

maskIdx = [];
idx = [];
for i = 1:10475
    if (obj.v(i,1) > 0.0793 && obj.v(i,1) < 0.0794 && obj.v(i,2) > 0.1299 && obj.v(i,2) < 0.1300 && obj.v(i,3) > -0.0114 && obj.v(i,3) < -0.01139)
        idx = [idx;i];
    end
end
figure, dispFaceSep(obj.v, obj.f.v, [0, 0.8, 0], [0.8, 0, 0], maskIdx);