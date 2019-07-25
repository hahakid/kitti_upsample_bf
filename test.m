function [] = test(velo)

figure;
pcshow([velo(:,1),velo(:,2),velo(:,3)]);%具备坐标轴自适应功能
title('1111111111');
xlabel('X');
ylabel('Y');
zlabel('Z');
end

