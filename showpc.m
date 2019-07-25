function [] = showpc(velo)
pc=pointCloud(velo(:,1:3));%强制转换到pc

%转换矩阵
x = pi/180;% 1 degree 
R = [ cos(x) sin(x) 0 0
     -sin(x) cos(x) 0 0
      0         0   1 0
      0         0   0 1];

tform = affine3d(R);

lower = min([pc.XLimits pc.YLimits]);
upper = max([pc.XLimits pc.YLimits]);
  
xlimits = [lower upper];
ylimits = [lower upper];
zlimits = pc.ZLimits;

player = pcplayer(xlimits,ylimits,zlimits);

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');

for i = 1:1:720      
    pc = pctransform(pc,tform);     
    view(player,pc);
    pause(0.1);
end

end

