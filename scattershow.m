function [] = scattershow(velo)
scatter3(-velo(:,1),-velo(:,2),velo(:,3),1,velo(:,4));
az=90;el=90;
xlim([-80 80]);
ylim([-80 80]);
zlim([-4 4]);
view(az,el);
end

