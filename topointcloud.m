function [pc] = topointcloud(velo)
%veloΪ��ȡ��N*4������

normal=[velo(:,4),velo(:,4),velo(:,4)];
pc=pointCloud(velo(:,1:3),'Normal',normal);

end

