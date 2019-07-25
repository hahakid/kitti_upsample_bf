function [pc] = topointcloud(velo)
%velo为读取的N*4向量，

normal=[velo(:,4),velo(:,4),velo(:,4)];
pc=pointCloud(velo(:,1:3),'Normal',normal);

end

