clear all; close all; clc;
root_dir = '.';
data_set = 'training';

cam = 2; % 2 = left color camera
seq_idx=20; % 0 5 20
velo_dir = fullfile(root_dir,data_set, sprintf('/velodyne/%04d',seq_idx));
totalframes=length(dir(fullfile(velo_dir,'*.bin')));
xlimits=[-10 50];ylimits=[-20 20];zlimits=[-2 2];
playerwind=pcplayer(xlimits,ylimits,zlimits);
window=2;

for frame=0+window-1:1:totalframes-1
    disp(frame);
    for i=0:1:window-1
        fd = fopen(sprintf('%s/%06d.bin',velo_dir,frame+i),'rb');% 打开雷达二进制文件
        if fd < 1
            fprintf('No LIDAR files !!!\n');
            keyboard
        else
        velo = fread(fd,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
        %figure;plot3(velo(:,1),velo(:,2),velo(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('Original Velo');
        fclose(fd);
        end
        vlist{i+1}=velo;     
    end
    cvelo=vlist{1};
    pc=topointcloud(cvelo);
    view(playerwind,pc);
end


%close all;
