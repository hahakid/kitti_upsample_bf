% clear and close everything
clear all; close all; clc;
%disp('======= KITTI DevKit Demo =======');

% options
%root_dir = '/media/PHILIP_KITTI/datasets/kitti/2012_tracking';
root_dir = '.';

data_set = 'training';
% set camera
cam = 2; % 2 = left color camera
% show data for tracking sequences
%nsequences = numel(dir(fullfile(root_dir,data_set, sprintf('image_%02d',cam))))-2;
seq_idx=0; % 0 5 20
% get sub-directories
%image_dir = fullfile(root_dir,data_set, sprintf('image_%02d/%04d',cam, seq_idx));
velo_dir = fullfile(root_dir,data_set, sprintf('/velodyne/%04d',seq_idx));
%label_dir = fullfile(root_dir,data_set, sprintf('label_%02d',cam));
%calib_dir = fullfile(root_dir,data_set, 'calib');
%P = readCalibration(calib_dir,seq_idx,cam);
% get number of images for this dataset
totalframes=length(dir(fullfile(velo_dir,'*.bin')));

%nimages = length(dir(fullfile(image_dir, '*.png')));
% load labels
%tracklets = readLabels(label_dir, seq_idx);
% main loop
window=2;
for frame=0+window-1:1:totalframes-1
    disp(frame);
    
    for i=0:1:window-1
        %ptCloud = pcread('teapot.ply');
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
    %showpc(vlist{1});
    %velo_diff(vlist{1},vlist{2}); %显示两帧点云区别
    player(vlist{1});
    %scatter(velo(:,1),velo(:,2),1);
    %scattershow(velo);
    %test(velo);
    %cameratoolbar;
    %pause(0.1);
end


%close all;
