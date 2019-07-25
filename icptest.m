
clear; dbstop error; clc;
warning off; close all;

path='./training/velodyne/';
file='0000';%0000
velo_path=[path,file];

velo= dir([velo_path,'/*.bin']);
step=1;
window=2;
framelist=[];
xl=[-10 30];
yl=[-15 15];
zl=[-2.5 2.5];

player1=pcplayer(xl,yl,zl);
player2=pcplayer(xl,yl,zl);% for compare

%hide(player);
gridSize = 0.1;

velo1=veloread(velo_path,0);
%normal1=[velo1(:,4),velo1(:,4),velo1(:,4)];%因为读取的point是按照N*3，所以必须给每个点常量。要么就直接只注册velo(:,1:3),保留velo(:,4)
%以第一帧为基准，向上叠加
ptCloudRef = pointCloud(velo1(:,1:3));%,'Normal',normal1);   
%fixed=ptCloudRef;%pcdownsample(ptCloudRef, 'gridAverage', gridSize);
%view(player,ptCloudRef);% before
last=ptCloudRef;

forwardflog=1;% 1- use first frame as fixed, and sencond frame as moving;

for frame=1:1:length(velo)-1%window
    fixed=pcdownsample(ptCloudRef, 'gridAverage', gridSize);%每轮先对基准帧降采样
    
    velo=veloread(velo_path,frame);
    %normal=[velo(:,4),velo(:,4),velo(:,4)];%因为读取的point是按照N*3，所以必须给每个点常量。要么就直接只注册velo(:,1:3),保留velo(:,4)
    ptCloudCurrent =pointCloud(velo(:,1:3));%,'Normal',normal);
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    
    mergeSize = 0.015;
    
    if forwardflog==1 % forward
        [tform,movingReg,rmse]= pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
        ptCloudAligned = pctransform(ptCloudCurrent,tform);
        ptCloudRef = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
    else %backward
        tform = pcregrigid(fixed, moving, 'Metric','pointToPlane','Extrapolate', true);
        ptCloudAligned = pctransform(ptCloudRef,tform);
        ptCloudCurrent = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
    end
    
    
    
    %figure;
    %subplot(2,1,1);
    view(player1,last);
    %hold on;
    %subplot(2,1,2);
    view(player2,ptCloudRef);
    
    
    
    last=ptCloudRef;
    
    %mat=pcregrigid(pc1,pc2);
    %pc1=pctransform(pc1,mat);
    %pcshow(pc1);
    
   
end


function [cvelo]=veloread(velo_dir,frame)
sprintf('%s/%06d.bin',velo_dir,frame+i);
fd = fopen(sprintf('%s/%06d.bin',velo_dir,frame+i),'rb');% 打开雷达二进制文件
        if fd < 1
            fprintf('No LIDAR files !!!\n');
            keyboard
        else
        cvelo = fread(fd,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
        %figure;plot3(velo(:,1),velo(:,2),velo(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('Original Velo');
        fclose(fd);
        end
end
