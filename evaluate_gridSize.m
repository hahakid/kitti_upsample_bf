%{
[tform,movingReg,rmse]= pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
第三个参数为
additionally returns the root mean squared error of the 
Euclidean distance between the aligned point clouds, 
using any of the preceding syntaxes.

在进行ICP算法时，计算连续两帧之间点云的 均方根误差


可能影响的参数：
    0.是否过滤/裁剪（pcdenoise）,尽量只用默认情况
    1.是否进行降采样（pcdownsample）
    2.降采样算法，及参数  random--percentage；gridAverage--gridStep；nonuniformGridSample--maxNumPoints
        'random'	'random'
        'gridAverage'	'gridAverage'
        'gridAverage'	'gridAverage'
        'random'	'nonuniformGridSample'
    3.帧数（多帧叠加后的结果）=控制window
%}
clear; dbstop error; clc;
warning off; close all;
%file path
%path='./training/velodyne/';
path='../';
file='0000';%选择数据集0000/0012
velo_path=[path,file];
velo= dir([velo_path,'/*.bin']);

step=1;
window=2;
%framelist=[];
velo1=veloread(velo_path,0); %the first frame 
%normal1=[velo1(:,4),velo1(:,4),velo1(:,4)];%因为读取的point是按照N*3，所以必须给每个点常量。要么就直接只注册velo(:,1:3),保留velo(:,4)
%以第一帧为基准，向上叠加
ptCloudRef = pointCloud(velo1(:,1:3));%,'Normal',normal1);   
last=ptCloudRef;

%downsampleTest(ptCloudRef,3);
%一次性显示多个点云 对比
%pclist={ptCloudRef;ptCloudRef;ptCloudRef;ptCloudRef};
%layout=[2 2];
%multishow(pclist,layout);

forwardflog=1;% 1- use first frame as fixed, and sencond frame as moving;
%参数配置%%%%%%%%%%%%%%%%%%%%%%%%
%不同的down sample 方法要对应 pcregrigid的参数,为了方便还是将降采样与计算合并，共计5类
%0-无降采样
%1-random+random+pointToPoint
%2-gridAverage+gridAverage+pointToPoint
%3-gridAverage+gridAverage+pointToPlane
%4-random+nonuniformGridSample+pointToPlane
DownSampleOption=0;  

gridSize = 0.1; % only works when using 'gridAverage'
percentage=0.2; % only works when using 'random', [0,1] 
maxNumPoints=6; % nonuniformGridSample [6,]

rmselist=zeros(length(velo)-1,1);
count=1;
%开始遍历
for frame=1:1:length(velo)-1%window
    ptCloudRef=last;
    
    velo=veloread(velo_path,frame);
    %normal=[velo(:,4),velo(:,4),velo(:,4)];%因为读取的point是按照N*3，所以必须给每个点常量。要么就直接只注册velo(:,1:3),保留velo(:,4)
    ptCloudCurrent =pointCloud(velo(:,1:3));%,'Normal',normal);
    
    % downsample 
    % 由于downsample的策略需要跟 pcregistericp 的策略关联
    % 主要控制4种情况  0-all；  1，2- pointToPoint； 2，3-pointToPlane
    if DownSampleOption==0 % no down sample
        fixed=ptCloudRef;
        moving = ptCloudCurrent;
        metric='pointToPlane'; %pointToPlane
    elseif DownSampleOption==1 % A=random|B=random 
        fixed=pcdownsample(ptCloudRef, 'random', percentage);
        moving = pcdownsample(ptCloudCurrent, 'random', percentage);
        metric='pointToPoint';
    elseif DownSampleOption==2 %  A=gridAverage|B=gridAverage 
        fixed=pcdownsample(ptCloudRef, 'gridAverage', gridSize);
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
        metric='pointToPoint';
    elseif DownSampleOption==3 %  A=gridAverage|B=gridAverage 
        fixed=pcdownsample(ptCloudRef, 'gridAverage', gridSize);
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
        metric='pointToPlane';
    elseif DownSampleOption==4 % Random 
        fixed=pcdownsample(ptCloudRef, 'random', percentage);
        moving = pcdownsample(ptCloudCurrent, 'nonuniformGridSample', maxNumPoints);
        metric='pointToPlane';
    end
    
    mergeSize = 0.015;
    if forwardflog==1 % forward
        [tform,movingReg,rmse]= pcregrigid(moving, fixed, 'Metric',metric,'Extrapolate', true);
        %ptCloudAligned = pctransform(ptCloudCurrent,tform);
        %ptCloudRef = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
        rmse
        rmselist(count)=rmse;
    else %backward
        [tform,movingReg,rmse] = pcregrigid(fixed, moving, 'Metric',metric,'Extrapolate', true);
        %ptCloudAligned = pctransform(ptCloudRef,tform);
        %ptCloudCurrent = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
        rmse
        rmselist(count)=rmse;
    end
    last=ptCloudCurrent;%ptCloudRef;
    count=count+1;
end

% function [cvelo]=veloread(velo_dir,frame)
% sprintf('%s/%06d.bin',velo_dir,frame+i);
% fd = fopen(sprintf('%s/%06d.bin',velo_dir,frame+i),'rb');% 打开雷达二进制文件
%         if fd < 1
%             fprintf('No LIDAR files !!!\n');
%             keyboard
%         else
%         cvelo = fread(fd,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
%         %figure;plot3(velo(:,1),velo(:,2),velo(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('Original Velo');
%         fclose(fd);
%         end
% end
