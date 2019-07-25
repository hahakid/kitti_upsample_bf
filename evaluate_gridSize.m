%{
[tform,movingReg,rmse]= pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
����������Ϊ
additionally returns the root mean squared error of the 
Euclidean distance between the aligned point clouds, 
using any of the preceding syntaxes.

�ڽ���ICP�㷨ʱ������������֮֡����Ƶ� ���������


����Ӱ��Ĳ�����
    0.�Ƿ����/�ü���pcdenoise��,����ֻ��Ĭ�����
    1.�Ƿ���н�������pcdownsample��
    2.�������㷨��������  random--percentage��gridAverage--gridStep��nonuniformGridSample--maxNumPoints
        'random'	'random'
        'gridAverage'	'gridAverage'
        'gridAverage'	'gridAverage'
        'random'	'nonuniformGridSample'
    3.֡������֡���Ӻ�Ľ����=����window
%}
clear; dbstop error; clc;
warning off; close all;
%file path
%path='./training/velodyne/';
path='../';
file='0000';%ѡ�����ݼ�0000/0012
velo_path=[path,file];
velo= dir([velo_path,'/*.bin']);

step=1;
window=2;
%framelist=[];
velo1=veloread(velo_path,0); %the first frame 
%normal1=[velo1(:,4),velo1(:,4),velo1(:,4)];%��Ϊ��ȡ��point�ǰ���N*3�����Ա����ÿ���㳣����Ҫô��ֱ��ֻע��velo(:,1:3),����velo(:,4)
%�Ե�һ֡Ϊ��׼�����ϵ���
ptCloudRef = pointCloud(velo1(:,1:3));%,'Normal',normal1);   
last=ptCloudRef;

%downsampleTest(ptCloudRef,3);
%һ������ʾ������� �Ա�
%pclist={ptCloudRef;ptCloudRef;ptCloudRef;ptCloudRef};
%layout=[2 2];
%multishow(pclist,layout);

forwardflog=1;% 1- use first frame as fixed, and sencond frame as moving;
%��������%%%%%%%%%%%%%%%%%%%%%%%%
%��ͬ��down sample ����Ҫ��Ӧ pcregrigid�Ĳ���,Ϊ�˷��㻹�ǽ������������ϲ�������5��
%0-�޽�����
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
%��ʼ����
for frame=1:1:length(velo)-1%window
    ptCloudRef=last;
    
    velo=veloread(velo_path,frame);
    %normal=[velo(:,4),velo(:,4),velo(:,4)];%��Ϊ��ȡ��point�ǰ���N*3�����Ա����ÿ���㳣����Ҫô��ֱ��ֻע��velo(:,1:3),����velo(:,4)
    ptCloudCurrent =pointCloud(velo(:,1:3));%,'Normal',normal);
    
    % downsample 
    % ����downsample�Ĳ�����Ҫ�� pcregistericp �Ĳ��Թ���
    % ��Ҫ����4�����  0-all��  1��2- pointToPoint�� 2��3-pointToPlane
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
% fd = fopen(sprintf('%s/%06d.bin',velo_dir,frame+i),'rb');% ���״�������ļ�
%         if fd < 1
%             fprintf('No LIDAR files !!!\n');
%             keyboard
%         else
%         cvelo = fread(fd,[4 inf],'single')';% ��ȡ�״����ݣ���Ϊn*4�ľ���
%         %figure;plot3(velo(:,1),velo(:,2),velo(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('Original Velo');
%         fclose(fd);
%         end
% end
