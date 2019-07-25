function [Ima_RangeD,Ima_RangeH,Ima_RangeA] = Fun_dense_range_map(calib,calib_dir,base_dir,...
    frame,ImaRGB)
image_size=size(ImaRGB);
T = Fun_open_calib(calib.name,calib_dir);
% T = Fun_open_calib(calib(frame+1).name,calib_dir);%
% 这句是我加的，为的是可以单独这里的代码，读取第frame个雷达文件

fd = fopen(sprintf('%s/%06d.bin',base_dir,frame),'rb');% 打开雷达二进制文件
if fd < 1
    fprintf('No LIDAR files !!!\n');
    keyboard
else
velo = fread(fd,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
%figure;plot3(velo(:,1),velo(:,2),velo(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('Original Velo');
% 这里画图的程序是我加的，为了看中间结果
fclose(fd);
end

% remove all points behind image plane (approximation)
idx = velo(:,1)<-3;% 原值=5，扩充到-3，可以消除两侧部分空白区域
velo(idx,:) = [];
%figure;plot3(px(:,1),px(:,2),px(:,3),'.');% 这里画图的程序是我加的，为了看中间结果

% project to image plane (exclude luminance)
px = (T.P2 * T.R0_rect * T.Tr_velo_to_cam * velo')';% 旋转到RGB平面，公式参考KITTI官网关于校正文件的使用的readme
px(:,1) = px(:,1)./px(:,3);%1,2是图像像素的xy坐标，3是深度
px(:,2) = px(:,2)./px(:,3);
%figure;plot3(px(:,1),px(:,2),px(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('project to image plane');
% 这里画图的程序是我加的，为了看中间结果

%20170831增加的hha矩阵------------------------------------------------------------------
pxvelo = [px,velo];%px(:,1)是雷达对应深度图像的列坐标，px(:,2)是雷达对应深度图像的行坐标，px(:,3)是雷达对应深度图像的深度，velo是该点雷达三维坐标
%上采样H
PtsH = zeros(size(pxvelo,1),4);
PtsH(:,1) = pxvelo(:,1);
PtsH(:,2) = pxvelo(:,2);
PtsH(:,3) = pxvelo(:,3);
yMinh = prctile(pxvelo(:,7), 0);
if yMinh < -4
    yMinh = -4;
end
PtsH(:,4) = (pxvelo(:,7) - yMinh);% * 100;
%PtsH(:,4) =pxvelo(:,4);
ix = PtsH(:,1)<1;                 PtsH(ix,:)=[];% 忽略位置小于1的像素
ix = PtsH(:,1)>size(ImaRGB,2);    PtsH(ix,:)=[];% 忽略超出RGB范围的像素   
ix = PtsH(:,2)<1;                 PtsH(ix,:)=[];% 忽略位置小于1的像素
ix = PtsH(:,2)>size(ImaRGB,1);    PtsH(ix,:)=[];% 忽略超出RGB范围的像素   
%ix = PtsH(:,4)<1;                 PtsH(ix,:)=[];% 忽略位置小于1的像素
%ix = PtsH(:,4)>600;    PtsH(ix,:)=[];% 忽略超出   
PtsH = sortrows(PtsH,2);% 按照第二列排序
c_px = floor(min(PtsH(:,2)));% 对第二列最小的数向下取证，即最小的像素位置
i_size = size(ImaRGB(c_px:end,:,1));% 
ImaH = zeros( size(ImaRGB(:,:,1)) );% 初始化一个矩阵用来存储稠密化的深度图
ImaH(c_px:end,:) = fun_dense3D(PtsH,[c_px i_size]); % MEX-file 调用C++程序
idxx=find(ImaH==0); % find all 1.5
ImaH(idxx)=max(max(ImaH)); % set 1 to these indexes
Ima_RangeH= uint8( 255*ImaH/max(max(ImaH)) ); % :)% 转为uint8格式，得到最终稠密化的深度图
% imwrite(Ima_RangeH, 'Height.png');
%求A，不用上采样
ix = pxvelo(:,1)<1;                 pxvelo(ix,:)=[];% 忽略位置小于1的像素
ix = pxvelo(:,1)>size(ImaRGB,2);    pxvelo(ix,:)=[];% 忽略超出RGB范围的像素   
ix = pxvelo(:,2)<1;                 pxvelo(ix,:)=[];% 忽略位置小于1的像素
ix = pxvelo(:,2)>size(ImaRGB,1);    pxvelo(ix,:)=[];% 忽略超出RGB范围的像素   
%Hhalidarimage = NaN(370,1224,3);%初始化这个hha图像，坐标代表hha图像的坐标，三维的内容代表对象雷达点的xyz
Hhalidarimage = NaN(image_size(1),image_size(2),image_size(3));%初始化这个hha图像，坐标代表hha图像的坐标，三维的内容代表对象雷达点的xyz

for i=1:size(pxvelo,1)
   Hhalidarimage(floor(pxvelo(i,2)),floor(pxvelo(i,1)),1) = pxvelo(i,7);
   Hhalidarimage(floor(pxvelo(i,2)),floor(pxvelo(i,1)),2) = pxvelo(i,6);
   Hhalidarimage(floor(pxvelo(i,2)),floor(pxvelo(i,1)),3) = pxvelo(i,5);%756   5深度   7高度 或765
end

[N b1]  = computeNormalsSquareSupport(Hhalidarimage,3,1,size(ImaRGB,2));
yDir = [0,1,0];
angl = acosd(min(1,max(-1,sum(bsxfun(@times, N, reshape(yDir, 1, 1, 3)), 3))));
Ima_RangeA = uint8(angl);

% PtsA = zeros(size(angl,1) * size(angl,2),4);
% for i=1:size(angl,1) 
% 	for j=1:size(angl,2) 
%          PtsA((i - 1) * size(angl,2) + j,1) = i;
%          PtsA((i - 1) * size(angl,2) + j,2) = j;
%          PtsA((i - 1) * size(angl,2) + j,3) = Ima_RangeA(i,j);
% 	end
% end
% imwrite(PtsA, 'Angle.png');
% % -----------------------------------------------------------------------
%上采样D
ix = px(:,1)<1;                 px(ix,:)=[];% 忽略位置小于1的像素
ix = px(:,1)>size(ImaRGB,2);    px(ix,:)=[];% 忽略超出RGB范围的像素   
% figure;% 这里画图的程序是我加的，为了看中间结果
% m3=max(px(:,3));collor=[px(:,3)/m3 px(:,3)/m3 px(:,3)/m3];
% for i=1:length(collor)
%     hold on;
%     plot3(px(i,1),px(i,2),px(i,3),'.','color',collor(i,:));
% end
% xlabel('X');ylabel('Y');zlabel('Z');
% % Ordering
Pts = zeros(size(px,1),4);
Pts = sortrows(px,2);% 按照第二列排序
% % ======================= Interpolation / Upsampling :::
c_px = floor(min(px(:,2)));% 对第二列最小的数向下取证，即最小的像素位置
if c_px<1
    c_px=1;
end
i_size = size(ImaRGB(c_px:end,:,1));% 
Ima3D = zeros( size(ImaRGB(:,:,1)) );% 初始化一个矩阵用来存储稠密化的深度图
% Simply type: mex fun_dense3D.cpp
Ima3D(c_px:end,:) = fun_dense3D(Pts,[c_px i_size]); % MEX-file 调用C++程序
% % -----------------------------------------------------------------------
% Normalization 8 bits
idxx=find(Ima3D==0); % find all 1.5
Ima3D(idxx)=max(max(Ima3D)); % set 1 to these indexes
Ima_RangeD = uint8(255*Ima3D/max(max(Ima3D)) ); % :)% 转为uint8格式，得到最终稠密化的深度图
end %END main Function
