function [Ima_RangeD,Ima_RangeH,Ima_RangeA] = Fun_dense_range_map(calib,calib_dir,base_dir,...
    frame,ImaRGB)
image_size=size(ImaRGB);
T = Fun_open_calib(calib.name,calib_dir);
% T = Fun_open_calib(calib(frame+1).name,calib_dir);%
% ������Ҽӵģ�Ϊ���ǿ��Ե�������Ĵ��룬��ȡ��frame���״��ļ�

fd = fopen(sprintf('%s/%06d.bin',base_dir,frame),'rb');% ���״�������ļ�
if fd < 1
    fprintf('No LIDAR files !!!\n');
    keyboard
else
velo = fread(fd,[4 inf],'single')';% ��ȡ�״����ݣ���Ϊn*4�ľ���
%figure;plot3(velo(:,1),velo(:,2),velo(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('Original Velo');
% ���ﻭͼ�ĳ������Ҽӵģ�Ϊ�˿��м���
fclose(fd);
end

% remove all points behind image plane (approximation)
idx = velo(:,1)<-3;% ԭֵ=5�����䵽-3�������������ಿ�ֿհ�����
velo(idx,:) = [];
%figure;plot3(px(:,1),px(:,2),px(:,3),'.');% ���ﻭͼ�ĳ������Ҽӵģ�Ϊ�˿��м���

% project to image plane (exclude luminance)
px = (T.P2 * T.R0_rect * T.Tr_velo_to_cam * velo')';% ��ת��RGBƽ�棬��ʽ�ο�KITTI��������У���ļ���ʹ�õ�readme
px(:,1) = px(:,1)./px(:,3);%1,2��ͼ�����ص�xy���꣬3�����
px(:,2) = px(:,2)./px(:,3);
%figure;plot3(px(:,1),px(:,2),px(:,3),'.');xlabel('X');ylabel('Y');zlabel('Z');title('project to image plane');
% ���ﻭͼ�ĳ������Ҽӵģ�Ϊ�˿��м���

%20170831���ӵ�hha����------------------------------------------------------------------
pxvelo = [px,velo];%px(:,1)���״��Ӧ���ͼ��������꣬px(:,2)���״��Ӧ���ͼ��������꣬px(:,3)���״��Ӧ���ͼ�����ȣ�velo�Ǹõ��״���ά����
%�ϲ���H
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
ix = PtsH(:,1)<1;                 PtsH(ix,:)=[];% ����λ��С��1������
ix = PtsH(:,1)>size(ImaRGB,2);    PtsH(ix,:)=[];% ���Գ���RGB��Χ������   
ix = PtsH(:,2)<1;                 PtsH(ix,:)=[];% ����λ��С��1������
ix = PtsH(:,2)>size(ImaRGB,1);    PtsH(ix,:)=[];% ���Գ���RGB��Χ������   
%ix = PtsH(:,4)<1;                 PtsH(ix,:)=[];% ����λ��С��1������
%ix = PtsH(:,4)>600;    PtsH(ix,:)=[];% ���Գ���   
PtsH = sortrows(PtsH,2);% ���յڶ�������
c_px = floor(min(PtsH(:,2)));% �Եڶ�����С��������ȡ֤������С������λ��
i_size = size(ImaRGB(c_px:end,:,1));% 
ImaH = zeros( size(ImaRGB(:,:,1)) );% ��ʼ��һ�����������洢���ܻ������ͼ
ImaH(c_px:end,:) = fun_dense3D(PtsH,[c_px i_size]); % MEX-file ����C++����
idxx=find(ImaH==0); % find all 1.5
ImaH(idxx)=max(max(ImaH)); % set 1 to these indexes
Ima_RangeH= uint8( 255*ImaH/max(max(ImaH)) ); % :)% תΪuint8��ʽ���õ����ճ��ܻ������ͼ
% imwrite(Ima_RangeH, 'Height.png');
%��A�������ϲ���
ix = pxvelo(:,1)<1;                 pxvelo(ix,:)=[];% ����λ��С��1������
ix = pxvelo(:,1)>size(ImaRGB,2);    pxvelo(ix,:)=[];% ���Գ���RGB��Χ������   
ix = pxvelo(:,2)<1;                 pxvelo(ix,:)=[];% ����λ��С��1������
ix = pxvelo(:,2)>size(ImaRGB,1);    pxvelo(ix,:)=[];% ���Գ���RGB��Χ������   
%Hhalidarimage = NaN(370,1224,3);%��ʼ�����hhaͼ���������hhaͼ������꣬��ά�����ݴ�������״���xyz
Hhalidarimage = NaN(image_size(1),image_size(2),image_size(3));%��ʼ�����hhaͼ���������hhaͼ������꣬��ά�����ݴ�������״���xyz

for i=1:size(pxvelo,1)
   Hhalidarimage(floor(pxvelo(i,2)),floor(pxvelo(i,1)),1) = pxvelo(i,7);
   Hhalidarimage(floor(pxvelo(i,2)),floor(pxvelo(i,1)),2) = pxvelo(i,6);
   Hhalidarimage(floor(pxvelo(i,2)),floor(pxvelo(i,1)),3) = pxvelo(i,5);%756   5���   7�߶� ��765
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
%�ϲ���D
ix = px(:,1)<1;                 px(ix,:)=[];% ����λ��С��1������
ix = px(:,1)>size(ImaRGB,2);    px(ix,:)=[];% ���Գ���RGB��Χ������   
% figure;% ���ﻭͼ�ĳ������Ҽӵģ�Ϊ�˿��м���
% m3=max(px(:,3));collor=[px(:,3)/m3 px(:,3)/m3 px(:,3)/m3];
% for i=1:length(collor)
%     hold on;
%     plot3(px(i,1),px(i,2),px(i,3),'.','color',collor(i,:));
% end
% xlabel('X');ylabel('Y');zlabel('Z');
% % Ordering
Pts = zeros(size(px,1),4);
Pts = sortrows(px,2);% ���յڶ�������
% % ======================= Interpolation / Upsampling :::
c_px = floor(min(px(:,2)));% �Եڶ�����С��������ȡ֤������С������λ��
if c_px<1
    c_px=1;
end
i_size = size(ImaRGB(c_px:end,:,1));% 
Ima3D = zeros( size(ImaRGB(:,:,1)) );% ��ʼ��һ�����������洢���ܻ������ͼ
% Simply type: mex fun_dense3D.cpp
Ima3D(c_px:end,:) = fun_dense3D(Pts,[c_px i_size]); % MEX-file ����C++����
% % -----------------------------------------------------------------------
% Normalization 8 bits
idxx=find(Ima3D==0); % find all 1.5
Ima3D(idxx)=max(max(Ima3D)); % set 1 to these indexes
Ima_RangeD = uint8(255*Ima3D/max(max(Ima3D)) ); % :)% תΪuint8��ʽ���õ����ճ��ܻ������ͼ
end %END main Function
