clear; dbstop error; clc;
warning off; close all;
data_path = 'H:/kitti/object/training/';
save_path = 'H:/kitti/object/training/upsample/bf/';
base_dir = [data_path,'velodyne'];
calib_dir = [data_path,'calib/'];
path_ima = [data_path,'image_2/'];
% % % load data
calib = dir([calib_dir,'*.txt']);   %7518 files
ima   = dir([path_ima,'*.png']);    %7518 files
fst_frame = 0; nt_frames = 7481;%7517; % 0 的维度和所有图像都不匹配
% %------------------------------------------------------------------------
for frame = fst_frame: 1: nt_frames
% tic
fd = fopen( [path_ima,ima(frame+1).name] );
if fd < 1
   fprintf('Cound not open RGB image !!!\n');    keyboard
else
    ImaRGB = imread( [path_ima,ima(frame+1).name] );
    %path_ima = [data_path,'deepimage/'];
    %ImaRGB = imread( [path_ima,ima(frame+1).name] );
end
fclose(fd);
tic;

[ImaRangeD,ImaRangeH,ImaRangeA] = Fun_dense_range_map(calib(frame+1),calib_dir,base_dir,frame,ImaRGB);%稠密化
%[ImaRangeD,ImaRangeH,ImaRangeA] = Fun_dm(calib(frame+1),calib_dir,base_dir,frame,ImaRGB);%稠密化

toc;
% imshow(ImaRange)
%imwrite(ImaRange,data_path,'jpeg');
%print(ImaRange,'-djpeg','C:/Image1.jpg'); 
%print(1,'-dpng',[data_path,'deepimage/',num2str(frame),'.png']) ;
%combineimage= uint8(zeros(size(ImaRGB)));
%combineimage(:,:,1)=ImaRangeD;
%combineimage(:,:,2)=ImaRangeA;
%combineimage(:,:,3)=ImaRangeH;

combineimage=cat(3,ImaRangeD,ImaRangeH,ImaRangeA);% 3 channel combined
%image(combineimage);
%imwrite(ImaRangeD,[data_path,'deepimage/',ima(frame+1).name],'png');%','mode','lossless'
%imwrite(ImaRangeH,[data_path,'heightimage/',ima(frame+1).name],'png');%','mode','lossless'
%imwrite(ImaRangeA,[data_path,'angleimage/',ima(frame+1).name],'png');%','mode','lossless'
imwrite(combineimage,[save_path,ima(frame+1).name],'png');
fprintf('%d finish;\n',frame)
end



