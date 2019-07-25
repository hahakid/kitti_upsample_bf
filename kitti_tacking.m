clear; dbstop error; clc;
warning off; close all;
data_path = 'H:/kitti/tracking/training/';
save_path = 'H:/kitti/tracking/training/upsample/bf/';

for seq_idx=0:0 %20

c_save_dir=fullfile(save_path,sprintf('%04d/',seq_idx));
if ~exist(c_save_dir,'dir')
   mkdir(c_save_dir);
end

base_dir = [data_path,sprintf('velodyne/%04d', seq_idx)];
calib_dir=[data_path,'calib/'];
path_ima = [data_path,sprintf('image_02/%04d', seq_idx)];

% % % load data
calib = dir([calib_dir,sprintf('%04d.txt', seq_idx)]);   %7518 files

nt_frames = length(dir(fullfile(base_dir, '*.bin')));
fst_frame = 0;
% %------------------------------------------------------------------------
for frame = fst_frame: 1: nt_frames-1

ima   = [path_ima,sprintf('/%06d.png', frame)]; 
% tic
fd = fopen(ima);
if fd < 1
   fprintf('Cound not open RGB image !!!\n');    keyboard
else
    ImaRGB = imread(ima);
    %path_ima = [data_path,'deepimage/'];
    %ImaRGB = imread( [path_ima,ima(frame+1).name] );
end
fclose(fd);
tic;

[ImaRangeD,ImaRangeH,ImaRangeA] = Fun_dense_range_map(calib,calib_dir,base_dir,frame,ImaRGB);%³íÃÜ»¯
%[ImaRangeD,ImaRangeH,ImaRangeA] = Fun_dm(calib(frame+1),calib_dir,base_dir,frame,ImaRGB);%³íÃÜ»¯

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
imwrite(combineimage,[c_save_dir,sprintf('/%06d.png', frame)]);
fprintf('%d finish;\n',frame)
end

end

