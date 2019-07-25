function [] = downsampleTest(ptCloudRef,mode)
if mode==1
    ptCloudA=pcdownsample(ptCloudRef,'random',0.1);
elseif mode==2
    ptCloudA=pcdownsample(ptCloudRef,'gridAverage',1);
elseif mode==3%nonuniform box grid filter
    ptCloudA=pcdownsample(ptCloudRef,'nonuniformGridSample',20);
end
%ptCloudB=pcdownsample(ptCloudRef,mode,0.3);
%ptCloudC=pcdownsample(ptCloudRef,mode,0.6);
%ptCloudD=pcdownsample(ptCloudRef,mode,0.8);
%ptCloudE=pcdownsample(ptCloudRef,mode,0.5);
figure;
subplot(1,2,1);
pcshow(ptCloudRef);
subplot(1,2,2);
pcshow(ptCloudA);
%subplot(2,2,3);
%pcshow(ptCloudB);
%subplot(2,2,4);
%pcshow(ptCloudC);

end

