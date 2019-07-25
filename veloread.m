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
