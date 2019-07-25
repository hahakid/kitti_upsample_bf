function [] = multishow(ptlist,layout)

if length(ptlist)~=layout(1)*layout(2)
    disp 'error number';
else
    figure;
    count=1;
    for i=1:1:layout(1)
        for j=1:1:layout(2)
            subplot(layout(1),layout(2),count);
            %(i-1)*layout(1)+j
            %ptlist{(i-1)*layout(1)+j};
            pcshow(ptlist{(i-1)*layout(1)+j});
            count=count+1;
        end
    end
end

