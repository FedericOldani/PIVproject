function bg=get_bg(path, depth_set )
%path
%depth set - depth set, e.g. dir([path 'depth1*.mat']);


d=dir([path depth_set]);
imgmed=zeros(480,640,length(d));

%collect pixel value from all images in dataset
for i=1:length(d)
    load([path d(i).name]);
    %figure(4);
    %imagesc(depth_array);
    imgmed(:,:,i)=double(depth_array)/1000;
end

%compute the median
bg=median(imgmed,3);
