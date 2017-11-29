function bg=get_bg(path)
%path
%depth set - depth set, e.g. dir([path 'depth1*.mat']);

imgmed=zeros(480,640,length(path));

%collect pixel value from all images in dataset
for i=1:length(path)
    load(path(i).depth);
    %figure(4);
    %imagesc(depth_array);
    imgmed(:,:,i)=double(depth_array)/1000;
end
%compute the median
bg=median(imgmed,3);

load(path(1).depth);
bg=double(depth_array)/1000;

