d=dir('*.png');
imgmed=zeros(480,640,length(d));
for i=1:length(d)
    load(['depth1_' int2str(i) '.mat']);
    imagesc(depth_array);
    imgmed(:,:,i)=double(depth_array)/1000;
    pause(.05)
    drawnow;
end
bg=median(imgmed,3);
d2=dir('depth1*.mat');
for i=1:length(d2)
    load(d2(i).name);
    imagesc(abs(double(depth_array)/1000-bg)>.25);
    colormap(gray);
    pause(0.5);
end
    