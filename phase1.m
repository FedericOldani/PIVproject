clear;
clc;
close all;

load('calib_asus.mat');
path='maizena3/data_rgb/';

bg1=get_bg(path, 'depth1*.mat');
bg2=get_bg(path, 'depth2*.mat');


% -------------------------compute T and R   --------------------------
    
im1=imread([path 'rgb_image1_3.png']);
im2=imread([path 'rgb_image2_3.png']);
load([path 'depth1_3.mat'])
dep1=depth_array;
load([path 'depth2_3.mat'])
dep2=depth_array;

tr=rt_computation(im1,dep1,im2,dep2);

depdir=dir([path 'depth1*.mat']);

%eliminate bg for all images in dataset
%for i=1:length(depdir)
i=10;%problem with 6,7
    fprintf('%d...',i);
    
    % ----------------------   objects in image1   ----------------------
    
    
    load([path depdir(i).name]);
    dep1=depth_array;
    objects1=remove_bg(dep1,bg1);
    
    [L1, num1]=bwlabel(objects1);
    
    
    % ----------------------   objects in image2   ----------------------
    
    depdir=dir([path 'depth2*.mat']);
    load([path depdir(i).name]);
    dep2=depth_array;
    objects2=remove_bg(dep2,bg2);
    
    [L2, num2]=bwlabel(objects2);

    
    
    % ----------------------   point cloud   ---------------------
        
    imdir=dir([path 'rgb_image1_*.png']);
    im1=imread([path imdir(i).name]);
    
    imdir=dir([path 'rgb_image2_*.png']);
    im2=imread([path imdir(i).name]);
    
    depdir=dir([path 'depth1_*.mat']);
    load([path depdir(i).name]);
    dep1=depth_array;
    dep1(find(depth_array>2000))=0;

    depdir=dir([path 'depth2_*.mat']);
    load([path depdir(i).name]);
    dep2=depth_array;
    dep2(find(depth_array>2000))=0;

    
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz21,'Color',reshape(rgbd2,[480*640 3]));
    %figure(7);
    %showPointCloud(pc1)
    figure(3);
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;
    
pause(0.5);



%% TEST


    
    obj3d_1=xyz1(find(L1~=0),:);
    obj3d_1 = unique(obj3d_1,'rows');

    %%
    

    obj3d_2=xyz21(find(L2~=0),:);
    obj3d_2 = unique(obj3d_2,'rows');
   


%%
obj3d=[obj3d_1; obj3d_2];


%clusterdata is the perfect function to detect objects but it takes too
%long! Hence, let's simplify the problem. We use kmeans to divided the 3d
%in 2000 clusters, then we use cluterdata to group nearest clusters.

group=2000;
tic;
[idx,cc]=kmeans(obj3d,group);
toc;
group=max(idx);

%default 8
distance=8;
idxcc=clusterdata(cc,distance);
toc;

%%
% 
 for w=1:group
     idx(find(idx==w))=idxcc(w)+group;
 end
idx=idx-group;

for i=1:group
    l=length(find(idx==i));
    if l<2000
        idx(find(idx==i))=0;
    end
end


%%
 pcshow(pcmerge(pc1,pc2,0.001));
for n=1:max(idx)
    hold on;
    if length(find(idx==n))~=0 
    plot3(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3),'.','MarkerSize',10); hold on;
        p=corner3d(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3));
        plot_box(p);
       fprintf("printing>");
    hold on;
    end
end

