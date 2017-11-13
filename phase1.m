clear;
clc;

path='maizena3/data_rgb/';
d=dir([path 'depth1*.mat']);

imgmed1=zeros(480,640,length(d));
imgmed2=zeros(480,640,length(d));

%collect pixel value from all images in dataset
for i=1:length(d)
    load([path 'depth1_' int2str(i) '.mat']);
    
    %figure(4);
    %imagesc(depth_array);
    
    imgmed1(:,:,i)=double(depth_array)/1000;
    load([path 'depth2_' int2str(i) '.mat']);
    
    %figure(5);
    %imagesc(depth_array);
    imgmed2(:,:,i)=double(depth_array)/1000;
    
    pause(0.5);
end

%compute the median
bg1=median(imgmed1,3);
bg2=median(imgmed2,3);

% -------------------------compute T and R   --------------------------
    
load('calib_asus.mat');
im1=imread([path 'rgb_image1_3.png']);
im2=imread([path 'rgb_image2_3.png']);
load([path 'depth1_3.mat'])
dep1=depth_array;
load([path 'depth2_3.mat'])
dep2=depth_array;
%dep2(find(dep2(:)>4000))=0;
xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
%REGISTER RGB TO DEPTH
rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
% figure(1);imagesc(rgbd1 );
% figure(2);imagesc(rgbd2 );
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));
% figure(3);clf; showPointCloud(pc1);
% figure(4);clf; showPointCloud(pc2);

np=6;
x1=zeros(np,1); y1=x1; x2=y1; y2=x1;
x1=[178 280 169 268 501 467];
y1=[306 303 211 204 218 243];
x2=[169 277 174 286 383 358];
y2=[193 185 68 71 108 133];

ind1=sub2ind(size(dep1),y1,x1);
ind2=sub2ind(size(dep2),y2,x2);

P1=xyz1(ind1,:);
P2=xyz2(ind2,:);
inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(inds,:);P2=P2(inds,:);
[dd,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz21,'Color',reshape(rgbd2,[480*640 3]));
% figure(1);clf; showPointCloud(pc1);
% figure(2);clf; showPointCloud(pc2);
% pause(1);

%eliminate bg for all images in dataset
%for i=1:length(d)
i=5
    fprintf('%d...',i);
    
    % ----------------------   objects in image1   ----------------------
    
    depdir=dir([path 'depth1*.mat']);
    load([path depdir(i).name]);
    absimage=abs((double(depth_array)/1000-bg1))>.25; %pick objects which differ at least 25cm from bg
    objects1=bwareaopen(absimage,2500); %pick objects with more than 2500px

    %figure(7);
    %imagesc(objects1);
    %pause(0.5);
    
    %create a depth array with only objects
    dep1=depth_array;
    dep1(find(dep1(:)>3000))=0;
    dep1(find(objects1(:)==0))=0;
    
    % ----------------------   objects in image2   ----------------------
    
    depdir=dir([path 'depth2*.mat']);
    load([path depdir(i).name]);
    absimage=abs((double(depth_array)/1000-bg2))>.25; %pick objects which differ at least 25cm from bg
    objects2=bwareaopen(absimage,2500); %pick objects with more than 2500px
    
    %figure(2);
    %imagesc(objects2);
    %pause(0.5);
    
    %create a depth array with only objects
    dep2=depth_array;
    dep2(find(dep2(:)>3000))=0;
    dep2(find(objects2(:)==0))=0;
    
    % ----------------------   point cloud   ---------------------
        
    imdir=dir([path 'rgb_image1_*.png']);
    im1=imread([path imdir(i).name]);
    
    imdir=dir([path 'rgb_image2_*.png']);
    im2=imread([path imdir(i).name]);
    
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    
    %REGISTER RGB TO DEPTH
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);

    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
    pc2=pointCloud(xyz21,'Color',reshape(rgbd2,[480*640 3]));
    figure(7);hold off;
    showPointCloud(pc1)
    figure(3);
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;
    
    
pause(0.5);
%%
    xyz_tot=imfuse(xyz1,xyz21);
    
    [L,n]= bwlabeln(xyz_tot,4);

    
% end
    
    
    