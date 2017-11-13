
path='maizena/data_rgb/';
d=dir([path 'depth1*.mat']);

imgmed=zeros(480,640,length(d));
for i=1:length(d)
    load([path 'depth1_' int2str(i) '.mat']);
    imagesc(depth_array);
    imgmed(:,:,i)=double(depth_array)/1000;
    pause(.1)
    drawnow;
end
bg=median(imgmed,3);
for i=1:length(d)
    load([path d(i).name]);
    absimage=abs((double(depth_array)/1000-bg))>.25;
    objects1=bwareaopen(absimage,2500);
    imagesc(objects1);
    
     %create a depth array with only objects
    dep1=depth_array;
    for q=1:480
        for w=1:640
            if(objects1(q,w)==0)
                dep1(q,w)=0;
            end
        end
    end
    
    colormap(winter);
    pause(0.5);
    %its better compute bwlabel in pointcloud
    [L, num]=bwlabel(objects1);
   
end

%% compute same as before for the second image
path='maizena/data_rgb/';
d=dir([path 'depth2*.mat']);

imgmed=zeros(480,640,length(d));
for i=1:length(d)
    load([path 'depth2_' int2str(i) '.mat']);
    imagesc(depth_array);
    imgmed(:,:,i)=double(depth_array)/1000;
    pause(.1)
    drawnow;
end
bg=median(imgmed,3);

imdir=dir([path 'rgb_image2_']);

for i=1:length(d)
    load([path d(i).name]);
    absimage=abs((double(depth_array)/1000-bg))>.25;
    objects2=bwareaopen(absimage,2500);
    imagesc(objects2);
    
    %create a depth array with only objects
    dep2=depth_array;
    for q=1:480
        for w=1:640
            if(objects2(q,w)==0)
                dep2(q,w)=0;
            end
        end
    end
    colormap(winter);
    pause(0.5);
    %its better compute bwlabel in pointcloud
    [L, num]=bwlabel(objects2);
   
end

%% point cloud for objects only
path='maizena/data_rgb/';
load('calib_asus.mat');
d=dir([path 'rgb_image1_*']);
for i=1:length(d),
    im1=zeros(480,640,3);
    im1=im1 + objects1;
    
    im2=zeros(480,640,3);
    im2 = im2 + objects2;
    %dep1, dep2 already created before
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    figure(1);hold off;
    %showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;
 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% READ IMAGES and GENERATE POINT CLOUDS
path='maizena3/data_rgb/';

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
figure(1);imagesc(rgbd1 );
figure(2);imagesc(rgbd2 );
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));
figure(3);clf; showPointCloud(pc1);
figure(4);clf; showPointCloud(pc2);
%GET CORRESPONDING POINTS
np=6;
figure(1);x1=zeros(np,1);y1=x1;x2=y1;y2=x1;
x1=[178 280 169 268 501 467];
y1=[306 303 211 204 218 243];
x2=[169 277 174 286 383 358];
y2=[193 185 68 71 108 133];
% for i=1:np,
%     figure(1);
%     [xa ya]=ginput(1);text(xa,ya,int2str(i));
%     xa=fix(xa);ya=fix(ya);
%     x1(i)=xa;y1(i)=ya;
%     aux1=xyz1(sub2ind([480 640],ya,xa),:);
%     figure(3);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
%     hold off;
%     figure(2);
%     [xa ya]=ginput(1);text(xa,ya,int2str(i));
%     xa=fix(xa);ya=fix(ya);
%   x2(i)=xa;y2(i)=ya;
%     aux1=xyz2(sub2ind([480 640],fix(ya),fix(xa)),:);
%     figure(4);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
%     hold off;
%     drawnow;
% end
 
%%
figure(1);hold on; plot(x1,y1,'*r');hold off;
figure(2);hold on;plot(x2,y2,'*r');hold off;
ind1=sub2ind(size(dep1),y1,x1);
ind2=sub2ind(size(dep2),y2,x2);
%%
P1=xyz1(ind1,:);
P2=xyz2(ind2,:);
inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(inds,:);P2=P2(inds,:);
[d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz21,'Color',reshape(rgbd2,[480*640 3]));
figure(1);clf; showPointCloud(pc1);
figure(2);clf; showPointCloud(pc2);
pause(1);
%% 
%SHOW ALL CLOUDS FUSING
d=dir([path 'rgb_image1_*']);
%for i=1:length(d),
i=5;
    im1=imread([path 'rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread([path 'rgb_image2_' d(i).name(12:end-3) 'png']);
    load([path 'depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    load([path 'depth2_' d(i).name(12:end-3) 'mat'])
    dep2=depth_array;
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    figure(1);hold off;
    showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;
 %end