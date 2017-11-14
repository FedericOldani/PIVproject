clear;
clc;

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

%eliminate bg for all images in dataset
%for i=1:length(d)
i=5
    fprintf('%d...',i);
    
    % ----------------------   objects in image1   ----------------------
    
    depdir=dir([path 'depth1*.mat']);
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

i=1;
    
    obj3d=xyz1(find(L1~=0),:);
    
    %some optimization on the object
        obj3d( ~any(obj3d,2), : ) = [];  %delete 0 value rows
        for v=1:obj3d(end)
            cont=0;
            for u=1:obj3d(end)
                if u~=v
                    cont++;
                    err=abs(obj3d(u,v,3));
                end
            end
            err=err/cont;
            if(err
        end
    hold on;
    plot3(obj3d(:,1),obj3d(:,2),obj3d(:,3),'.','MarkerSize',10); hold on;
    
    %[s,corner,s,s]=minboundbox(obj3d(:,1),obj3d(:,2),obj3d(:,3));
    
    %plot3(xyz1(:,1),xyz1(:,2),xyz1(:,3),'.','MarkerSize',10); hold on;
    %plotminbox(corner);
    %hold on;
    %%
    

    obj3d=xyz21(find(L2==1),:);
    obj3d( ~any(obj3d,2), : ) = [];  %delete 0 value rows
    plot3(obj3d(:,1),obj3d(:,2),obj3d(:,3),'.','MarkerSize',10); hold on;

    %plot3(obj3d_2(:,1),obj3d_2(:,2),obj3d_2(:,3),'.','MarkerSize',10); hold on;

    %[s,corner,s,s]=minboundbox(obj3d(:,1),obj3d(:,2),obj3d(:,3));

%     dep_test2=dep2;
%     dep_test2(find(L2~=1))=0;
%     xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);% end
%     xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
    