clear;
clc;
close all;

load('calib_asus.mat');
path='maizena4/data_rgb/';

bg1=get_bg(path, 'depth1*.mat');
bg2=get_bg(path, 'depth2*.mat');


% -------------------------compute T and R   --------------------------

im1=imread([path 'rgb_image1_01.png']);
im2=imread([path 'rgb_image2_01.png']);
load([path 'depth1_01.mat'])
dep1=depth_array;
load([path 'depth2_01.mat'])
dep2=depth_array;

tr=rt_computation(im1,dep1,im2,dep2);

depdir=dir([path 'depth1*.mat']);

%eliminate bg for all images in dataset
for i=1:length(depdir)
    %i=9;
    
    close all;
    
    fprintf('%d...',i);
    
    % ----------------------  get objects in image1   ----------------------
    
    depdir=dir([path 'depth1*.mat']);
    path1=[path depdir(i).name];
    load(path1);
    dep1=depth_array;
    obj1=remove_bg(dep1, bg1);
    %objects1=bwpropfilt(obj1,'EulerNumber',[-30 1]);%remove region with holes
    [L1, num1]=bwlabel(obj1,8);
    
    
    
    % ----------------------  get objects in image2   ----------------------
    
    depdir=dir([path 'depth2*.mat']);
    path2=[path depdir(i).name];
    load(path2);
    dep2=depth_array;
    obj2=remove_bg(dep2,bg2);
    %objects2=bwpropfilt(obj2,'EulerNumber',[-30 1]);%remove region with holes
    [L2, num2]=bwlabel(obj2,8);
    
    % ----------------------   point cloud   ---------------------
    
    imdir=dir([path 'rgb_image1_*.png']);
    im1=imread([path imdir(i).name]);
    
    imdir=dir([path 'rgb_image2_*.png']);
    im2=imread([path imdir(i).name]);
    
    dep1(find(dep1>2000))=0;%eliminate too far objects
    dep2(find(dep2>2000))=0;%eliminate too far objects
    
    %get coordinate in a unique reference system
    [xyz1,rgbd1,xyz21,rgbd2]=getCoordinate(dep1,im1,dep2,im2,tr);
    pc1=getPointCloud(xyz1,rgbd1);
    pc2=getPointCloud(xyz21,rgbd2);
    
    obj3d_1=getObj3d(xyz1,L1);
    obj3d_2=getObj3d(xyz21,L2);
    
    obj3d=[obj3d_1; obj3d_2]; %put all points coordinates together
   
   % ------------------------ search objects in 3d ------------------------
   
   % 1.knn search to search for nearest neighbour of each point
   % 2.load a sparse distance matrix with values found (only if distane is
   %    below a certain limit)
   % 3.build a graph (different objects are not connected)
   % 4.each separate subgraph represent an object
    
    [kn,D] = knnsearch(obj3d,obj3d,'k',15);%default 10
    
    idx=0;
    if ~isempty(kn)
        DG=sparse(length(kn(:,1)),length(kn(:,1)),0);
        for h=2:length(kn(1,:))
            DG=DG+sparse(kn(:,1),kn(:,h),D(:,h)<0.05,length(kn(:,1)),length(kn(:,1)));
        end
        G=graph(DG,'upper');
        bins = conncomp(G);
        idx=bins;
    end
    
    %{ 
        consider this section as a previous version of objects detection
    
    
    %
    % group=round(length(obj3d)/10);
    % %if length(obj3d)>group %if there is no objects it won't create cluster
    %     tic;
    %     [idx,cc]=kmeans(obj3d,group);
    %     toc;
    %     group=max(idx);
    %
    % for w=1:group
    %         l=length(find(idx==w));
    %         if l<group/300
    %             idx(find(idx==w))=0;
    %         end
    % end
    %
    %
    %     distance=10; %default 12
    %     idxcc=clusterdata(cc,distance);
    %
    %     % go back to the original image and set the right index to the right object
    %      for w=1:group
    %          idx(find(idx==w))=idxcc(w)+100000;
    %      end
    %     idx=idx-100000;
    %
    %  for w=1:group
    %         l=length(find(idx==w));
    %         if l<500
    %             idx(find(idx==w))=0;
    %         end
    % end
    %
    % obj3d(idx==0,:)=[];
    %     tic;
    %     [idx,cc]=kmeans(obj3d,max(num1,num2));
    %     toc;
    %     group=max(idx);
    
    %print image pointcloud, print objects inside boxes
    
    %}
    
    pcshow(pcmerge(pc1,pc2,0.001));
    for n=1:max(idx)
        hold on;
        if length(find(idx==n))>1000
            fprintf("printing>");
            plot3(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3),'.','MarkerSize',10); hold on;
            p=corner3d(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3));
            plot_box(p);
            hold on;
        end
    end
    pause(4);
    %  else
    %      fprintf('\n no objects detected in the scene\n');
    %  end
    
end
