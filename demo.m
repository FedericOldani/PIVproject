clear;
clc;
close all;

path='lab1/';

%depth path
depdir1=dir([path 'depth1*.mat']);
imdir1=dir([path 'rgb_image1_*']);

depdir2=dir([path 'depth2*.mat']);
imdir2=dir([path 'rgb_image2_*']);

imgseq1=struct('depth',{},'rgb',{});
imgseq2=struct('depth',{},'rgb',{});


for i=1:length(depdir1)
    
    d=[path depdir1(i).name];
    im=[path imdir1(i).name];
    
    imgseq1(i).depth=d;
    imgseq1(i).rgb=im;
    
    d=[path depdir2(i).name];
    im=[path imdir2(i).name];
    
    imgseq2(i).rgb=im;
    imgseq2(i).depth=d;
end


load('calib_asus.mat');
cam_params.Kdepth=Depth_cam.K;
cam_params.Krgb=RGB_cam.K;
cam_params.R=R_d_to_rgb;
cam_params.T=T_d_to_rgb;
%compute T and R


im1=imread(imgseq1(1).rgb);
im2=imread(imgseq2(1).rgb);
load(imgseq1(1).depth);
dep1=depth_array;
load(imgseq2(1).depth);
dep2=depth_array;
tr=rt_computation(im1,dep1,im2,dep2);

%cam1toW=struct('R',eye(3),'T',zeros(3,1));
%cam2toW=struct('R',tr.T','T',tr.c(1,:)');%tr.T is the rotation matrix, %tr.c is the translation



% objects = track3D_part1( imgseq1, imgseq2,   cam_params,  cam1toW, cam2toW);
[objects, cam1toW, cam2toW] = track3D_part2( imgseq1, imgseq2,   cam_params);


%% test
close all;
i=17;
for k=1:length(objects(i).frames_tracked)
    figure(1);
        middle(1)=sum(objects(i).X(k,:))/8;
        middle(2)=sum(objects(i).Y(k,:))/8;
        middle(3)=sum(objects(i).Z(k,:))/8;
        plot3(middle(1),middle(2),middle(3),'.');hold on;
        figure(2);
    im1 = imread( imgseq1(objects(i).frames_tracked(k)).rgb);
    imshow(im1)
    pause(1);
end


