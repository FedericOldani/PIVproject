function [objects, cam1toW, cam2toW] = track3D_part2( imgseq1, imgseq2,   cam_params)

%Initializations
i=1;
niter=200; 
errorthresh=0.1;

%Load depth
load(imgseq1(i).depth);
dep1=depth_array;
load(imgseq2(i).depth);
dep2=depth_array;

%Load images
im1=imread(imgseq1(i).rgb);
im2=imread(imgseq2(i).rgb); 

%Image parameters matched in xyz and rgb
[xyz1,rgbd1]=ImageParameters(dep1,im1,cam_params);
[xyz2,rgbd2]=ImageParameters(dep2,im2,cam_params);

%Matching key features
rgbd1aux = single(rgb2gray(rgbd1));
rgbd2aux = single(rgb2gray(rgbd2));
[fa, da] = vl_sift(rgbd1aux,'edgethresh',30,'PeakThresh',0);
[fb, db] = vl_sift(rgbd2aux,'edgethresh',30,'PeakThresh',0);
[matches, scores] = vl_ubcmatch(da, db);

%Get coordenates of those matches
xa=fa(1,matches(1,:)); xa=fix(xa); x1=xa';
xb=fb(1,matches(2,:)); xb=fix(xb); x2=xb';
ya=fa(2,matches(1,:)); ya=fix(ya); y1=ya';
yb=fb(2,matches(2,:)); yb=fix(yb); y2=yb';

%RANSAC
tr=ransactr(xyz1, x1, y1, xyz2, x2, y2, niter, errorthresh);

%Transformations
cam1toW=struct('R',eye(3),'T',zeros(1,3));
cam2toW=struct('R',tr.T,'T',tr.c);
objects = track3D_part1( imgseq1, imgseq2,   cam_params,  cam1toW, cam2toW);

