clear; clc; close all;

load('calib_asus.mat');
path='corredor1/data_rgb/';

nameimage1='rgb_image1_232.png';
nameimage2='rgb_image2_232.png';
namedepth1='depth1_232.mat';
namedepth2='depth2_232.mat';
    
im1=imread([path nameimage1]);
im2=imread([path nameimage2]);

load([path namedepth1])
dep1=depth_array;
load([path namedepth2])
dep2=depth_array;


%-----------------------------Image 1

figure(1);
image(im1);
im1 = single(rgb2gray(im1));
[f,d] = vl_sift(im1,'edgethresh',30,'PeakThresh',0);

perm = randperm(size(f,2));
sel = perm(1:50);
h1 = vl_plotframe(f(:,sel));
h2 = vl_plotframe(f(:,sel));
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);

h3 = vl_plotsiftdescriptor(d(:,sel),f(:,sel));
set(h3,'color','g');


%----------------------Image 2

figure(2);
image(im2);
im2 = single(rgb2gray(im2));
[f,d] = vl_sift(im2,'edgethresh',30,'PeakThresh',0);

perm = randperm(size(f,2));
sel = perm(1:50);
h1 = vl_plotframe(f(:,sel));
h2 = vl_plotframe(f(:,sel));
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);

h3 = vl_plotsiftdescriptor(d(:,sel),f(:,sel));
set(h3,'color','g');


%-----------------------Matching images

[fa, da] = vl_sift(im1,'edgethresh',30,'PeakThresh',0);
[fb, db] = vl_sift(im2,'edgethresh',30,'PeakThresh',0);
[matches, scores] = vl_ubcmatch(da, db);


%------------------------Plot common points

clear im1 im2;
im1=imread([path nameimage1]);
im2=imread([path nameimage2]);
figure(3); image(cat(2, im1, im2));

xa = fa(1,matches(1,:));
xb = fb(1,matches(2,:));
ya = fa(2,matches(1,:));
yb = fb(2,matches(2,:));
xbaux = fb(1,matches(2,:)) + size(im1,2);

hold on;
h = line([xa ; xbaux], [ya ; yb]);
set(h,'linewidth', 1, 'color', 'b');

vl_plotframe(fa(:,matches(1,:)));
fbaux=fb;
fbaux(1,:) = fb(1,:) + size(im1,2);
vl_plotframe(fbaux(:,matches(2,:)));
axis image off;


%%
%%--------------------Testing

%clear; 

% load('calib_asus.mat');
% path='corredor1/data_rgb/';
% % nameimage1='rgb_image1_232.png';
% % nameimage2='rgb_image2_232.png';
% % namedepth1='depth1_232.mat';
% % namedepth2='depth2_232.mat';
% im1=imread([path nameimage1]);
% im2=imread([path nameimage2]);
% 
% load([path namedepth1])
% dep1=depth_array;
% load([path namedepth2])
% dep2=depth_array;
tic
clear; clc; close all;

path='tworoooms/data_rgb/';

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

%--------------------Matching with rgbd

numMatches=[];
for i=1:length(imgseq1)

    load(imgseq1(i).depth);
    dep1=depth_array;
    load(imgseq2(i).depth);
    dep2=depth_array;
    im1=imread(imgseq1(i).rgb);
    im2=imread(imgseq2(i).rgb); 
    
 %   dep1(find(dep1>4000))=0; dep2(find(dep2>4000))=0; %eliminate objects that are too far away

    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd1aux = single(rgb2gray(rgbd1));
    rgbd2aux = single(rgb2gray(rgbd2));
    [fa, da] = vl_sift(rgbd1aux);
    [fb, db] = vl_sift(rgbd2aux);
    [matches, scores] = vl_ubcmatch(da, db);
    [n m]=size(matches);
    numMatches=[numMatches m];
end

[mm,ind]=max(numMatches);
load(imgseq1(ind).depth);
dep1=depth_array;
load(imgseq2(ind).depth);
dep2=depth_array;
im1=imread(imgseq1(ind).rgb);
im2=imread(imgseq2(ind).rgb); 

%dep1(find(dep1>4000))=0; dep2(find(dep2>4000))=0; %eliminate objects that are too far away

xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
rgbd1aux = single(rgb2gray(rgbd1));
rgbd2aux = single(rgb2gray(rgbd2));
[fa, da] = vl_sift(rgbd1aux,'edgethresh',500,'PeakThresh',0);
[fb, db] = vl_sift(rgbd2aux,'edgethresh',500,'PeakThresh',0);%7.5 12.5
[matches, scores] = vl_ubcmatch(da, db);

xa = fa(1,matches(1,:));
xb = fb(1,matches(2,:));
ya = fa(2,matches(1,:));
yb = fb(2,matches(2,:));


%---------------------Point Clouds

% clear im1 im2 xyz1 xyz2 rgbd1 rgbd2;
% im1=imread([path 'rgb_image1_1.png']);
% im2=imread([path 'rgb_image2_1.png']);
% 
% dep1(find(dep1>2000))=0; dep2(find(dep2>2000))=0; %eliminate objects that are too far away
% 
% xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
% xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
% 
% rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
% rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
% 
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));

figure(4);clf; showPointCloud(pc1);
figure(5);clf; showPointCloud(pc2);
toc

%%
%-------------------------RANSAC
tic
niter=1000;
%iteration=1;
%minimumpoits=15;
%maxerror=0.1;
errorthresh=0.1;
numinliers=[];R=[];T=[];
 
xa=fix(xa); x1=xa'; 
ya=fix(ya); y1=ya';
xb=fix(xb); x2=xb';
yb=fix(yb); y2=yb';

ind1=sub2ind([480 640],y1,x1);
ind2=sub2ind([480 640],y2,x2);

P1=xyz1(ind1,:);
P2=xyz2(ind2,:);

inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(inds,:);P2=P2(inds,:);

randind=size(P1,1);
aux=fix(1+randind*rand(4*niter,1));%Makes random sets of 4 indexes

%for errorthresh=0.01:0.01:0.3
%for errorthresh=0.01:0.01:maxerror     
    for i=1:niter-4,
        P1aux=P1(aux(4*i:4*i+3),:);%Select 4 points to estimate the inliers
        P2aux=P2(aux(4*i:4*i+3),:);  

        [d,xx,tr]=procrustes(P1aux,P2aux,'scaling',false,'reflection',false);

        R=[R tr.T];
        T=[T tr.c];

        erro=P1-P2*tr.T-ones(length(P2),1)*tr.c(1,:);

        numinliers=[numinliers length(find(sum(erro.*erro,2)<errorthresh^2))];  %Computing the norm2
    end

%     [mm,ind]=max(numinliers);
% 
%     if (mm>=minimumpoits)
%         break;
%     end    
%     test(iteration,1)=mm; test(iteration,2)=errorthresh; %colum 1 -> number of inliers ; colum 2 -> errorthresh 
%     iteration=iteration+1
%end

    [mm,ind]=max(numinliers);
    
    R=R(:,(ind-1)*3+1:(ind-1)*3+3);
    T=T(1,(ind-1)*3+1:(ind-1)*3+3);

    erro=P1-P2*R-ones(length(P2),1)*T;
    inds=find(sum(erro.*erro,2)<errorthresh^2); %Find the indexes of the inliers

    P1=P1(inds,:);  %Fetch the inliers 
    P2=P2(inds,:);

    [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false); %Use yhose inliers to compute the final transformation

    %-------------------------------------------
    % figure (7);
    % image (rgbd1);
    % hold on;
    % plot(P1(:,1),P1(:,2),'*','MarkerSize',10)
    % hold off;
%     figure(7); image(cat(2, rgbd1, rgbd2));
%     
%     [C1,ia1,ib1] = intersect(xyz1,P1,'rows');
%     [xaa,yaa] = ind2sub([480 640],ia1);
%     
%     [C2,ia2,ib2] = intersect(xyz2,P2,'rows');
%     [xbb,ybb] = ind2sub([480 640],ia2);
%     
%     xaa=fix(xaa)'; xbb=fix(xbb)';
%     yaa=fix(yaa)'; ybb=fix(ybb)';
%     
%     xbbaux = xbb + size(rgbd1,2);
%     
%     hold on;
%     hh=line([xaa ; xbbaux], [yaa ; ybb]);
%     set(hh,'linewidth', 1, 'color', 'b');
%     hold on;
%     scatter(xaa, yaa, 'g', 'filled');
%     hold on;
%     scatter(xbbaux, ybb, 'r', 'filled');

    %--------------------------------------------

    xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
    pc2=pointCloud(xyz21,'Color',reshape(rgbd2,[480*640 3]));
    figure(6); pcshow(pcmerge(pc1,pc2,0.001));
    toc
