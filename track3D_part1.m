%function objects = track3D_part1( imgseq1, imgseq2,   cam_params,  cam1toW, cam2toW)

%changing variables!!!
farAwayObj = 4000; %4 meters

objPixelSize = 2000; %more than 2000 pixels
acceptHoles = -30; %accept atleast 25 holes!!!
nearPoints = 100; %nearest X points to some point; professor said 10
accept3dPoints = 2000; %accept objects > 1000 3d points
waitSeconds = 5; %See the image for 7 seconds

bgdist=0.2;
faraway=4000;
objsize=2000;

bg1=get_bg(imgseq1);
bg2=get_bg(imgseq2);

objects=struct('X',{[]},'Y',{[]},'Z',{[]},'frames_tracked',{});


%eliminate bg for all images in dataset


for i=1:length(imgseq1)
    %i=17;
    vector_old_obj=[];
    new_obj=struct('X',{[]},'Y',{[]},'Z',{[]},'frames_tracked',{},'f',{},'f1',{},'f2',{});
    
    %close all;
    
    
    fprintf('\n%d...',i);
    % ----------------------  get objects in image1   ----------------------
    
    load(imgseq1(i).depth);
    dep1=depth_array;
    obj1=remove_bg(dep1, bg1, bgdist, faraway, objsize);
    %     obj1 = imfill(obj1,'holes');
%    obj1=bwpropfilt(obj1,'EulerNumber',[acceptHoles 1]);%remove regions with many holes
    
    [L1, num1]=bwlabel(obj1,8);
    
    %      se = strel('disk',10);
    %      L1=imopen(L1,se);
    
    
    % ----------------------  get objects in image2   ----------------------
    
    load(imgseq2(i).depth);
    dep2=depth_array;
    
    obj2=remove_bg(dep2,bg2, bgdist, faraway, objsize);
%    obj2=bwpropfilt(obj2,'EulerNumber',[acceptHoles 1]);%remove regions with many holes
    %     obj2 = imfill(obj2,'holes');
    
    [L2, num2]=bwlabel(obj2,8);
    
    %      L2=imopen(L2,se);
    
    im1=imread(imgseq1(i).rgb);
    im2=imread(imgseq2(i).rgb);
    
    dep1(find(dep1>farAwayObj))=0;%eliminate too far objects 2 meters
    dep2(find(dep2>farAwayObj))=0;%eliminate too far objects
    
    %get coordinate in a unique reference system
    [xyz1,rgbd1]=getCoordinate(dep1,im1,cam1toW,cam_params);
    [xyz21,rgbd2]=getCoordinate(dep2,im2,cam2toW,cam_params);
    
    pc1=getPointCloud(xyz1,rgbd1);
    pc2=getPointCloud(xyz21,rgbd2);
    
    %-------------- features points im1-----------------------
    bw1=rgb2gray(im1);
    [points1,f1] = vl_sift(single(bw1));%,'edgethresh',10,'PeakThresh',20);
    
    matbw=zeros(480,640);    
    loc1(:,2)=round(points1(1,:));
    loc1(:,1)=round(points1(2,:));
    for b=1:length(loc1)
        matbw(loc1(b,1),loc1(b,2))=b;
    end
    matbw=reshape(matbw,[480*640, 1]);
    xyz1=[xyz1 matbw zeros(length(xyz21(:,1)),1)];
    
    %-------------- features points im2-----------------------
    
    bw2=single(rgb2gray(im2));
    [points2,f2] = vl_sift(bw2);%,'edgethresh',10,'PeakThresh',20);
    matbw=zeros(480,640);
    
    loc2(:,2)=round(points2(1,:));
    loc2(:,1)=round(points2(2,:));
    for b=1:length(loc2)
        matbw(loc2(b,1),loc2(b,2))=b;
    end
    matbw=reshape(matbw,[480*640, 1]);
    xyz21=[xyz21 zeros(length(xyz21(:,1)),1) matbw];
    
    f1=f1';
    f2=f2';
    f=[f1; f2];
    %--------------------------------------------------------
    
    if ~isempty(find(L1~=0))
        
        obj3d_1=xyz1(find(L1~=0),:);
        [~,r] = unique(obj3d_1(:,1:3),'rows');%optimization, remove rows with duplicated values
        obj3d_1=obj3d_1(r,:);
    else
        obj3d_1=[];
    end
    
    if ~isempty(find(L2~=0))
        
        obj3d_2=xyz21(find(L2~=0),:);
        [~,r] = unique(obj3d_2(:,1:3),'rows');%optimization, remove rows with duplicated values
        obj3d_2=obj3d_2(r,:);
        
    else
        obj3d_2=[];
    end
    
    obj3d=[obj3d_1 ; obj3d_2]; %put all points coordinates together
    
    if ~isempty(obj3d)
        [~,r] = unique(obj3d(:,1:3),'rows');%optimization, remove rows with duplicated values
        obj3d=obj3d(r,:);
        %
        
        % ------------------------ search objects in 3d ------------------------
        
        % 1.knn search to search for nearest neighbour of each point
        % 2.load a sparse distance matrix with values found (only if distane is
        %    below a certain limit)
        % 3.build a graph (different objects are not connected)
        % 4.each separate subgraph represent an object
        
        [kn,D] = knnsearch(obj3d(:,1:3),obj3d(:,1:3),'k',nearPoints);%default 10; more than 50 points together
        
        idx=0;
        if ~isempty(kn)
            DG=sparse(length(kn(:,1)),length(kn(:,1)),0);
            for h=2:length(kn(1,:))
                DG=DG+sparse(kn(:,1),kn(:,h),D(:,h)<0.1,length(kn(:,1)),length(kn(:,1))); %up to 15cm
            end
            G=graph(DG,'lower');
            bins = conncomp(G);
            idx=bins;
        end
        
        figure();
        j=0;%index to save 'new_obj'
        pcshow(pcmerge(pc1,pc2,0.001));
        
        for n=1:max(idx) %num of objects
            
            hold on;
            if length(find(idx==n))>accept3dPoints %more than 1000 3d points
                j=j+1;
                fprintf('printing>');
                plot3(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3),'.','MarkerSize',10); hold on;
                box=corner3d(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3));
                plot_box(box);
                hold on;
                
                new_obj(j).X(1,:)=box(:,1)';
                new_obj(j).Y(1,:)=box(:,2)';
                new_obj(j).Z(1,:)=box(:,3)';
                
                %------features1
                a=obj3d(idx==n,4);
                a=a(a~=0);
                for z=1:length(a)
                    new_obj(j).f1(end+1,:)=f1(a(z),:);
                end
                %------features2
                a=obj3d(idx==n,5);
                a=a(a~=0);
                for z=1:length(a)
                    new_obj(j).f2(end+1,:)=f2(a(z),:);
                end
                
            else
                box(:,:,n)=zeros(8,3);
            end
            
            
        end
        
        if ~isempty(new_obj)
            
            if ~isempty(objects) && i~=1
                
                abs_distance_matrix=ones(j,number_obj_prev)*(-inf);
                for q=1:j
                    for y=1:number_obj_prev
                        indexPairs1=[];
                        indexPairs2=[];
                        if ~isempty(new_obj(q).f1) && ~isempty(old_obj(y).f1)
                            indexPairs1 = vl_ubcmatch(new_obj(q).f1',old_obj(y).f1') ;
                        end
                        if ~isempty(new_obj(q).f2) && ~isempty(old_obj(y).f2)
                            indexPairs2 = vl_ubcmatch(new_obj(q).f2',old_obj(y).f2') ;
                        end
                       
                            indexPairs=length(indexPairs1)+length(indexPairs2); 
                        
                        if ~isempty(new_obj(q).f) && ~isempty(old_obj(y).f)
                            indexPairs = vl_ubcmatch(new_obj(q).f',old_obj(y).f') ;
                        end
                            abs_distance_matrix(q,y)=indexPairs;                                               
                    end
                end
                
                more=1;
                while more==1
                    more=0;
                    
                    [m,index]=max(abs_distance_matrix(:));
                    [q,y]=ind2sub(size(abs_distance_matrix),index);
                    
                    if abs_distance_matrix(q,y)>2
                        objects(old_obj(y).index).frames_tracked(end+1)= i;
                        objects(old_obj(y).index).X(end+1,:)=new_obj(q).X(1,:);
                        objects(old_obj(y).index).Y(end+1,:)=new_obj(q).Y(1,:);
                        objects(old_obj(y).index).Z(end+1,:)=new_obj(q).Z(1,:);
                        new_obj(q).index=old_obj(y).index;
                        tag=old_obj(y).index;
                        txt=['object' int2str(tag)];
                        text(new_obj(q).X(1),new_obj(q).Y(1),new_obj(q).Z(1),txt);
                        
                        abs_distance_matrix(:,y)=-inf;
                        abs_distance_matrix(q,:)=-inf;
                        more=1;
                        
                        vector_old_obj=[vector_old_obj q];
                    end
                end
            end
            
            idx_new_obj=setdiff(1:j,vector_old_obj);
            
            
            for p=1:length(idx_new_obj)
                r=idx_new_obj(p);
                objects(end+1).frames_tracked(1)= i;
                objects(end).X=new_obj(r).X;
                objects(end).Y=new_obj(r).Y;
                objects(end).Z=new_obj(r).Z;
                tag=length(objects);
                new_obj(r).index=length(objects);
                txt=['object' int2str(tag)];
                text(new_obj(r).X(1),new_obj(r).Y(1),new_obj(r).Z(1),txt);
            end
            
            %pause(waitSeconds); %seconds to see the image
            input('continua');
            
            number_obj_prev=j;
            clear old_obj;
            old_obj=new_obj;
            clear new_obj;            
            
        end
    end
    
    clear loc1;
    clear loc2;
end


%% 21 22

