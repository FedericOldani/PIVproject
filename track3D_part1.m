%function objects = track3D_part1( imgseq1, imgseq2,   cam_params,  cam1toW, cam2toW)

%changing variables!!!
farAwayObj = 2000; %2 meters
objPixelSize = 900; %more than 900 pixels
cmFromBG = 0.05; %5cm from background
acceptHoles = -30; %accept atleast 25 holes!!!
nearPoints = 50; %nearest X points to some point; professor said 10
accept3dPoints = 1000; %accept objects > 1000 3d points
waitSeconds = 7; %See the image for 7 seconds


bg1=get_bg(imgseq1);
bg2=get_bg(imgseq2);

objects=struct('X',{[]},'Y',{[]},'Z',{[]},'frames_tracked',{});


%eliminate bg for all images in dataset


for i=1:length(imgseq1)
%       i=1; 
    
    new_obj=struct('X',{[]},'Y',{[]},'Z',{[]},'frames_tracked',{},'hue1',{},'sat1',{});
    
    close all;
    fprintf('\n%d...',i);
    % ----------------------  get objects in image1   ----------------------
    
    load(imgseq1(i).depth);
    dep1=depth_array;
    obj1=remove_bg(dep1, bg1);
    %obj1=bwpropfilt(obj1,'EulerNumber',[-30 1]);%remove regions with many holes    
  
    [L1, num1]=bwlabel(obj1,8);
    
     se = strel('disk',10);
     L1=imopen(L1,se);

    
    % ----------------------  get objects in image2   ----------------------
    
    load(imgseq2(i).depth);
    dep2=depth_array;

    obj2=remove_bg(dep2,bg2);
    %obj2=bwpropfilt(obj2,'EulerNumber',[-30 1]);%remove regions with many holes

    [L2, num2]=bwlabel(obj2,8);
    
     L2=imopen(L2,se);
    
    im1=imread(imgseq1(i).rgb);
    im2=imread(imgseq2(i).rgb);
    
    
    dep1(find(dep1>farAwayObj))=0;%eliminate too far objects 2 meters
    dep2(find(dep2>farAwayObj))=0;%eliminate too far objects
    
    %get coordinate in a unique reference system
    [xyz1,rgbd1]=getCoordinate(dep1,im1,cam1toW,cam_params);
    [xyz21,rgbd2]=getCoordinate(dep2,im2,cam2toW,cam_params);
    
    pc1=getPointCloud(xyz1,rgbd1);
    pc2=getPointCloud(xyz21,rgbd2);
    
    %im1
    hsv=rgb2hsv(im1);
    hsv=reshape(hsv,[480*640 3]);
    hue=hsv(:,1);
    sat=hsv(:,2);
    hue1=hue(L1~=0);
    sat1=sat(L1~=0);
    
    %im2
    hsv=rgb2hsv(im2);
    hsv=reshape(hsv,[480*640 3]);
    hue=hsv(:,1);
    sat=hsv(:,2);
    hue2=hue(L2~=0);
    sat2=sat(L2~=0);
    
    
    %
    
    
    obj3d_1=getObj3d(xyz1,L1);
    obj3d_2=getObj3d(xyz21,L2);
    
    obj3d_1=xyz1(find(L1~=0),:);
    [obj3d_1,r1] = unique(obj3d_1,'rows');%optimization, remove rows with duplicated values
    hue1=hue1(r1);
    sat1=sat1(r1);
    
    obj3d_2=xyz21(find(L2~=0),:);
    [obj3d_2,r2] = unique(obj3d_2,'rows');%optimization, remove rows with duplicated values
    hue2=hue2(r2);
    sat2=sat2(r2);
    
    obj3d=[obj3d_1; obj3d_2]; %put all points coordinates together
    hue=[hue1;hue2];
    sat=[sat1;sat2];
    %
    
    % ------------------------ search objects in 3d ------------------------
    
    % 1.knn search to search for nearest neighbour of each point
    % 2.load a sparse distance matrix with values found (only if distane is
    %    below a certain limit)
    % 3.build a graph (different objects are not connected)
    % 4.each separate subgraph represent an object
    

    [kn,D] = knnsearch(obj3d,obj3d,'k',nearPoints);%default 10; more than 50 points together

    
    idx=0;
    if ~isempty(kn)
        DG=sparse(length(kn(:,1)),length(kn(:,1)),0);
        for h=2:length(kn(1,:))
            DG=DG+sparse(kn(:,1),kn(:,h),D(:,h)<0.1,length(kn(:,1)),length(kn(:,1))); %up to 15cm
        end
        G=graph(DG,'upper');
        bins = conncomp(G);
        idx=bins;
    end
    
    
    j=0;%index to save 'new_obj'
    pcshow(pcmerge(pc1,pc2,0.001));

    for n=1:max(idx) %num of objects
        
        hold on;
        if length(find(idx==n))>accept3dPoints %more than 1000 3d points
            j=j+1;
            fprintf("printing>");
            plot3(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3),'.','MarkerSize',10); hold on;
            box=corner3d(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3));
            plot_box(box);
            hold on;
            
            new_obj(j).X(1,:)=box(:,1)';
            new_obj(j).Y(1,:)=box(:,2)';
            new_obj(j).Z(1,:)=box(:,3)';            
            new_obj(j).hue=mean(hue(idx==n));
            new_obj(j).sat=mean(sat(idx==n));
            
%{             flag=0;
%             if ~isempty(objects) && i~=1
%                 th_hue=0.08;
%                 th_sat=0.02;
%                 
%                 y=1;
%                 while y<=number_obj_prev && flag==0                   
%                     if ~isempty(obj_prev(y).hue1)
%                         abs_distance_matrix(j,y,1)=abs(obj(j).hue1-obj_prev(y).hue1);
%                         abs_distance_matrix(j,y,2)=abs(obj(j).sat1-obj_prev(y).sat1);
%                                                 fprintf('\nHUE:compare %d with %d: %f  -  %f',j,obj_prev(y).index,abs_hue1(y),obj_prev(y).hue1);
%                                                fprintf('\nSAT:compare %d with %d: %f  -  %f',j,obj_prev(y).index,abs_sat1(y),obj_prev(y).sat1);
%                         
%                     else
%                         abs_distance_matrix(j,y,1)=inf;
%                         abs_distance_matrix(j,y,2)=inf; 
%                     end
%                     y=y+1;
%                 end
% 
% 
%                 
%                 if exist('abs_hue1')
%                     distance=sqrt(abs_hue1.^2+abs_sat1.^2);
%                     [~, y]=min(distance);
%                     min_abs_hue=abs_hue1(y);
%                     min_abs_sat=abs_sat1(y);
%                     
%                     if min_abs_hue<th_hue && min_abs_sat<th_sat
%                         objects(obj_prev(y).index).frames_tracked(end+1)= i;
%                         objects(obj_prev(y).index).X(end+1,:)=new_obj.X(1,:);
%                         objects(obj_prev(y).index).Y(end+1,:)=new_obj.Y(1,:);
%                         objects(obj_prev(y).index).Z(end+1,:)=new_obj.Z(1,:);
%                         flag=1;
%                         tag=obj_prev(y).index;
%                         obj(j).index=obj_prev(y).index;
%                     end
%                 end
%                 clear abs_hue1;
%                 clear abs_sat1;
%                 
% 
% 
% 
% 
% 
%                                 for y=1:length(objects)
%                                     fr=objects(y).frames_tracked;
%                                     for z=1:length(fr)
%                                         if fr(z)==i-1
%                                             differenceX=abs(objects(y).X(z,:)-new_obj.X(1,:)).^2;
%                                             differenceY=abs(objects(y).Y(z,:)-new_obj.Y(1,:)).^2;
%                                             differenceZ=abs(objects(y).Z(z,:)-new_obj.Z(1,:)).^2;
%                                             difference=sqrt(differenceX+differenceY+differenceZ);
%                                             if sum(difference<threshold)==8
%                                                 objects(y).frames_tracked(end+1)= i;
%                                                 objects(y).X(end+1,:)=new_obj.X(1,:);
%                                                 objects(y).Y(end+1,:)=new_obj.Y(1,:);
%                                                 objects(y).Z(end+1,:)=new_obj.Z(1,:);
%                                                 flag=1;
%                                                 tag=y;
%                                             end
%                                         end
%                                     end
%                                 end
%             end
%             
%             if flag==0
%                 new_obj.frames_tracked(1)= i;
%                 objects(end+1)=new_obj;
%                 tag=length(objects);
%                 obj(j).index=length(objects);
%             end
%             
%             txt=['object' int2str(tag)];
%             text(box(1,1),box(1,2),box(1,3),txt);
%}            
            
        else
            box(:,:,n)=zeros(8,3);
        end
        
        
    end
    
    if ~isempty(new_obj)
        
        if ~isempty(objects) && i~=1
            th_hue=0.08;
            th_sat=0.05;
            
            abs_distance_matrix=ones(j,number_obj_prev)*inf;
            
            for q=1:j
                for y=1:number_obj_prev
                    hue(q,y)=abs(new_obj(q).hue-old_obj(y).hue);
                    sat(q,y)=abs(new_obj(q).sat-old_obj(y).sat);
                    abs_distance_matrix(q,y)=sqrt(hue(q,y)^2+sat(q,y)^2);
                end
            end
            
            more=1;
            while more==1
                more=0;
                
                [~,index]=min(abs_distance_matrix(:));
                [q,y]=ind2sub(size(abs_distance_matrix),index);
                
                if hue(q,y)<th_hue && sat(q,y)<th_sat && abs_distance_matrix(q,y)~=inf
                    objects(old_obj(y).index).frames_tracked(end+1)= i;
                    objects(old_obj(y).index).X(end+1,:)=new_obj(q).X(1,:);
                    objects(old_obj(y).index).Y(end+1,:)=new_obj(q).Y(1,:);
                    objects(old_obj(y).index).Z(end+1,:)=new_obj(q).Z(1,:);
                    new_obj(q).index=old_obj(y).index;
                    tag=old_obj(y).index;
                    txt=['object' int2str(tag)];
                    text(new_obj(q).X(1),new_obj(q).Y(1),new_obj(q).Z(1),txt);
                    
                    abs_distance_matrix(:,y)=inf;
                    abs_distance_matrix(q,:)=inf;
                    more=1;
                    
                    vector_old_obj=[vector_old_obj q];
                end
            end
        end
        
        idx_new_obj=setdiff(1:j,vector_old_obj);
        %     [idx_new_obj,~]=find(abs_distance_matrix~=inf);
        %     idx_new_obj=unique(idx_new_obj,'rows');
        
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

        pause(waitSeconds); %seconds to see the image

        number_obj_prev=j;
        clear old_obj;
        old_obj=new_obj;
        clear new_obj;
        
    end
    
    vector_old_obj=[];
end




