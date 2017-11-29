%function objects = track3D_part1( imgseq1, imgseq2,   cam_params,  cam1toW, cam2toW);

bg1=get_bg(imgseq1);
bg2=get_bg(imgseq2);

objects=struct('X',{},'Y',{},'Z',{},'frames_tracked',{});
%eliminate bg for all images in dataset
for i=1:length(imgseq1)
    %i=9;
    
    close all;
    fprintf('\n%d...',i);
    % ----------------------  get objects in image1   ----------------------

    load(imgseq1(i).depth);
    dep1=depth_array;
    obj1=remove_bg(dep1, bg1);
    obj1=bwpropfilt(obj1,'EulerNumber',[-30 1]);%remove region with holes
    [L1, num1]=bwlabel(obj1,8);
    
    
    
    % ----------------------  get objects in image2   ----------------------
    
    load(imgseq2(i).depth);
    dep2=depth_array;
    obj2=remove_bg(dep2,bg2);
    obj2=bwpropfilt(obj2,'EulerNumber',[-30 1]);%remove region with holes
    [L2, num2]=bwlabel(obj2,8);
    
    % ----------------------   point cloud   ---------------------
    
    im1=imread(imgseq1(i).rgb);
    im2=imread(imgseq2(i).rgb);
    
    dep1(find(dep1>2000))=0;%eliminate too far objects
    dep2(find(dep2>2000))=0;%eliminate too far objects
    
    %get coordinate in a unique reference system
    [xyz1,rgbd1]=getCoordinate(dep1,im1,cam1toW,cam_params);
    [xyz21,rgbd2]=getCoordinate(dep2,im2,cam2toW,cam_params);
    
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
    
    [kn,D] = knnsearch(obj3d,obj3d,'k',20);%default 10
    
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
            box=corner3d(obj3d(idx==n,1),obj3d(idx==n,2),obj3d(idx==n,3));
            plot_box(box);    
            hold on;
            
            new_obj.X(1,:)=box(:,1)';
            new_obj.Y(1,:)=box(:,2)';
            new_obj.Z(1,:)=box(:,3)';
            
            
            flag=0;
            if length(objects)>0
                threshold=ones(1,8)*0.2;
    
                for y=1:length(objects)
                    fr=objects(y).frames_tracked;
                    for z=1:length(fr)
                        if fr(z)==i-1
                            differenceX=abs(objects(y).X(z,:)-new_obj.X(1,:)).^2;
                            differenceY=abs(objects(y).Y(z,:)-new_obj.Y(1,:)).^2;
                            differenceZ=abs(objects(y).Z(z,:)-new_obj.Z(1,:)).^2;
                            difference=sqrt(differenceX+differenceY+differenceZ);
                            if sum(difference<threshold)==8
                                objects(y).frames_tracked(end+1)= i;
                                objects(y).X(end+1,:)=new_obj.X(1,:);
                                objects(y).Y(end+1,:)=new_obj.Y(1,:);
                                objects(y).Z(end+1,:)=new_obj.Z(1,:);
                                flag=1;
                                tag=y;
                            end
                        end
                    end
                end
            end
            
            if flag==0
                new_obj.frames_tracked(1)= i;
                objects(end+1)=new_obj;
                tag=length(objects);
            end
            
            txt=['object' int2str(tag)];
            text(box(1,1),box(1,2),box(1,3),txt);

        else
            box(:,:,n)=zeros(8,3);
        end
        
    end
    clear box;
    pause(4);
    
end




%end