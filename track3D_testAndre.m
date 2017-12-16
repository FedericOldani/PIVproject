%function objects = track3D_part1( imgseq1, imgseq2,   cam_params,  cam1toW, cam2toW)

%changing variables!!!
farAwayObj = 4000; %4 meters
farawayArray = [3000 3200 3400 3600 3800 4000 4200 4400 4600 4800 5000];

objsize = 2000; %more than 2000 pixels
objsizeArray = [900 1020 1140 1260 1380 1500 1620 1740 1860 1980 2100];

bgdist = 0.05; %5cm from background
bgdistArray = [0.05 0.07 0.09 0.11 0.13 0.15 0.17 0.19 0.21 0.23 0.25];

acceptHoles = -30; %accept atleast 25 holes!!!
acceptholesArray = [-26 -27 -28 -29 -30 -31 -32 -33 -34 -35 -36];

nearPoints = 50; %nearest X points to some point; professor said 10
nearpointsArray = [10 20 30 40 50 60 70 80 90 100 110];

accept3dPoints = 1000; %accept objects > 1000 3d points
acceptpointsArray = [700 800 900 1000 1100 1200 1300 1400 1500 1600 1700];

waitSeconds = 7; %See the image for 7 seconds


test = [1 1 1 2 2 2 1 3 1 2 3 3 2 3 3 2 2 2 3 2];

bg1=get_bg(imgseq1);
bg2=get_bg(imgseq2);

objects=struct('X',{[]},'Y',{[]},'Z',{[]},'frames_tracked',{});


%eliminate bg for all images in dataset

nrObjs = zeros(11,11,11,11,11,length(imgseq1)+1);

for a=1:length(farawayArray)
    a
    for b=1:length(objsizeArray)
        b
       for c=1:length(bgdistArray)
           c
          % d=1:length(acceptholesArray)
             % d
            for e=1:length(nearpointsArray)
                e
                for f=1:length(acceptpointsArray)
                    f
                   for i=1:(length(imgseq1)+1)
%       i=1;    
                        if(i == length(imgseq1)+1)
                         
                        else
                            new_obj=struct('X',{[]},'Y',{[]},'Z',{[]},'frames_tracked',{},'hue1',{},'sat1',{});
                            close all;
                            %fprintf('\n%d...',i);
                            % ----------------------  get objects in image1   ----------------------

                            load(imgseq1(i).depth);
                            dep1=depth_array;
                            obj1=remove_bg(dep1, bg1, bgdistArray(c), farawayArray(a), objsizeArray(b));
                            %obj1=bwpropfilt(obj1,'EulerNumber',[-30 1]);%remove regions with many holes    

                            [L1, num1]=bwlabel(obj1,8);

                        %      se = strel('disk',10);
                        %      L1=imopen(L1,se);


                            % ----------------------  get objects in image2   ----------------------

                            load(imgseq2(i).depth);
                            dep2=depth_array;

                            obj2=remove_bg(dep2,bg2, bgdistArray(c), farawayArray(a), objsizeArray(b));
                            %obj2=bwpropfilt(obj2,'EulerNumber',[-30 1]);%remove regions with many holes

                            [L2, num2]=bwlabel(obj2,8);

                        %      L2=imopen(L2,se);

                            im1=imread(imgseq1(i).rgb);
                            im2=imread(imgseq2(i).rgb);


                            dep1(find(dep1>farawayArray(a)))=0;%eliminate too far objects 2 meters
                            dep2(find(dep2>farawayArray(a)))=0;%eliminate too far objects

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


                            [kn,D] = knnsearch(obj3d,obj3d,'k',nearpointsArray(e));%default 10; more than 50 points together


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


                            j=0;%index to save 'new_obj'
                            %pcshow(pcmerge(pc1,pc2,0.001));

                            for n=1:max(idx) %num of objects

                                hold on;
                                if length(find(idx==n))>acceptpointsArray(f) %more than 1000 3d points
                                    j=j+1;
                                    %fprintf("printing>");
                                    nrObjs(a,b,c,e,f,i) = nrObjs(a,b,c,e,f,i)+1; 
                                end


                            end
                               if(nrObjs(a,b,c,e,f,i) == test(i))
                                    nrObjs(a,b,c,e,f,length(imgseq1)+1) = nrObjs(a,b,c,e,f,length(imgseq1)+1) + 1;
                                end
                            
                        end
                   end
                    
                end
            end
          
       end
    end
end






