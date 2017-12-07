function obj3d=getObj3d(xyz,L)

%get objects coordinate: pick points coordinate only where bwlabel() detected an objects (=> L1~=0)
    obj3d=xyz(find(L~=0),:);
    %obj3d_1(obj3d_1(:,3)>2,:)=[];
    %obj3d = unique(obj3d,'rows');%optimization, remove rows with duplicated values
end