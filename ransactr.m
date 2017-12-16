function [tr]=ransactr(xyz1, x1, y1, xyz2, x2, y2, niter, errorthresh)

%Initializations
R=[]; T=[]; numinliers=[]; 

%Convert to linear indexes
ind1=sub2ind([480 640],y1,x1);
ind2=sub2ind([480 640],y2,x2);

%Get the corresponting points in xyz
P1=xyz1(ind1,:);
P2=xyz2(ind2,:);

%Valid points
inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(inds,:);P2=P2(inds,:);

%Makes random sets of 4 indexes
randind=size(P1,1);
aux=fix(1+randind*rand(4*niter,1));

%Test those sets
for k=1:niter-4,
    %Select 4 points to estimate the inliers
    P1aux=P1(aux(4*k:4*k+3),:);
    P2aux=P2(aux(4*k:4*k+3),:);  

    [d,xx,tr]=procrustes(P1aux,P2aux,'scaling',false,'reflection',false);
    R=[R tr.T]; T=[T tr.c];

    %Model
    erro=P1-P2*tr.T-ones(length(P2),1)*tr.c(1,:);
    
    %Compute the norm2 and count the number of inliers
    numinliers=[numinliers length(find(sum(erro.*erro,2)<errorthresh^2))];  
end

%Retrieve the R and T that got the maximum number of inliers
[mm,ind]=max(numinliers);
R=R(:,(ind-1)*3+1:(ind-1)*3+3);
T=T(1,(ind-1)*3+1:(ind-1)*3+3);

%Find the inliers
erro=P1-P2*R-ones(length(P2),1)*T;
inds=find(sum(erro.*erro,2)<errorthresh^2); 

%Fetch the inliers
P1=P1(inds,:); 
P2=P2(inds,:);

%Use those inliers to compute the final transformation
[d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);

%Create structure
field1 = 'T';  value1 = tr.T;
field2 = 'b';  value2 = tr.b;
field3 = 'c';  value3 = tr.c;
tr = struct(field1,value1,field2,value2,field3,value3);