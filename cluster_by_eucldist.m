function [idx,cluster]=cluster_by_eucldist(obj,distance)
idx=zeros(length(obj),1);
cluster=0;
for i=1:length(obj)
       if idx(i)==0
           cluster=cluster+1;
           idx(i)=cluster;
       end
       for k=i+1:length(obj)
           E_distance = sqrt(sum((obj(i)-obj(k)).^2));
           if E_distance<distance
               idx(k)=idx(i);
           end
       end
end

