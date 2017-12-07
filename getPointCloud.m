function pc=getPointCloud(xyz,rgbd)

%create point cloud
pc=pointCloud(xyz,'Color',reshape(rgbd,[480*640 3]));

end