function pc=getPointCloud(xyz,rgbd)

load('calib_asus.mat');





%create point cloud
pc=pointCloud(xyz,'Color',reshape(rgbd,[480*640 3]));

end