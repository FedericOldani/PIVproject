function [xyz1,rgbd1,xyz21,rgbd2]=getCoordinate(dep1,im1,dep2,im2,tr)
load('calib_asus.mat');

xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);

end