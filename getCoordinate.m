function [xyz,rgbd]=getCoordinate(dep,im,camtoW,cam_param)

xyz_temp=get_xyzasus(dep(:),[480 640],(1:640*480)', cam_param.Kdepth,1,0);
xyz=xyz_temp*camtoW.R+ones(length(xyz_temp),1)*camtoW.T(1,:);
rgbd = get_rgbd(xyz_temp, im, cam_param.R, cam_param.T, cam_param.Krgb);

end