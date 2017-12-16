function [xyz,rgbd]=ImageParameters(dep,im,cam_param)

xyz=get_xyzasus(dep(:),[480 640],(1:640*480)', cam_param.Kdepth,1,0);
rgbd = get_rgbd(xyz, im, cam_param.R, cam_param.T, cam_param.Krgb);

end