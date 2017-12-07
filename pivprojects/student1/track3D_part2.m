function [objects, cam1toW, cam2toW] = track3D_part2( im1, im2, cams )

load lab1JPC
cam1toW.R=R1;
cam1toW.T=T1;
cam2toW.R=R2;
cam2toW.T=T2;
