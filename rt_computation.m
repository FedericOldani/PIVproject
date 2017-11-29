function tr=rt_computation(im1, depth1, im2, depth2)
    
    load('calib_asus.mat');
    %dep2(find(dep2(:)>4000))=0;
    xyz1=get_xyzasus(depth1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2=get_xyzasus(depth2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    %REGISTER RGB TO DEPTH
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    % figure(1);imagesc(rgbd1 );
    % figure(2);imagesc(rgbd2 );

    np=6;
    x1=zeros(np,1); y1=x1; x2=y1; y2=x1;
    
    
     %values for stillnature
%     x2=[301 324 296 301 100 405];
%     y2=[239 143 175 221 317 190];
%     x1=[281 306 301 289 158 305];
%     y1=[126 17 46 103 206 84];


    %values for chocapics
%     x1=[267 165 168 371 270 273];
%     y1=[187 193 78 116 136 84];
%     x2=[269 174 163 479 265 260];
%     y2=[301 301 211 220 253 210];
%     
    
    %values for maizena4
%     x1=[124 107 457 491 183 184];
%     y1=[394 301 317 208 294 193];
%     x2=[253 263 428 379 265 139];
%     y2=[283 162 190 105 162 63];
    
    
    %values for maizena3
    x1=[178 280 169 268 501 467];
    y1=[306 303 211 204 218 243];
    x2=[169 277 174 286 383 358];
    y2=[193 185 68 71 108 133];

    ind1=sub2ind(size(depth1),y1,x1);
    ind2=sub2ind(size(depth2),y2,x2);

    P1=xyz1(ind1,:);
    P2=xyz2(ind2,:);
    inds=find((P1(:,3).*P2(:,3))>0);
    P1=P1(inds,:);P2=P2(inds,:);
    [dd,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);

