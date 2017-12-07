function objects=remove_bg( depth, bg, bgdist, faraway, objsize)
    absimage=abs((double(depth)/1000-bg))>bgdist; %pick objects which differ at least 5cm from bg
    absimage(depth>faraway)=0; %more than 2 meters
    %absimage=medfilt2(absimage);
    objects=bwareaopen(absimage,objsize); %pick objects with more than 900px
    
    %objects=absimage;
