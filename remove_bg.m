function objects=remove_bg( depth, bg)
    absimage=abs((double(depth)/1000-bg))>0.05; %pick objects which differ at least 5cm from bg
    absimage(depth>2000)=0; %more than 2 meters
    %absimage=medfilt2(absimage);
    objects=bwareaopen(absimage,900); %pick objects with more than 900px
    
    %objects=absimage;
