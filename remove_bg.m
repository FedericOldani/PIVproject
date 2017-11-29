function objects=remove_bg( depth, bg)
    absimage=abs((double(depth)/1000-bg))>0.2; %pick objects which differ at least 25cm from bg
    absimage(depth>2000)=0;
    objects=bwareaopen(absimage,2000); %pick objects with more than 500px
    
    %objects=absimage;
