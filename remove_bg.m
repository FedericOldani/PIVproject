function objects=remove_bg( depth, bg)
    absimage=abs((double(depth)/1000-bg))>.04; %pick objects which differ at least 25cm from bg
    objects=bwareaopen(absimage,500); %pick objects with more than 500px
    %objects=absimage;
