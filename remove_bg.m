function objects=remove_bg(depth,bg)
    absimage=abs((double(depth)/1000-bg))>.25; %pick objects which differ at least 25cm from bg
    %objects=bwareaopen(absimage,2500); %pick objects with more than 2500px
    objects=absimage;
