function p=corner3d(x,y,z)
p=[min(x) min(y) min(z);
    min(x) min(y) max(z);
    min(x) max(y) min(z);
    min(x) max(y) max(z);
    max(x) min(y) min(z);
    max(x) min(y) max(z);
    max(x) max(y) min(z);
    max(x) max(y) max(z);];
%{
000
001
010
011
100
101
110
111
    %}
   