function plot_box(p)
x=[p(1,1); p(2,1)]; y=[p(1,2); p(2,2)]; z=[p(1,3); p(2,3)]; plot3(x,y,z);hold on;
x=[p(1,1); p(3,1)]; y=[p(1,2); p(3,2)]; z=[p(1,3); p(3,3)]; plot3(x,y,z);hold on;
x=[p(1,1); p(5,1)]; y=[p(1,2); p(5,2)]; z=[p(1,3); p(5,3)]; plot3(x,y,z);hold on;
x=[p(2,1); p(4,1)]; y=[p(2,2); p(4,2)]; z=[p(2,3); p(4,3)]; plot3(x,y,z);hold on;
x=[p(2,1); p(6,1)]; y=[p(2,2); p(6,2)]; z=[p(2,3); p(6,3)]; plot3(x,y,z);hold on;
x=[p(3,1); p(4,1)]; y=[p(3,2); p(4,2)]; z=[p(3,3); p(4,3)]; plot3(x,y,z);hold on;
x=[p(3,1); p(7,1)]; y=[p(3,2); p(7,2)]; z=[p(3,3); p(7,3)]; plot3(x,y,z);hold on;
x=[p(4,1); p(8,1)]; y=[p(4,2); p(8,2)]; z=[p(4,3); p(8,3)]; plot3(x,y,z);hold on;
x=[p(7,1); p(8,1)]; y=[p(7,2); p(8,2)]; z=[p(7,3); p(8,3)]; plot3(x,y,z);hold on;
x=[p(5,1); p(6,1)]; y=[p(5,2); p(6,2)]; z=[p(5,3); p(6,3)]; plot3(x,y,z);hold on;
x=[p(5,1); p(7,1)]; y=[p(5,2); p(7,2)]; z=[p(5,3); p(7,3)]; plot3(x,y,z);hold on;
x=[p(6,1); p(8,1)]; y=[p(6,2); p(8,2)]; z=[p(6,3); p(8,3)]; plot3(x,y,z);hold off;
