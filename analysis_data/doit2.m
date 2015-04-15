field_results2;
% compute magnitude of errors for display
frows = 2/0.1+1;
fcols = 2/0.1+1;
z = [];
zmax=0.0005;
for i=1:fcols, for j=1:frows,fi=(i-1)*frows+j; ex=f(fi,7);ey=f(fi,8);ez=f(fi,9);z(j,i) = sqrt(ex*ex+ey*ey+ez*ez); end; end;
for i=1:fcols, for j=1:frows,fi=(i-1)*frows+j; if (z(j,i)>zmax), z(j,i) = zmax; endif; end; end;
y = -1:0.1:1.0;
x = -1:0.1:1.0;
figure(1);
mesh(x,y,z);
xlabel('Meters');
ylabel('Meters');
zlabel('Meters');
title('Localization Accuracy for work volume (Meters)')
print -djpg heatmesh2.jpg

figure(2);
contourf(x,y,z);
colorbar();
xlabel('Meters');
ylabel('Meters');
title('Localization Accuracy for work Volume')
print -djpg heatimage2.jpg

