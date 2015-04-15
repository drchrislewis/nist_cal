fp = fopen("field_points.yaml","w")
fprintf(fp,"---\npoints:\n");
for x=-1.0:0.1:1.0, for y=-1.0:0.1:1.0, fprintf(fp,"- pnt: [ %f, %f, 0.4]\n",x,y); end; end;
fclose(fp);
