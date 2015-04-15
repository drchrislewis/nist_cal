fp = fopen("floor_points.yaml","w");
fp2 = fopen("test.m","w");
fprintf(fp,"---\npoints:\n");
fprintf(fp2,"Points = [\n");
spacing = 10;
% main body
for x=5:spacing:155, for y=5:spacing:105, fprintf(fp2," %f, %f, 0.0;\n",x,y); end; end;
for x=5:spacing:155, for y=5:spacing:105, fprintf(fp,"- pnt: [ %f, %f, 0.0 ]\n",x,y); end; end;

% nose cube
for x=45:spacing:115, for y=110:spacing:185, fprintf(fp2,"%f, %f, 0.0;\n",x,y); end; end;
for x=45:spacing:115, for y=110:spacing:185, fprintf(fp,"- pnt: [ %f, %f, 0.0 ]\n",x,y); end; end;

% left diagonal section
for x=5:spacing:40, for y=110:spacing:(x+110), fprintf(fp2,"%f, %f, 0.0;\n",x,y); end; end;
for x=5:spacing:40, for y=110:spacing:(x+110), fprintf(fp,"- pnt: [ %f, %f, 0.0 ]\n",x,y); end; end;

% right diagonal section
for x=120:spacing:155, for y=110:spacing:(155-x+110), fprintf(fp2,"%f, %f, 0.0;\n",x,y); end; end;
for x=120:spacing:155, for y=110:spacing:(155-x+110), fprintf(fp,"- pnt: [ %f, %f, 0.0 ]\n",x,y); end; end;

fprintf(fp2,"];\n");
fclose(fp);
fclose(fp2);
