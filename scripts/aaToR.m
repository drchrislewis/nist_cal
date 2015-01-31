function R = aaToR(aa)
angle = norm(aa);
ct = cos(angle);
st = sin(angle);
ux = aa(1)/angle;
uy = aa(2)/angle;
uz = aa(3)/angle;
omct = 1.0 - ct;
R(1,1) = ct + ux*ux*omct;      R(1,2) = ux*uy*omct - uz*st;   R(1,3) = ux*uz*omct + uy*st;    R(1,4) = 0.0;
R(2,1) = uy*ux*omct + uz*st; R(2,2) =  ct+uy*uy*omct;        R(2,3) = uy*uz*omct - ux*st;   R(2,4) = 0.0;
R(3,1) = uz*ux*omct - uy*st;  R(3,2) =  uz*uy*omct + ux*st;  R(3,3) = ct + uz*uz*omct;       R(3,4) = 0.0;
R(4,1) = 0.0;                            R(4,2) = 0.0;                             R(4,3) = 0.0;                             R(4,4) = 1.0;
end;
