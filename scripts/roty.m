function R = roty(angle)
  ct = cos(angle);
  st = sin(angle);
  R(1,1) = ct;    R(1,2) = 0.0;    R(1,3) = st;     R(1,4) = 0.0;
  R(2,1) = 0.0;  R(2,2) = 1.0;    R(2,3) = 0.0;   R(2,4) = 0.0;
  R(3,1) = -st;   R(3,2) =  0.0 ;  R(3,3) = ct;     R(3,4) = 0.0;
  R(4,1) = 0.0;  R(4,2) = 0.0;    R(4,3) = 0.0;   R(4,4) = 1.0;
end;