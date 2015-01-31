function aa = rotToaa(R)
  trace_R = R(1,1)+R(2,2)+R(3,3);
  angle = acos((trace_R - 1)/2);
  st = sin(angle);
  aa(1) = (R(3,2)-R(2,3))/(2*st)*angle;
  aa(2) = (R(1,3)-R(3,1))/(2*st)*angle;
  aa(3) = (R(2,1)-R(1,2))/(2*st)*angle;
end;
