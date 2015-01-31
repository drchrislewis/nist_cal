function [rot_z, rot_y, rot_x] = rotToZYX(R)
  if( abs(R(3,1)) != 1 )
     theta = -asin(R(3,1));
     ct = cos(theta);
     psi = atan2(R(3,2)/ct, R(3,3)/ct);
     phi = atan2(R(2,1)/ct, R(1,1)/ct);
  else
    phi = 0;
    if(R(3,1) == -1)
      theta = pi/2;
      psi = phi + atan2(R(1,2), R(1,3));
    else
      theta = -pi/2;
      psi = -phi + atan2(-R(1,2), -R(1,3));
    endif
  endif
  rot_y = theta;
  rot_x = psi;
  rot_z = phi;
end;
