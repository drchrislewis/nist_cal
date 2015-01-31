function [qx, qy, qz, qw] =  rotToq(R)
    aa = rotToaa(R);
    ax = aa(1);
    ay = aa(2);
    az = aa(3);

    theta_squared = ax*ax + ay*ay + az*az;
    if (theta_squared>0.0)
      theta = sqrt(theta_squared);
      half_theta = theta*0.5;
      k = sin(half_theta)/theta;
      qw = cos(half_theta);
      qx = ax*k;
      qy = ay*k;
      qz = az*k;
    else
      k = 0.5;
      qw = 1.0;
      qx = ax;
      qy = ay;
      qz = az;
   endif;
end;
