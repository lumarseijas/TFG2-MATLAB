function elev = elevation2 (dist, alt, posRadar, geoide)
% posRadar(:,1) = latitud
% posRadar(:,3) = altura

N = length(dist);

% a = geoide(1);
% e = geoide(2);
a = geoide.SemimajorAxis;
f = geoide.Flattening;
e = sqrt(2*f - f^2);  % excentricidad a partir del achatamiento

R = a * (1-e^2) ./ sqrt( (1-e^2*sind(posRadar(1)).^2).^3 ); 

elev = asind ( (2*R.*(alt-ones(N,1)*posRadar(3)) + alt.^2 -...
    ones(N,1)*posRadar(3)^2 - dist.^2 ) ./ (2*(dist*(R+posRadar(3)))) );