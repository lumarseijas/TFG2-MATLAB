function posGeod = radar2geodetic(dist,azim,elev, posRadar)

% existe la funcion que directamente lo calcula en grados (d de degrees)
xenu = dist .* cosd(elev) .* sind(azim);
yenu = dist .* cosd(elev) .* cosd(azim);
zenu = dist .* sind(elev);

%lv2ecef es antiguo
% [xecef, yecef, zecef ] = lv2ecef (xlv,ylv,zlv,posRadar(1)*pi/180,...
%     posRadar(2)*pi/180,posRadar(3),geoide);
spheroid = referenceEllipsoid('WGS84');
[xecef, yecef, zecef] = enu2ecef(xenu, yenu, zenu, posRadar(1), posRadar(2), posRadar(3), spheroid);

% [lat, long, alt] = ecef2geodetic(xecef, yecef, zecef, geoide);
[lat, long, alt] = ecef2geodetic(spheroid, xecef, yecef, zecef);

% posGeod = [ lat*180/pi long*180/pi alt ];
posGeod = [ lat, long, alt ];