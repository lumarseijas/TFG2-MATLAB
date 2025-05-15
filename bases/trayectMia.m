function [posSal, tiempo, vel, rumbo, velVert] = trayectMia(tramos, posIni, velIni, rumboIni, Tini, Tmuestreo, geoide)
%rumbo en grados

acelL = tramos(:,1);
acelT = tramos(:,2);
duracion = tramos(:,4);
Vvert=tramos(:,3);

duracionTotal = sum(duracion);
Ltotal = fix( duracionTotal / Tmuestreo );

% Calculo de los puntos iniciales y finales de cada tramo

% Rumbo inicial de cada tramo, y rumbo final (ultimo elemento)
rumbo = [ rumboIni ; zeros(Ltotal,1) ];
% Velocidad inicial de cada tramo y velocidad final (ultimo elemento)
vel = [ velIni ; zeros(Ltotal,1) ];
% Inicializacion del vector de velocidad vertical
velVert=[Vvert(1) ;  zeros(Ltotal,1) ];
% Posicion inicial de cada tramo y posicion final (ultimo elemento)
posicion = [ posIni(1:2) ; zeros(Ltotal,2) ];
% Altura inicial, de valor constante a lo largo de la trayectoria
altura = [posIni(3); zeros(Ltotal,1)];

tiempo = [ Tini ; zeros(Ltotal,1) ];

ultimo = 1;

%recorre cada tramo:
for i=1:size(tramos,1)
    
    N = fix( duracion(i) / Tmuestreo );
        
    if ( acelT(i) ~= 0 )   %Trayectoria curva sobre ciculos minimos
        
        if (acelL(i) == 0) %Velocidad constante            
            % distancia recorrida
            d = vel(ultimo)*duracion(i);                
            % radio de curvatura del giro
            r = vel(ultimo)^2 / acelT(i);
            %arco recorrido
            arc = (d+d/N) / (r+1e-6) * 180/pi ;
            errorRumbo = 0;
%                 if (geoide(2)~=0)
%                     %Iteramos para corregir la desviacion de rumbo introducida
%                     %por el calculo sobre el elipsoide no esferico
%                     centerAux = reckon('gc', posicion(ultimo,1), posicion(ultimo,2),...
%                         r, rumbo(ultimo)+90, geoide);
%                     azimCircAux = azimuth('gc', centerAux, posicion(ultimo,:), geoide);
%                     errorRumbo = azimCircAux - mod((rumbo(ultimo)+270*sign(arc)),360)
%                 end 
            % centro del circulo minimo
            center = reckon('gc', posicion(ultimo,1), posicion(ultimo,2),...
                r, rumbo(ultimo)+90+errorRumbo, geoide);                
            %azimuth inicial desde el centro del ciculo al avion
            azimCirc = azimuth('gc', center, posicion(ultimo,:), geoide);      

            trackAux = scircle1 ( center(1), center(2), abs(r),...
                [azimCirc+arc*(arc<0) azimCirc+arc*(arc>=0)], geoide, [], N+2 );                
            if (arc<0) 
                posicion(ultimo+1:ultimo+N,:) = trackAux(N+1:-1:2,:);
                rumbo(ultimo+1:ultimo+N) = azimuth( 'gc',...
                trackAux(N+1:-1:2,:), trackAux(N:-1:1,:) );
            else
                posicion(ultimo+1:ultimo+N,:) = trackAux(2:N+1,:);
                rumbo(ultimo+1:ultimo+N) = azimuth( 'gc',...
                trackAux(2:N+1,:), trackAux(3:N+2,:) );
            end
            tiempo(ultimo+1:ultimo+N) = tiempo(ultimo) + Tmuestreo*(1:N)';
            vel(ultimo+1:ultimo+N) = ones(N,1)*vel(ultimo);
            ultimo = ultimo + N;

        else  %Aceleracion constante
            % distancia recorrida
            d = vel(ultimo)*duracion(i);              
            % radio de curvatura del giro
            r = vel(ultimo)^2 / acelT(i);
            %arco recorrido
            arc = (d+d/N) / (r+1e-6) * 180/pi ;
            errorRumbo = 0;
%                 if (geoide(2)~=0)
%                     %Iteramos para corregir la desviacion de rumbo introducida
%                     %por el calculo sobre el elipsoide no esferico
%                     centerAux = reckon('gc', posicion(ultimo,1), posicion(ultimo,2),...
%                         r, rumbo(ultimo)+90, geoide);
%                     azimCircAux = azimuth('gc', centerAux, posicion(ultimo,:), geoide);
%                     errorRumbo = azimCircAux - mod((rumbo(ultimo)+270*sign(arc)),360)
%                 end
            % centro del circulo minimo
            center = reckon('gc', posicion(ultimo,1), posicion(ultimo,2),...
                r, rumbo(ultimo)+90+errorRumbo, geoide);                
            %azimuth inicial desde el centro del ciculo al avion
            azimCirc = azimuth('gc', center, posicion(ultimo,:), geoide);      

            trackAux = scircle1 ( center(1), center(2), abs(r),...
                [azimCirc+arc*(arc<0) azimCirc+arc*(arc>=0)], geoide, [], N+2 );                
            if (arc<0) 
                posicion(ultimo+1:ultimo+N,:) = trackAux(N+1:-1:2,:);
                rumbo(ultimo+1:ultimo+N) = azimuth( 'gc',...
                trackAux(N+1:-1:2,:), trackAux(N:-1:1,:) );
            else
                posicion(ultimo+1:ultimo+N,:) = trackAux(2:N+1,:);
                rumbo(ultimo+1:ultimo+N) = azimuth( 'gc',...
                trackAux(2:N+1,:), trackAux(3:N+2,:) );
            end
            di = d/N * (1:N)';            
            dT = -vel(ultimo)/acelL(i) + sqrt( (vel(ultimo)/acelL(i))^2 + 2*di/acelL(i) );
            if imag(dT(N))~=0
                disp('Error, la aeronave no puede pararse en el aire');
            end
            tiempo(ultimo+1:ultimo+N) = tiempo(ultimo) + dT;
            vel(ultimo+1:ultimo+N) = vel(ultimo)+acelL(i)*dT;
            ultimo = ultimo + N;

        end
        
    else  %Trayectoria recta sobre circulos maximos       
        if (acelL(i) == 0)  %Velocidad constante         
            % distancia recorrida
            d = vel(ultimo)*duracion(i) + acelL(i)*duracion(i)^2/2;                
            errorRumbo = 0;
            if geoide.Flattening ~= 0
                %Iteramos para corregir la desviacion de rumbo introducida
                %por el calculo sobre el elipsoide no esferico
                nextPosition = reckon('gc',posicion(ultimo,1),...
                    posicion(ultimo,2), d/N, rumbo(ultimo), geoide);                                        
                errorRumbo = azimuth( 'gc', posicion(ultimo,:),...
                    nextPosition ) - rumbo(ultimo);
            end
            trackAux = track1('gc',posicion(ultimo,1),posicion(ultimo,2),...
                rumbo(ultimo)-errorRumbo, d+d/N, geoide, [], N+2);
            posicion(ultimo+1:ultimo+N,:) = trackAux(2:N+1,:);
            tiempo(ultimo+1:ultimo+N) = tiempo(ultimo) + Tmuestreo*(1:N)';
            vel(ultimo+1:ultimo+N) = ones(N,1)*vel(ultimo);        
            rumbo(ultimo+1:ultimo+N) = azimuth( 'gc',...
                trackAux(2:N+1,:), trackAux(3:N+2,:) );  
            ultimo = ultimo + N;

        else  %Aceleracion constante            
            % distancia recorrida
            d = vel(ultimo)*duracion(i) + acelL(i)*duracion(i)^2/2;   
            errorRumbo = 0;
            % if (geoide(2)~=0)
            %     %Iteramos para corregir la desviacion de rumbo introducida
            %     %por el calculo sobre el elipsoide no esferico
            %     nextPosition = reckon('gc',posicion(ultimo,1),...
            %         posicion(ultimo,2), d/N, rumbo(ultimo), geoide);                                        
            %     errorRumbo = azimuth( 'gc', posicion(ultimo,:),...
            %         nextPosition ) - rumbo(ultimo);
            % end
            trackAux = track1('gc',posicion(ultimo,1),posicion(ultimo,2),...
                rumbo(ultimo)-errorRumbo, d+d/N, geoide, [], N+2);
            posicion(ultimo+1:ultimo+N,:) = trackAux(2:N+1,:);
            di = d/N * (1:N)';            
            dT = -vel(ultimo)/acelL(i) + sqrt( (vel(ultimo)/acelL(i))^2 + 2*di/acelL(i) );
            if imag(dT(N))~=0
                disp('Error, la aeronave no puede pararse en el aire');
            end
            tiempo(ultimo+1:ultimo+N) = tiempo(ultimo) + dT;
            vel(ultimo+1:ultimo+N) = vel(ultimo)+acelL(i)*dT;              
            rumbo(ultimo+1:ultimo+N) = azimuth( 'gc',...
                trackAux(2:N+1,:), trackAux(3:N+2,:) );            
            ultimo = ultimo + N;
 
        end
        
    end    
    
    % Generacion de la altura del tramo
    altura(ultimo-N+1:ultimo,1)=altura(ultimo-N,1)+...
        Vvert(i)*cumsum(diff(tiempo(ultimo-N:ultimo)));
    velVert(ultimo-N+1:ultimo,1)=Vvert(i)*ones(N,1);
end

posSal = [ posicion altura];

%Devuelve el control

return