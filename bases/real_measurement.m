function target = real_measurement(target_ideal, radar,...
    bErrDist, bErrAzim, bSesgoDist, deltaAlt, deltaAzim, projection)

Ntracks = length(target_ideal);
Nradar = length(radar);
geoide = projection.geoid;

target = target_ideal;

for j=1:Ntracks
    for k=1:Nradar
        
        ind = find ( target_ideal(j).measure(:,1) == radar(k).id );        
        N = length(target(j).measure(ind,3));
        
        %Incorporacion de errores a las medidas:
        %Sistematicos:
        if (bSesgoDist) %sesgoDist es cero o uno
            if(k==1)
                errsDist =2*75;
            else
                errsDist =-2*75;
            end
        else
            errsDist = 0;
        end
        if(k==1)
            errsAzim = deltaAzim;
        else
            errsAzim = -deltaAzim; 
        end
            errsAlt = deltaAlt;

        %Aleatorios:
        if (bErrDist)
            erraDist = radar(k).resDist* randn(N,1);
        else
            erraDist = 0;
        end
        if (bErrAzim)
            erraAzim = radar(k).resAzim*randn(N,1);
        else
            erraAzim = 0;
        end
        erraAlt = 5*randn(N,1);

        target(j).measure(ind,3) = target_ideal(j).measure(ind,3) + errsDist;
        target(j).measure(ind,4) = mod(target_ideal(j).measure(ind,4) + errsAzim,360);
        target(j).measure(ind,5) = target_ideal(j).measure(ind,9) + errsAlt; 

        target(j).measure(ind,6) = target(j).measure(ind,3) + erraDist;
        target(j).measure(ind,7) = mod(target(j).measure(ind,4) + erraAzim,360);
        target(j).measure(ind,8) =(unitsratio('m','feet')*100)*...
            round((target(j).measure(ind,5) + erraAlt)/(unitsratio('m','feet')*100));  %Anadimos la cuantificacion de 100 pies en altura
        if(k==1)
        %target(j).measure(ind,2)=target(j).measure(ind,2)+4; %Sesgo de
        %tiempo en el radar 1
        target(j).measure(ind,2)=target(j).measure(ind,2);  %Sin sesgo de tiempo
        end 
        
        %elevacion
        target(j).measure(ind,9) = elevation2(target(j).measure(ind,6),...
            target(j).measure(ind,8),radar(k).posGeod,geoide) ;
        
        %coordenadas geodesicas
        target(j).measure(ind,10:12) = radar2geodetic(target(j).measure(ind,6),...
            target(j).measure(ind,7),target(j).measure(ind,9),...
            radar(k).posGeod) ;
        
        %coordenadas estereograficas
        [target(j).measure(:,13) target(j).measure(:,14)] = stereo(...
            projection,target(j).measure(:,10),target(j).measure(:,11),...
            'surface','forward');
        
        %Calculo de la matriz de covarianza en coordenadas estereogr�ficas
        target(j).mcov(1,1,ind)=(radar(k).resDist.*sin(pi/180*target(j).measure(ind,7))).^2+...
            (radar(k).resAzim*pi/180*target(j).measure(ind,6).*cos(pi/180*target(j).measure(ind,7))).^2 ;
        target(j).mcov(1,2,ind)=(radar(k).resDist.^2-(radar(k).resAzim*pi/180*target(j).measure(ind,6)).^2).*...
            sin(2*pi/180*target(j).measure(ind,7))/2 ;
        target(j).mcov(2,1,ind)=target(j).mcov(1,2,ind);
        target(j).mcov(2,2,ind)=(radar(k).resDist.*cos(pi/180*target(j).measure(ind,7))).^2+...
            (radar(k).resAzim*pi/180*target(j).measure(ind,6).*sin(pi/180*target(j).measure(ind,7))).^2 ;
        
        %velocidad (damos la salida en coordenadas sur-norte, peste-este)
        %con adicion de ruido segun lo indicado en la estructura del radar

        if (radar(k).VelMS==1)
            aux1=radar(k).StVel*randn(length(ind),1)/sqrt(2);
            aux2=radar(k).StVel*randn(length(ind),1)/sqrt(2);
            target(j).measure(ind,15)=target_ideal(j).measure(ind,12).*sin(target_ideal(j).measure(ind,13)*pi/180)+aux1;
            target(j).measure(ind,16)=target_ideal(j).measure(ind,12).*cos(target_ideal(j).measure(ind,13)*pi/180)+aux2;
        else
            target(j).measure(ind,15)=zeros(length(target_ideal(j).measure(ind,12)),1);
            target(j).measure(ind,16)=zeros(length(target_ideal(j).measure(ind,12)),1);
        end
               
        
    end
end