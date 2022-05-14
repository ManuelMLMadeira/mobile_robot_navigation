function angle = OdoToRad (odo_read)
%Recebe angulo do odometro e transforma em angulos em radianos

angle = double(odo_read*2*sym(pi)/4096);

end