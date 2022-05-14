function [] = sonar_detect (sp, sai_da_frente, Fs_guedes)
    max=[5000, 5000, 5000, 5000, 5000, 5000, 5000, 600]; %Valores m?ximos aceit?veis para os v?rios sensores
    sonars=pioneer_read_sonars();

    if length(sonars)>8
        sonars=sonars(1:8);
    end
    
    danger4 =5000-sonars(4);
    danger5 = 5000-sonars(5);%Sinal de perigo
    threshold_sonars=4400;    
    
    if danger4>threshold_sonars || danger5>threshold_sonars
       pioneer_set_controls(sp,0,0); %Quando o perigo ? detetado, o robot p?ra
       soundsc(sai_da_frente, Fs_guedes);
       pause(0.3)
       counter = 0;
       while counter < 4
           %Enquanto o valor for maior que o threshold, o robot mant?m se
           %parado
           sonars=pioneer_read_sonars();
           danger4 =5000-sonars(4);
           danger5 = 5000-sonars(5); %Sinal de perigo
           %disp (['sonar reading parado: ' num2str(sonars)])
           if danger4 < 4400 && danger5 < 4400
               counter = counter+1;
           else
               counter = 0;
           end
           pause(0.3)
       end
       
    end
end
