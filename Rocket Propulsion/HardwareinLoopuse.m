function [] = HardwareinLoopuse()
% HARDWARE-IN-LOOP ROCKET FLIGHT SIMULATION.
% AEROSPACE ENGINEERING FINAL YEAR PROJECT AT SRM UNIVERSITY. 
% Send pitch and roll values to the hardware model via serial.

%Create Port for communication with arduino
s = serial('COM16','BaudRate',9600);

%Open Port , start communication
fopen(s);

p = 78;
k = 1;
y = 10;

fprintf(s,['$10,78,1,']);
disp('1');
pause(7);
%fscanf(s);

fprintf(s,['$10,48,1,']);
disp('2');
pause(7);
%fscanf(s);

fprintf(s,['$10,18,1,']);
disp('3');
pause(7);
%fscanf(s);

fprintf(s,['$10,10,1,']);
disp('4');
pause(7);
%fscanf(s);

fprintf(s,['$10,-18,1,']);
disp('5');
pause(7);
%fscanf(s);

fprintf(s,['$10,-28,1,']);
disp('6');
pause(7);
%fscanf(s);

fprintf(s,['$10,-38,1,']);
disp('7');
pause(7);
%fscanf(s);

fprintf(s,['$10,-45,1,']);
disp('8');
pause(7);
%fscanf(s);


fclose(s);



% while k == 1
%     fopen(s);
%     fprintf(s,['$%d,%d,1,'],[p y]);
%     p = p - 13;
%     y = y - 0.5;
%     pause(3);
%     if p < 0
%         k = 0;
%     end
%     fclose(s);
% end
% 
% while k == 0
%     fopen(s);
%     fprintf(s,['$%d,%d,1,'],[p y]);
%     p = p - 57.;
%     y = y - 1;
%     pause(3);
%     if p < -45
%         k = 2;
%     end
%     fclose(s);
% 
%     
% end

% fclose(s);

end