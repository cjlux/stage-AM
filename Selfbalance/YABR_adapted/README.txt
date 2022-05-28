## Différences avec les fichiers YABR d'origine:

### Balancing_robot.ino

- ligne 16:
    now: int acc_calibration_value = 1330; 
    was: int acc_calibration_value = 1000;
    
- ligne 19
    now: float pid_p_gain = 25 ;
    was: float pid_p_gain = 15;
   
- ligne 82-85
    now:
      pinMode(3, OUTPUT);
      pinMode(4, OUTPUT);
      pinMode(5, OUTPUT);
      pinMode(6, OUTPUT);
    was:
      pinMode(2, OUTPUT);
      pinMode(3, OUTPUT);
      pinMode(4, OUTPUT);
      pinMode(5, OUTPUT); 
      
- ligne 123-128 commentées
  //battery_voltage = (analogRead(0) * 1.222) + 85;
  
  //if(battery_voltage < 1050 && battery_voltage > 800){
  //  digitalWrite(13, HIGH);                           
  // low_bat = 1;                                       
  //}

- lignes 269-300:
    moteur ne tourne pas dans le même sens
    moteurs sur PORTD pin 5,6 et 3,4 au lieu de pin 3,2 et 5,4
