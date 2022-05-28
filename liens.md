# LIENS IMPORTANTS :

## Projet **YABR** 

Your Arduino Balancing Robot : http://www.brokking.net/yabr_main.html
vidéos :<br>
Part 1 : https://youtu.be/6WWqo-Yr8lA<br>
part 2 : https://youtu.be/VxpMWncBKZc<br>
Part 3 : https://youtu.be/mG4OtAiY_wU

Les docs YABR : http://www.brokking.net/yabr_downloads.html

video about PID controllers for quadcopters https://youtu.be/JBvnB0279-Q

MPU-6050 IMU accelerometer and gyro fusion:<br>
part 1 : https://youtu.be/4BoIE8YQwM8<br>
part 2 : https://youtu.be/j-kE0AMEWy4

## DRL

How to configure the reward function : https://medium.com/@BonsaiAI/deep-reinforcement-learning-models-tips-tricks-for-writing-reward-functions-a84fe525e8e0

## PID

"Ultimate Guide to Make Self Balancing Robot for Beginner":<br>
How to configure K_p,K_I,K_d : https://www.youtube.com/watch?v=CON0sWNDUco

## Balancing robots :

* théorie du balancing robot : https://www.diva-portal.org/smash/get/diva2:916184/FULLTEXT01.pdf

* "DIY Self Balancing Robot" (bois, moteurs PAP, MPU-6050)<br>
https://electricdiylab.com/diy-self-balancing-robot/

* projet robot a deux roues B-Robot<br>
http://cienciaycacharreo.blogspot.com/2013/10/b-robot-un-robot-equilibrista-impreso_28.html

* VertiBot : projet de robot type pendule inversé<br>
https://positron-libre.blog/robots/pendule-inverse-vertibot.php

* Cornell University: "Self-balancing Robot"<br>
https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/index.html<br>
lire dans la partie _Mathematical Background_ les exxplications sur le __complementary filter__ (filtre issu de l'algorithme de fusion des données accéléromètre et gyroscope) dans la partie _Invensense MPU-6050 IMU_ les explications intéressantes sur le MPU 6050<br>
L'article donne le lien vers https://eu.mouser.com/applications/sensor_solutions_mems/ qui est la source des explications de l'__algo de fusion__ et du __complementary filter__... avec des infos intéressantes à lire sur :
    * comment mesurer l'angle d'un balancing robot avec juste un accéléromètre ou avec un gyroscope et le complentary filter
    * le principe de l'asservissement de l'équilibre avec un contrôle PID
  
 * Dans cet article https://www.instructables.com/Self-Balancing-Robot/ on trouve les deux types de filtres possibles pour intégrer correctement l'angle d'une MPU :
    * __complementatry filter__ : https://bayesianadventures.wordpress.com/2013/10/20/gyroscopes-accelerometers-and-the-complementary-filter/
    * __kalman filter__ (le nec + ultra !) : <br>
https://github.com/TKJElectronics/KalmanFilter<br>
https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU/MPU6050/Kalman.h<br>
http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

* Chillibasket propose un tuto en 5 parties, assez bien documentées sur MPU6050 et PID <br>
  https://wired.chillibasket.com/2015/03/pid-controller/
  https://wired.chillibasket.com/2014/10/accel-gyro-sensors/
  https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
  https://wired.chillibasket.com/2015/03/pid-controller/
  https://wired.chillibasket.com/2015/10/putting-it-all-together/



How to make self balancing robot : https://www.youtube.com/watch?v=6aMrpmV17mU

## MPU

* généralités sur les MPU : https://blog.generationrobots.com/fr/imu-et-robotique-ce-quil-faut-connaitre/


## MPU-6050

* Doc ARDUINO : MPU-6050 Accelerometer + Gyro https://playground.arduino.cc/Main/MPU-6050/#info<br>
où l'on trouve cette info : <br>
  
  
* **Jeff Rowberg** has done an excellent job.
  See his I2C lib: http://www.i2cdevlib.com/devices/mpu6050<br>
  Or the latest code on GitHub: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

  Contribution de Louis Rodenas -- janvier 2014<br>
  _Arduino Sketch to automatically calculate MPU6050 offsets_<br>
  https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/

* "Arduino Uno and the InvenSense MPU6050 6DOF IMU": https://42bots.com/tutorials/arduino-uno-and-the-invensense-mpu-6050-6dof-imu/<br>
mise en oeuvre du Digital Motion Processor (DMP) du MPU-6050 avec les bibliothèques **I2Cdev** et __MPU6050__ de Jeff Rowberg.<br>
__nota__ ce tuto montre comment calibrer et utiliser un MPU6050 : on peut suivre leur protocole, ça marche, sauf qu'il faut utiliser les bibilothèques __MPU6050.zip__ et __I2Cdev.zip__  "à jour" sur https://github.com/jrowberg/i2cdevlib et dont j'ai copié les zip dans `code_composants/MPU6050/`.<br>
L'article cite aussi<br>
    * _some additional fine-tuning of the offsets in the sample code of the MPU-6050_<br>
       https://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/ <br>
       Le fichier attaché dans le forum cité n'est plus accessible, on peut le retrouver ici:<br>
       https://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/
    * _An Arduino self-balancing robot: working prototype_
       https://42bots.com/showcase/an-arduino-self-balancing-robot-working-prototype/

* Debra <br>
  "Gyroscopes and Accelerometers on a Chip" -- mars 2013<br>
  http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/<br>
  Explique bien le *complementary filter* avec un MPU6050.

  "MPU-6050: DMP Data from i2cdevlib" -- juin 2013<br>
  https://www.geekmomprojects.com/mpu-6050-dmp-data-from-i2cdevlib/

  "MPU-6050 Redux: DMP Data Fusion vs. Complementary Filter" -- janvier 2014<br>
  https://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/<br>
  Mentionne comment régler le PB du FIFO overflow : https://www.i2cdevlib.com/forums/topic/27-fifo-overflow/

  "New IMU Library for Arduino- RTIMULib" -- avril 2015<br>
  https://www.geekmomprojects.com/new-imu-library-for-arduino-rtimulib/

* Dejan : "Arduino and MPU6050 Accelerometer and Gyroscope Tutorial"<br>
https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/


### tutos CoppeliaSim :

https://www.youtube.com/watch?v=YcfARpQVKhU<br>
https://www.youtube.com/watch?v=i0a7K0zAOX0<br>
https://www.youtube.com/watch?v=vRi2Up0yfyk<br>

### Divers

Build an Electronic Level with MPU-6050 and Arduino<br>
https://www.youtube.com/watch?v=XCyRXMvVSCw

Gymbal lock<br>
