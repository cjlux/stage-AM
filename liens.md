
### Balancing robots :

* bois + moteurs pas à pas + MPU-6050 : https://electricdiylab.com/diy-self-balancing-robot/
Programme Arduino Centrale Inertielle : https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/

* Another Arduino project - Your Arduino Balancing Robot (YABR) - Part 1 : [video](https://www.youtube.com/watch?v=6WWqo-Yr8lA)



### MPU-6050
* [Arduino Uno and the InvenSense MPU6050 6DOF IMU](https://42bots.com/tutorials/arduino-uno-and-the-invensense-mpu-6050-6dof-imu/) : mise en oeuvre du Digital Motion Processor (DMP) du MPU-6050

* Cornell University : [Self-balancing Robot](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/index.html)<br>
lire dans la partie _Mathematical Background_ les exxplications sur le __complementary filter__ (filtre issu de l'algorithme de fusion des données accéléromètre et gyroscope) dans la partie _Invensense MPU-6050 IMU_ les explications intéressantes sur le MPU 6050<br>
L'article donne le lien vers https://eu.mouser.com/applications/sensor_solutions_mems/ qui est la source des explications de l'__algo de fusion__ et du __complementary filter__...

* des infos intéressantes à lire sur :
  * comment mesurer l'angle d'un balancing robot avec juste un accéléromètre ou avec un gyroscope et le complentary filter
  * le principe de l'asservissement de l'équilibre avec un contrôle PID
  
 * Dans cet article https://www.instructables.com/Self-Balancing-Robot/ on trouve les deux types de filtres possibles pour intégrer correctement l'angle d'une MPU :
    * __complementatry filter__ : https://bayesianadventures.wordpress.com/2013/10/20/gyroscopes-accelerometers-and-the-complementary-filter/
    * __kalman filter__ (le nec + ultra !) : <br>
https://github.com/TKJElectronics/KalmanFilter<br>
https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU/MPU6050/Kalman.h<br>
http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

### tutos CoppeliaSim :

https://www.youtube.com/watch?v=YcfARpQVKhU<br>
https://www.youtube.com/watch?v=i0a7K0zAOX0<br>
https://www.youtube.com/watch?v=vRi2Up0yfyk<br>

