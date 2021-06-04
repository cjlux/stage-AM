
* stepper_motor/
    * `stepperMotor_test`: Programm to test the A4988 driver. This chip is devoted to drive one stepper motor. 
    * `continuousSpeed`: driver for the CartPole experience with continuous speed.  
    => Do not run this programme with the motor of the CartPole bench,  it wil move beyond the bench limits! Use this programme with a free stepper moteur (linked to nothing).
    * `continueousSpeed_bench_test`: Make the cart move along the silder. Run test with the carpole stepper-motor: the motor moves right over 400 steps, then it moves left until it reaches 400 steps, and the moves right to reach the position 0 step. The program xaits for the user to hit the ENTER key to start.
    
