# Line Follower Project - Robotics Hackathon

This final project developed during the Introduction To Robotics course consists of building, assembling and programming a robot whose objective is to follow a black line on a track and complete it as fast as possible. In order to do so, me and my colleagues spent 12 hours at the university, where we developed our robots in teams of three. 

My team - _Powerpuff Girls_:
- [@lemnaruamedeea](https://github.com/lemnaruamedeea)
- [@Diana5B](https://github.com/Diana5B)

## Project and Design Description

### Project components:
- Arduino Uno
- Power source: LiPo battery
- Two wheels
- QTR-8A reflectance sensor
- Ball caster
- L293D motor driver
- Two DC motors
- Medium breadboard
- Wires (M - F, M - M), zip-ties, screws as needed

For the chassis we cut into a foam board after measuring an apropriate distance between the wheels and for the sensor. We improvised "pockets" for the breadboard and battery and we secured the Arduino board and the motors using zip-ties.

### Requirements

Minimum requirements for the project were to have the robot finish the line follower track, including the curved lines. To achieve maximum grade, the course had to be finished in under 20 seconds and, upon starting, the sensor had to be calibrated using automatic motor movement.

### Software implementation

At first, we implemented automatic motor movement so that the robot could calibrate its sensor by moving left and right in order to recognize the black line it had to follow. The movement behaviour was determined by using a proportional-integrative-derivative controller. We started with a simple code provided by our teacher in which we had to alter the kp, ki, and kd values to achieve the desired movement. We started by assigning random values to the proportional constant in order to observe the behaviour. Once the robot was able to take the turns without overshooting, we began updating the kd value to smooth the wobble. 

The final values were:
- kp = 4.3;
- ki = 0.000;
- kd = 23.2;

Afterwards, we juggled the tresholds for the error and motor speed values.

## Performance and outcome

Final setup of the robot: 
![Robot](https://github.com/vfranci/Line-Follower/assets/115077321/08085af5-6bf5-4b55-920f-c0e0b4081a76)

When presenting our design, the lowest time recorded for completing the track was 18... seconds. 

Performance video: https://youtu.be/dwM-u17O9Z4?si=YvZB7Q0n8osCDkEj 



