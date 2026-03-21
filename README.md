# ME-405-Term-Project-Chamberlin-Zhang
## Overview

This project involves the design and implementation of an autonomous line-following robot using the Romi platform. The system integrates mechanical design, electrical hardware, and embedded software to navigate a predefined track.

The robot uses a reflectance sensor array for line detection, encoders for wheel feedback, and an IMU for state estimation. A multitasking control system was implemented to manage sensor input, motor control, and course navigation.

The goal of the project was to achieve reliable and repeatable performance while maintaining a simple and modular design.

## Mechanical Design
 ### See Mechanical section for Information on:
Drive System

Sensor Placement

Mounting

Stability

Design Considerations

Robot Base

Microcontroller/Development board

Drive System

Bump Sensor

Fasteners

Array Sensor

IMU

## Electrical Design
### See Electrical section for information on:
Microcontroller

Motor Control System

Encoders

Sensor Integration

Power Distribution

Wiring Layout

## Hardware Overview 
### For more information on hardware, refer to the Mechanical section
<img width="830" height="623" alt="Romi_FrontView_Labels" src="https://github.com/user-attachments/assets/3c06af01-4d21-4298-aabf-135a6bf4c737" />


## Wiring Diagram
See Below for wiring Dirgrams
<img width="2000" height="1300" alt="romi_final_wiring_diagram" src="https://github.com/user-attachments/assets/c9755791-b81e-45a8-8f98-fb7529009119" />
[Electrical_Romi Cables.pdf](https://github.com/user-attachments/files/26152373/Electrical_Romi.Cables.pdf)

## Task Diagram
<img width="2000" height="1200" alt="task diagram" src="https://github.com/user-attachments/assets/8a9f50b0-ee7e-4203-a6dc-50dc5facbfba" />

## FSM Diagrams

### Line Follow Task
<img width="1900" height="1150" alt="fsm_line_follow" src="https://github.com/user-attachments/assets/3c401413-8dbc-44bc-9a86-c4ee7c1c6637" />

### Motor Task
<img width="1950" height="1200" alt="fsm_motor" src="https://github.com/user-attachments/assets/88ea9727-2f76-451b-ad17-0be89db08628" />

### State Estimation Task
<img width="2050" height="1200" alt="fsm_state_est" src="https://github.com/user-attachments/assets/46e0ec02-cc53-40da-9aa8-6e34d1fbcc9c" />

### Track Task
<img width="2400" height="1550" alt="fsm_track" src="https://github.com/user-attachments/assets/b8e3d22c-a871-4253-b2a3-17f641611fc6" />

### User Task
<img width="2050" height="1300" alt="fsm_user" src="https://github.com/user-attachments/assets/e5eefbd3-c1e1-41cd-9bbf-de83d45badc2" />

## Software Design
add any nuances our code has or you can say "refer to scripts README" or something
## Results
### Demo Video
Below is the link to our results video. It begins with three successful test runs from the night before the presentation and ends with the three DNF runs from presentation day. 
https://youtu.be/jFiXQZCSne8?si=h6S_fZmUDP1rZtpE
### Results Discussion
During testing conducted the night prior to the in-class demonstration, the Romi exhibited a consistent and repeatable performance. Across three consecutive trials, the Romi successfully completed the track with nearly identical times. The first trial, conducted at 10:25 PM on March 15th, resulted in a completion time of 1 minute and 53 seconds. The second trial at 10:28 PM also resulted in a time of 1 minute and 53 seconds, and the third trial at 10:31 PM resulted in a slightly faster time of 1 minute and 52 seconds. These results indicate that the system was well-tuned and capable of reliable operation under consistent conditions.

However, performance during the in-class presentation differed significantly from the previous night’s trials. On March 16th, the robot failed to complete the track in all three attempts. The first two trials, at 1:37 PM and 1:39 PM, both ended prematurely at approximately 26 seconds when the Romi failed to make it through teh "parking garage". The third trial at 1:40 PM lasted longer but still failed to complete the course, stopping at approximately 1 minute and 8 seconds. This inconsistency contrasted sharply with the repeatability observed during pre-presentation testing.

Upon reflection, a key difference between the two testing conditions was the type of batteries used. The night before, the system was tuned using standard Duracell batteries with a nominal voltage of 1.2 V per cell. On presentation day, rechargeable batteries were used instead, with a higher nominal voltage of approximately 1.4 V per cell. Although this difference may seem small, the increased voltage likely messed with the control tuning developed during previous trials. This change in conditions introduced variability that the system was not designed to accommodate, leading to poor Romi performance and failure to complete the track.

Overall, the results demonstrate that the robot is capable of highly consistent and successful operation when properly tuned and tested under stable conditions. However, they also highlight the sensitivity of the system to changes in electrical input, particularly battery voltage, and emphasize the importance of maintaining consistent testing conditions between tuning and final demonstration.
## Reflection
One of the most significant lessons learned during this project was the importance of consistency in testing conditions, particularly with respect to power supply. As discussed before, the robot was tuned using standard Duracell batteries, but on presentation day, rechargeable batteries with a higher nominal voltage were used instead. This seemingly small difference had a noticeable impact on system behavior. Because the control parameters were tuned for a lower voltage input, the higher voltage effectively pushed the system outside of its stable operating range. This highlighted how sensitive embedded control systems can be to changes in electrical input and emphasized the need to either regulate voltage or retune the system for different operating conditions.

Another major limitation of the system was the use of a relatively narrow reflectance sensor array. While the sensor functioned correctly, its limited width reduced the robot’s ability to detect the line when deviations became larger. As a result, the control system had to be tuned conservatively, requiring the robot to move at slower speeds to maintain reliable line tracking. At higher speeds, the robot was more likely to lose the line entirely because the sensor could not capture enough positional information quickly enough. This tradeoff between speed and reliability demonstrated the importance of sensor selection and placement in control system design. In order to make the line following work, the period had to be significantly raised which made the robots movements extremely twitchy, which also mde the sensor lose the line in certain trials. This also caused the group to have to tune more significantly to get the Romi to make it around the track.

A particularly challenging aspect of the project was integrating state estimation with the existing line-following control system. While both systems worked independently, combining them introduced unexpected issues where enabling state estimation caused the line-following performance to fail. This required significant debugging and testing to resolve. Ultimately, the issue was addressed, but the process revealed how interconnected subsystems can introduce unintended interactions, especially when sharing data and timing resources. This reinforced the importance of modular design, careful signal management, and incremental integration when developing complex embedded systems.

Overall, this project demonstrated that successful robotic performance not only depends on individual task functionality, but also getting the operating conditions to function consistantly and the interaction between mechanical, electrical, and software components needs to be looked into as each component is added. While the final system showed strong performance under controlled conditions, the challenges encountered provided insight into the importance of tuning, limiting variable conditions, and leaving a significant amount of time for debugging and fixing a system that might varry.

