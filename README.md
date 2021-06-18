## Theme - Sahayak Bot (Assistance Robot)

The theme that we got was Sahayak Bot. We had to program an Autonomous Ground Vehicle (AGV) to make it capable of autonomously traversing an indoor environment to assist moving objects from one place to another. The scenario was that objects were being moved in e-Yantra’s lab and they were short of manpower so Sahayak bot was used to help move the boxes from one place to another.

Challenges in this theme inlcuded: 2D mapping, 3D mapping, Autonomous Navigation, Perception, Pick and Place.

<p align="center">
<img src="SB1.png" alt="SB"
	title="Sahayak Bot" width="200" height="250" />
</p>
<p align = "center">
Fig. - Sahayak Bot
</p>

<!-- ![SB1](https://user-images.githubusercontent.com/52562790/122603360-d65a2780-d091-11eb-8ff2-48432c7481fe.png) -->

Our team's performance:

|               | Objectives                                                                                                                                                                                                                                                                                                                                                                  | Score                                             | ~ No. of teams who completed the task |
|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------|:-------------------------------------:|
| <b>Task 0</b> | To get familiar with Ubuntu & ROS                                                                                                                                                                                                                                                                                                                                           |                      Accepted                     |                  330                  |
| <b>Task 1</b> | To get an overview of Gazebo                                                                                                                                                                                                                                                                                                                                                |                     87.33/100                     |                  270                  |
| <b>Task 2</b><br />[Video Submission Link](https://youtu.be/INFCEz6ewPM) | To explore Mapping and Navigation in ROS                                                                                                                                                                                                                                                                                                                                    |                     90.05/100                     |                  179                  |
| <b>Task 3</b><br />[Link](https://youtu.be/8W77nu1Geig) | To explore Robotic Arm Manipulation in ROS using MoveIt motion planning framework                                                                                                                                                                                                                                                                                           |                      100/100                      |                  139                  |
| <b>Task 4</b><br />[Link](https://youtu.be/qrt8uVUZWl0) | To perform Perception                                                                                                                                                                                                                                                                                                                                                       |                      100/100                      |                   70                  |
| <b>Task 5</b><br />[Link](https://youtu.be/hcRuW85iM1o) | To collaborate learnings of all the previous tasks ie.<br />- Navigate to the correct location to pick up objects  using the Navigation pipeline<br />- Detecting objects using perception pipelines<br />- Grasping objects with MoveIt planner<br />- Navigate again to your destination using the Navigation pipeline<br />- Placing the objects in the correct drop box |                     31.42/100                     |                   45                  |
| <b>Task 6</b><br />[Original](https://youtu.be/9-B2YUj1zHM)<br />[Bonus](https://youtu.be/cYznWN5Cslk) | Final task Similar to task 5 with added constraints like 48 hours time-limit, original and bonus configuration                                                                                                                                                                                                                                                              | Original 625.8<br/>Bonus 151.67 <br/>Code 100/100 |                   41                  |
| <b>Result</b> | We couldn’t get shortlisted for Finals of the competition. e-Yantra has not released overall rankings but we calculate that we are in top 25. Top 7 teams were selected for the Finals. The competition was a good learning experience.                                                                                                                                     |                                                   |                                       |

## Prerequisites
- ROS
- Gazebo
- MoveIt
- FindObject2d
- Navigation packages(Gmapping, amcl, map server, tf2)
- Eyantra's original [repo](https://github.com/vishalgpt579/sahayak_bot) which contains the original packages

This repository contains the additional ROS packages we built and used to complete the tasks.

## Packages
- ebot_nav - 2D mapping and navigation 
- ebot_perc - Arm manipulation and perception
- ebot_task5 - Main controller

## Task 6 Original Configuration
There are total five scripts that are used:
1) /ebot_task5/scripts/SB#637_main2_task6.py - Main script that controls the bot
2) /ebot_nav/scripts/SB#637_move_base_task5.py - Navigational control
3) /ebot_perc/scripts/SB#637_moveit_perc.py - Perception and Arm control
4) /ebot_task5/scripts/SB#637_img_dis.py - To display required images with object names and bounding boxes
5) /ebot_task5/scripts/SB#637_mainr.py - To meet the remaining output terminal print requirements

All the scripts communicate with each other with the help of topics.
The general approach in our solution is that:
- First we take the required objects and destinations in list. 
- Find out from where these objects can be picked. Maintain its list.
- Main script then tells the move base node to go to a particular room
- Once reached, main script tells moveit script to detect the objects from that
room. Once detected, main script gets to know where the required is kept.
- Then it sends the location to move base node from where the object can be picked up
- Once reached, it tells the moveit script that this is the object that needs to be
picked up now. Moveit performs its operations and after picking the required object,
informs the main script.
- Once picked, main script sends the destination room goal to the move base node.
- Then, when the ebot has reached its destination, main script commands the moveit to
drop the object in the dropbox.
- Then the whole sequence is repeated if more objects are required.
- Along with all this, the other requirements of the rulebook are met by the other two scripts.

To stay on a particular position, drift neutralization is also used.
For perception, a faithful session in find_object_2d was created and used.