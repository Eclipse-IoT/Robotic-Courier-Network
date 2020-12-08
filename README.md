# RCN - Robot Currier Network

Doing the walk so you don't have too.

(v1.0.0, Dec 8 2020)

## Business/Use Case

#### What is the problem?



> "Anything that can be automated will be automated, and anything that's left will become 100 times more valuable." - Tariq Qureishy

 To be most effective workers must spend their time focusing on what they do the best. Although seemingly simple, this tends to not always be the case. In the practice of hiring, a skilled worker is selected for the role based on the particular expertise that they bring to the company. Given that [studies show that skilled-labourers who are given tasks which do not require their expertise will be uninspired and less productive](https://www.sciencedirect.com/science/article/abs/pii/0272775787900033) it can be assumed that having them perform tasks below their realm of competence will cause them to become underutilized.  However, in a dynamic factory/warehouse it has become common that one individual often holds the responsibility of a variety of different roles proving it to be difficult for them to focus on the one task they do best. [These additional, often unrelated tasks, increase Job complexity leading to a higher turnover rate.](https://www.researchgate.net/publication/5189573_The_Effect_of_Job_Complexity_on_Job_Satisfaction_Evidence_From_Turnover_and_Absenteeism) 

Furthermore, employees do not only have to worry about completing tasks beyond their job title, but rather many of these added complexities can also be hazardous. Tasks seemingly as simple as moving a package from a workstation to the shipping center can be dangerous. Manual Handling, a task that is almost unavoidable, is the process of picking up and moving heavy loads consistently throughout the day. [This inevitable chore, often left out of the job description,  has become one of the biggest causes of workplace injury.](https://gocontractor.com/blog/workplace-accidents-happen/)



#### What is the solution

The solution to this unwavering issue is not new, but if implemented correctly can be revolutionary. The solution is automation. Ever since the 1930s, and the introduction of the industrial revolution,  [humans have been looking for way to complete tasks automatically](https://www.mentorworks.ca/blog/market-trends/history-of-automation/).  With growing research in computers, AI, edge computing and robotics the idea of automating everything has never appeared so possible. Through leveraging these emerging technologies humans can create complex automated routines to complete tasks that would normally get in the way. As a result of this automation,  humans will be able to switch their focus on what really matters, the original role they have been hired to fill. Human-Robot-Collaboration further helps this idea by additionally improving the effectiveness of human work though the optimization of robotic and AI based tools. Types of Human-Robot-Collaboration range from smart tools which assist the operator in precisely drilling holes in an assembly, to advanced VR and robotic solutions that allow a doctor to perform surgery from a remote location, and everything in between. Through the introduction of human controlled robots, the required manual-handling can be decreased, or completely removed, leaving those employees injury free and inspired to get their job done.



In this demo, we will be utilizing AI, Robotics and Edge computing in order to automate the delivery of packages between two stations in a warehouse. Zenoh is in change of



#### _Why is this important?_

- Many people fear that robots will take away jobs; however, no one looks at the possibility of how robotics can improve quality of life of workers.
- To mitigate workplace injuries
- Improve production yield, while by optimally utilizing employees 
- Properly trained AI already can anticipate raw material and component shortages; automating their delivery will improve productivity

#### _Key benefits_

- Happier employees improve production yield 
- Improve employee utilization
- Prevent Injury by allowing robots to handle work that can hurt people
- Optimize manufacturing flows by ensuring workstations are supplied adequately at all times automatically.



**Keywords/Verticals:** ROS, Industrial Automation, Linux, Performance, Connectivity

## Architecture

#### _Visual representation of the solution._

![signal-2020-12-02-110107](/home/greg/Desktop/working-name/assets/signal-2020-12-02-110107.jpeg)

#### What components is the demo using?

###### Software Components

- **[Zenoh](http://zenoh.io/)**
  - A Lightweight pub/sub protocol that enables fast messaging between machines.
- **[ROS 2](https://index.ros.org/doc/ros2/)**
  - "is a set of software libraries and tools for building robot applications."

**Hardware Components**

- Turtle Bot 3 - Burger

- PC running ubuntu 20.04

- 2 Raspberry pis (must run a 64bit OS)

- 2 9g micro servos

  

## Implementation

**Edge Agent Layer:**

- Zenoh allows for communication between raspberry pi's and other edge servers
- Turtlebot's on board edge computer, relies on a more powerful edge server to perform navigation

## How to run



It is higly suggested that you install a multi-terminal application such as "Terminator" on the ubuntu machine.

1) open up 6 diffrent terminal shells, 3 will be used for the turtlebot, 3 will be used for the stations.

2) open a SSH connection to the turtlebot, station1, station2

3) run the turtle-bot bring-up script

**Turtlebot Bring-up:**

```bash
ros2 launch turtlebot3_bringup robot.launch.py

rviz2 -d `ros2 pkg prefix turtlebot3_navigation2`/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz

ros2 launch nav2_bringup bringup_launch.py map:=/home/greg/roomWithGates.yaml inflation_layer.inflation_radius:=0.01 inflation_layer.cost_scaling_factor:=10.0

#After
#in RVIZ goto Panels > Add New Panel > Navigation 2 > Ok
#now we need to set up the 2D position Estimate
# press 2D position Estimate, and point on the map where the robot is facing in 3D space with a click and drag.
#if this crashes, relaunch and try again!
```

4) clone this repository on the stations (raspbery pi's) and on the ubuntu machine

5) launch stationManager.py on each station

```bash
#Usage: python3 stationManager.py <stationName> <station Cords>

python3 stationManager.py 1 "pose: {header: {frame_id: map}, pose: {position: {x: -2.7, y: -3.3, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: -0.70, w: 0.70}}}"
```

6) run masterEdge.py on the ubuntu machine 

```bash
Python3 masterEdge.py
```










# **Setup Guide**

1. - ROS setup, Tutlebot + PC
     - In this demo we are using ROS Foxy. You must install FOXY on both the controller PC and the Turtlebot SBC.
     - *MAKE SURE TO SELECT FOXY BEFORE FOLLOWING THIS GUIDE:* https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/**

2. - Use SLAM to create a map

     - in-order to perform navigation, we must map out the space the robot will function in

     - Follow this guide: https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node

     - ```
       #To save the final map use:
       ros2 run nav2_map_server map_saver_cli -f ~/map
       ```

3. - Setting up the Stations & Ubuntu computer

     - First we need to get an OS on the pi's

       - since there is no armf build of zenoh at the time of making these documentation we must use a 64bit distro that being https://ubuntu.com/download/raspberry-pi

       - After flashing we must perform a full system update & install eclipse-zenoh

       - ```bash
         #On the stations run
         
         sudo apt-get update
         sudo apt-get upgrade
         sudo apt install python3 python3-pip
         pip install eclipse-zenoh
         ```

       - Lastly we must clone this project

       - ```bash
         sudo apt-get install git
         git clone https://github.com/Eclipse-IoT/Robotic-Courier-Network.git
         cd Robotic-Courier-Network
         ```

       - Now this is when things differ

       - On the stations

       - ```bash
         #Usage: python3 stationManager.py <stationName> <station Cords>
         #example:
         #Read more to see how to get the cords. 
         python3 stationManager.py 1 "pose: {header: {frame_id: map}, pose: {position: {x: -2.7, y: -3.3, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: -0.70, w: 0.70}}}"
         ```

       - On the Ubuntu Machine

       - ```bash
         #Usage: 
         Python3 masterEdge.py
         ```

4. - Getting coordinates from RVIZ
     - Assuming you have already followed the steps above and created a map
     - Launch that map and NAV2 as demonstrated above
     - 2D Pose Goal > Drag to point a vector that describes goal positon > look at the terminal used to launch RVIZ, selected position will be visible > Copy and paste this with quotes around it as a parameter.
