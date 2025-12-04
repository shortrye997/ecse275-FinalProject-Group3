# Self-Driving Smart Car (Coppelia Waymo)

## Team Members & Roles
+ Ryan Shorter: 
  - Obstacle Logic
    - Created and arranged obstacles in roadmap simulation in multiple different scenes
    - Developed logic for vehicle's interaction with obstacles
    - Programmed path-planning to reroute A-Star accounting for new obstacles
+ Joshua Kim:
  - Sensor/Vehicle Logic
    - Implemented sensor data to determine relevancy of obstacles
    - Controlled robot reaction and response to changes in environment
    - Experimented with varying measurable robot-vision thresholds
  - Project Write up
+ Peyten Hargraves:
  - Sensor Implementation
    - Programmed LiDAR to collect angle and distance data of obstacles
    - Tested various situations of collecting LiDAR data
    - Built the robot to mount necessary sensors
    - Implemented RGB camera to integrate color into the LiDAR return data
+ Yechan Kim:
  - Developed A-Star
    - Designed CoppeliaSim Environment and associated A-Star programming
    - Integrated A-Star as a function to be reactive
  - Generated plots 

### Introduction
Our project is inspired by the challenges of self-driving smart cars. With an unpredictable world around them, with so many different variables and objects to account for, how do they perform their function while effectively ensuring the safety of not only the user, but the world around them as well? Products such as Waymo, use a complex network of cameras, LiDAR sensors, radar, as well as mapping, smart software, GPS and more to effectively analyze the world around them and react accordingly. 

In our project, we look to simulate this concept using various ECSE275 robotics concepts. The two we will focus on will be utilizing robot vision, to add a level of complexity in identifying the robot's surroundings, as well as A-Star but in a more reactive context as well as map-based planning.

Summarize the final deliverable (i.e. what you will eventually demo)

### Approach
We had four main building blocks that build up our system: Sensors, A-Star, Robot Logic, and Obstacle Logic, with A-Star being the backbone of our project. It dictated the initial position and movement of the robot. The sensor system would then analyse the world and continuously pass color, distance, and angle data to Obstacle Logic. Obstacle Logic would then determine what sets of data were relevant to act upon, which when true, would pass information to Robot Logic which will change the robot's behavior, which can be generalized to avoiding obstacles and moving to a point optimal for a new path. It would then re-run A-Star with taking into account the obstacle it encountered, which means Robot Logic is passing the new situation to A-Star, which will re-run and set the robot on a new course. 

We considered between using A-Star and Potential fields to reach our goal. Considering the benefits and costs of map-based planning and reactive planning as applied to our goals, we ended up deciding to use A-Star, as it would better represent a car traveling through a city using GPS. However, unexpected obstacles would necessitate for the path-planning to be reactive as well, which A-Star is not. To account for this, we developed a system that would autonomously reiterate A-Star, while updating the existing map that takes into account new data. 

When it comes to "seeing" the world around the robot, we do not have the resources in CoppeliaSim to utilize sensor networks to classify if something is a green traffic light, a child crossing the road, or a bush. However, to simulate this vision, we have three different obstacles with different colors. Blue will represent roadblocks that point the car away from the restricted area. Green and red obstacles will represent safe and dangerous obstacles respectively. Green obstacles, like standalone trees, will not be a concern when on the side of the road. However, red obstacles will represent a risky obstacle, like a child.

What experiments did you conduct and what data did you collect to measure the performance and define success?

<img width="1320" height="810" alt="image" src="https://github.com/user-attachments/assets/37c0f7a7-bd27-4fad-8246-e8959ac0c0df" />


### Results


### Conclusion


