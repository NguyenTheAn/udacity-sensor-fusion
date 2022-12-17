# Write a short recap of the four tracking steps and what you implemented there (EKF, track management, data association, camera-lidar sensor fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

1. Extended Kalman Filter (EKF): <br>
I have implemented the EKF for 3D objects. It is included 6 dimensions which are the position and the velocity of the objects. The EKF contains the predict and the update function which is the main of EKF.
2. Track Management: <br>
This module is used for managing the tracking part. It includes creating a new track, deleting an old track, and update the current track
3. Data Association: <br>
This algorithm is used to match the nearest pair of tracks based on Mahalanobis Distance (MHD). It calculates the MHD and finds which one is the closest to the current object and matches them together.
4. Camera-Lidar Sensor Fusion: <br>
This is the final step. It connects the whole system. It calibrated the camera and lidar sensor to the same coordinate.

After doing the project, I learned how I can build an object detection and tracking system based on a lidar sensor and camera. How to process the data and implement the tracking algorithm

While doing this project, I found the most difficult thing to understand the theory of the camera and sensor system. This takes me along time to understand because it's all about math which is my weakness. Also, I have to read through all the sources of the system to find a way to implement the function.

# Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?
Using the camera along with the lidar sensor boots the accuracy of the 3D tracking model. With lidar only, we cannot know about some features such as the color of the car, etc. With more information, we can make the algorithm much more accurate
In my concrete result, I found that when I finished implementing step 4, the accuracy is a significant improvement

# Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

In real life, many conditions in the environment make the system less accurate and make the information to be noisy. In this project, the data is collected in a daylight and clear environment. I will have less noise, but still, there is some noise when running the system

# Can you think of ways to improve your tracking results in the future?
The detection algorithm of this project is less accurate. I think I can change the algorithm to make it better