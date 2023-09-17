# imu_filter_cpp
A repo to implement non-linear filtering of imu data received via websocket, serialized with protobuffer, for state estimation.

## Cordinate Systems

Datasheet for sensor is [here](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf). We can see our sensor's default coordinate system in the PDF.   
![coordinate_system](assets/bosch_coordinate_system.png)

Note we have mounted the sensor as per below, yielding this system:    
![pre_rotation.png](assets/pre_rotation.png)

Our filter model has the below coordinate system:  
![saito_model_system](assets/saito_model_coordinates.PNG)  
Fig 1, Saito, A., Kizawa, S., Kobayashi, Y. et al. Pose estimation by extended Kalman filter using noise covariance matrices based on sensor output. Robomech J 7, 36 (2020). https://doi.org/10.1186/s40648-020-00185-y

So we want to rotate our sensor's coordinate system to the below to match our filter model. This entails flipping the sign on the z and x axis. 
![post_rotation.png](assets/post_rotation.png)

We can do this with a matrix like the below:
```
-1  0  0
 0  1  0
 0  0 -1
```