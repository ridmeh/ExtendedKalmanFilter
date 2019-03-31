# ExtendedKalmanFilter


Setup
=====

This is my first project so setup took time. I attempted to start with ubuntu install on windows machine and that went fine with guides provided by Udacity. I figured out issues of build and connecting to simulator in ubuntu with class help. However, I had issues in using ubuntu for debugging without proper IDE. I later installed Eclipsce C++ which had its own set of problems. Lastly, what started working is MS Visual studio Community edition. I would use windows enviorment to develop and test with data file available and later move code to ubuntu to test using simulator. 

Sensor Fusion Algorithm:
========================

Extended Kalman Filter helps us fuse data from various sensors like RADAR and LIDAR. This algorithm blends data from both the sensors beautifully although care has to be taken in doing co-ordinate conversions. Attached implementation data is feed via simulator and algorithm predicts

 Whenever a measurement ( RADAR / LIDAR ) comes:
 {
    if ( first measurement ) 
      { initialize the state }
    else 
      { Predict 
        Update the state
      }  
 }
  
Notes
======

Following are helpful notes tht 
1. Initialization of previous_time variable : At the initialization step it is important we set the previous_time. If it is retained to 0, it throws off calculation which do not recover.
2.  LIDAR and RADAR data feeding: RADAR is in polar co-ordinate and need to be converted to cartisean co-ordinate. Also, RADAR data has 3 measurements and you will face issues if wrong # of data points are fed.
3. Normalization : Just re-iterating what is already a requirement. We need to handle phi > M_PI OR phi < -M_PI

RMSE
====

The lowest observed RMSE: 
vx = 0.495963
vy = 0.413815
The maximum observed RMSE:
vx = 4.19994
vy = 2.07675
