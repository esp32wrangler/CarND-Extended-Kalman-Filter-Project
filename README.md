#**Extended Kalman Filter Project**


The goals / steps of this project are the following:
* Complete the missing sections of the Kalman filter, processing both laser
 and radar type measurements
 
* Verify that the algorithm runs correctly with the Term 2 simulator and
 maintains an RMS error of less than 0.11 for the position, and less than 0
 .52 for the speed vectors. 

[//]: # (Image References)

[image1]: ./writeup_images/simulator1.png "Successful run with Dataset 1"
[image2]: ./writeup_images/simulator2.png "Successful run with Dataset 2"

---

## Writeup / README

In `FusionEKF.cpp` I added the initialization code for Laser and Radar
 measurements. If the Kalman filter has not yet been initialized, and the
  first incoming measurement is a laser measurement, then the x vector will
   be initialized with the px, py positions of the measurement.
   If the first measurement is a radar measurement, the polar
    direction and speed coordinates from the radar are first converted to
    the cartesian coordinate system and then used to initialize the x vector.
    
Prediction is common for the laser and radar case, and based on the delay
 from the previous measurement an F and Q vector are initialized and used for
  the calculation.
  
Finally the Update step is separate for the two cases. For laser measurements
 a linear Kalman filter algorithm is used, but for the radar measurements an
  extended Kalman filter needs to be applied, because the conversion of polar
   coordinates to the cartesian coordinates is not a linear operation.
    
The Update steps for both the linear and the extended case are implemented in
 `kalman_filter.cpp`. They are similar in structure, but in the extended case
  a h(x) non-linear function is used to convert polar coordinates to
   cartesian ones, instead of a simple linear mapping with a H matrix. For
    the S, K and P matrix calculations, a Hj matrix is used that approximates
     the non-linear h(x) function at the x_ position with a linear Jacobian
      Matrix.
         
## Testing

The code was compiled in XCode on OSX and tested against the Term2 simulator
 (by the way, the simulator package is missing a +x execute flag on its
  executable, so it was an interesting excercise to get it running).
 
Here are the simulation results on the two datasets:

![image1]
![image2]