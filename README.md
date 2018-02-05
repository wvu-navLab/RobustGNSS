# RobustGNSS

This repository contains a modified version of [GTSAM](https://bitbucket.org/gtborg/gtsam), which has been modified for GNSS data processing. A detailed description of the modification made can be found in ["Robust Navigation In GNSS Degraded
Environment Using Graph Optimization"](https://www.researchgate.net/profile/Ryan_Watson7/publication/320084321_Robust_Navigation_in_GNSS_Degraded_Environment_Using_Graph_Optimization/links/59cd10ef0f7e9b6e147906ec/Robust-Navigation-in-GNSS-Degraded-Environment-Using-Graph-Optimization.pdf) This software has been cleared for public release by the USAF Case # 88ABW-2017-3893


## How to Install 


### 1) Requirements/Recommendations 

#### Required 
* Boost -->  ```` sudo apt-get install libboost-all-dev ````
* CMake -->  ```` sudo apt-get install cmake ````

#### Recommended 
* Intel TBB -->  ```` sudo apt-get install libtbb-dev ````
* [Intel MKL](https://software.intel.com/en-us/mkl)



### 2) Clone repository to local machine  
* RobustGNSS --> ```` git clone https://github.com/wvu-navLab/RobustGNSS.git ````


### 3) Build 

````bash 

cd RobustGNSS/gtsam; 
mkdir build;  
cd build;
cmake ..
make

````

### 4) Test 

Contained within the __RobustGNSS/gtsam/gnssExamples__ directory are several examples. As an initial test, let's run the non-robust optimization script. First, move into the __GTSAM__ build directory.

```` 
cd RobustGNSS/gtsam/build 
````

Next, the RINEX file saved in the __RobustGNSS/gtsam/gnssData__ directory must be converted to a format readable by GTSAM. ( It should be noted that GTSAM only looks for data files in the __RobustGNSS/gtsam/gnssData__ directory, so all new data files must be stored there.  ) 

```` 
./gnssExamples/rnx2Gtsam --obs dec12.16o --sp3 dec12.sp3 > ../gnssData/dec12.gtsam 
````

Now, we can run the optimization script over the newly generated data file. 

```
./gnssExamples/l2Example -i dec12.gtsam --dir test1 --writeENU  
````

Finally, we can look at the ground trace of the solution, 

````
cd test1;
gnuplot 
plot 'enu.sol' using 2:3 with points
````
For a complete list of available options, run the command provided below.  

````
./gnssExamples/l2Example -h 
````


