# RobustGNSS

This repository contains a modified version of [GTSAM](https://bitbucket.org/gtborg/gtsam), which has been updated for GNSS signal processing. To enable RINEX file reading and GNSS observation modeling, the [GPSTk](http://www.gpstk.org/bin/view/Documentation/WebHome) library is utilized. A detailed description of the modification can be found in ["Robust Navigation In GNSS Degraded Environment Using Graph Optimization"](https://www.researchgate.net/profile/Ryan_Watson7/publication/320084321_Robust_Navigation_in_GNSS_Degraded_Environment_Using_Graph_Optimization/links/59cd10ef0f7e9b6e147906ec/Robust-Navigation-in-GNSS-Degraded-Environment-Using-Graph-Optimization.pdf). This software has been cleared for public release by the USAF Case # 88ABW-2017-3893

For information on incorporating carrier-phase observations, please see ["Evaluation of Kinematic Precise Point Positioning Convergence with an Incremental Graph Optimizer"](https://www.researchgate.net/publication/324454778_Evaluation_of_Kinematic_Precise_Point_Positioning_Convergence_with_an_Incremental_Graph_Optimizer). The software released with this paper can be found here: [https://github.com/wvu-navLab/PPP-BayesTree](https://github.com/wvu-navLab/PPP-BayesTree).


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
![fg error](https://lh3.googleusercontent.com/DnI5HJqO9Y6wzI4MMIp6Vx8gPyoSZphMc-5f1hD8U2kEZIS2jl1NoaZpWtxSwGO86PVa8E91nth6KDbZKpJN0Yc5yUMN_4JGSItvJRW3HAhO0RHGdseCkq5QaQ59PBE8LHWl-cjUz9RJse5T9upbfdin_yYyn_nCkBMJJj3r_2kTo_oTI1QsqSC8QZnfLACIwOO_vUBksVvYq3Bg6XhxkfIVVcXYB5f8gJFtRdO_405Bv-cge-BmoUwsq0pR-yx0JGAEnrR8aqVKEj9OFS5QwbLUTmat4R9G-DCzjBaCZQ_OJ3kGcJkprS0bCqRAc0b_iZpq7qgo-MFvS-qUfYcvN8c1QKLDlqvqPZXtxXkfRqbnEbjpTFfJX5B3ZH0O-FcFlV8YP0MboosaYrVom9P4NCLUpk-vR5BHCyZr9Oz0Bh93NhYuPw28zZQJh1iGmqYTBOX5XdUFUtnt1KGDqCGUYRlcSzLONmc8pa-9aA5L0Cvr10-IgipRrcGav890-a-HdrqU6ib3ua078lf0cWb55D50fgtad0RTXF89ypRPkhrRhwBbIrZzlbERWw2G6agyFItqJbAXPSd_nhaopAVZN0tYWtoaNYCWyl100H_5wSFmy_mdHUbiebpj19ab-IOMa66zfCyFpFkLdHN3oJgZYGkdRYopfxs6=w1280-h633-no)


For a complete list of available options, run the command provided below.  

````
./gnssExamples/l2Example -h 
````
