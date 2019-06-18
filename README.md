# slam_simulation
By [Or Tslil](https://github.com/ortslil64)

This repository is a good place to start with for SLAM, NDT, etc.
Its main audience is the Carmi research group.

## Simulations
It contains multiple recording of a Gazebo simulations with LIDAR sensors.
The raw files are ros_bag files. There is a MATLAB code that convert that file into `.csv` files, but I have already done that for you!
You can use the pre-processed files in the `data_csv` folder. Use pandas library to import those files, 
for example:

>import pandas as pd

>x = pd.read_csv('data_csv\file.csv')

You may use the `read_data.py` file in the `scrip` directory. It contains examples and a SLAM implementation of mine that you can learn from.
 ## Modifying the code
 You can (and should) change or modify my code in order to learn and try staff on your own.
 If you do that, please create a new branch or fork this repository.
 
 Good luck,
 Or.
