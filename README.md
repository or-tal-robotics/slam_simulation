# slam_simulation
By [Or Tslil](https://github.com/ortslil64)

This repository is a good place to start with for SLAM, NDT, etc.
Its main audiance is the Carmi reasearch group.

## Simulations
It contains multiple recording of a Gazebo simulations with LIDAR sensors.
The raw files are ros_bag files. There is a matlab code that convert that file into `.csv` files, but I have already done that for you!
You can use the preprocessed files in the `data_csv` folder. Use pandas library to import tose files, 
for example:
>import pandas as pd
>x = pd.read_csv('data_csv\file.csv')

You may use the `read_data.py` file in the `scrip` directory. It contains examples and a SLAM implemantation of mine that you can learn from.

