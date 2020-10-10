# Laptime simulation software
Started as a pilot project for the formula student team. The software has most of the functionalities required for the initial concept stage. Including features like sensitivity analysis, point-based results, suspension characteristics study etc. 

Most of the simulation process is also fully automated, and the user has the option to choose which test cases to run. There is also a post-processing window to process the final results.

<img src ="images/Capture.JPG" 

## Functionalities
1. It has Two-track Model
1. Includes the effects of center of pressure.
1. Includes option to change between 2wd and 4wd
1. Simulates all the events (Acceleration, skidpad , Autocross, and endurance)
1. Points calculator.

# Setting up the environment
1. Install Matlab 2019b+.
1. Add the all the content to the current working directory.
1. Install the "App designer" package if needed.

# Set up the code
1. open the "Lap_sim.mlapp".
1. Setup the vehicle model parameters as needed.
1. Setup the simulation parameter, test cases etc.
1. Run the simulation

# Bugs/Issues
1. There are some problems with the integration solver. This causes unreasonable values in the output results.
1. The Solver has issues at very tight corners.
1. Assumed to have rigid suspension.
1. The SW is still not stable. Might get errors some time.
1. Validation is still pending.
1. Option to build map is still pending
