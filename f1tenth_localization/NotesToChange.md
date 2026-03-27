# Localization
In AMCL node make sure that even with the new minimum distance needed to move it will still converge at start

Make ekf pose also output a position for a controller either calculate itself or directly transfer from odom so controller only needs to subscribe to one thing

# Matlab
Change matlab plot for the speed bags to exclude the first 3 seconds atleast to not get the starting face included

Change matlab plot for the speed bags to not place start and stop positions

# Scan splitter
Make scan splitter actually work better and more robust in terms of spotting obstacles

# Lateral planner
Make lateral planner work with the new map so it doesn't create random routes

# Pipeline and benchmark testing.
Make the pipeline monitor track CPU per core usage properly. Maybe even track pr process

Ændrer så den ikke tjekker på drive men på commands/motor_speed/ (Eller hvad den nu hedder)

# System ændringer 
Flyt lidar ned i system mappen