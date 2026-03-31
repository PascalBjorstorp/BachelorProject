# Task

## Localization [0 / 2] 
- [ ] In AMCL node make sure that even with the new minimum distance needed to move it will still converge at start

- [ ] Make ekf pose also output a position for a controller either calculate itself or directly transfer from odom so controller only needs to subscribe to one thing

## Matlab [3 / 3]
- [X] Change matlab plot for the speed bags to exclude the first 3 seconds atleast to not get the starting face included

- [X] Change matlab plot for the speed bags to not place start and stop positions

- [X] Change matlab plot to also include deviation from line

## Scan splitter [0 / 1]
- [ ] Make scan splitter actually work better and more robust in terms of spotting obstacles

## Lateral planner [0 / 1]
- [ ] Make lateral planner work with the new map so it doesn't create random routes

## Pipeline and benchmark testing [0 / 2]
- [ ] Make the pipeline monitor track CPU per core usage properly. Maybe even track pr process

- [ ] Ændrer så den ikke tjekker på drive men på commands/motor_speed/ (Eller hvad den nu hedder)

## System ændringer [ 1 / 1]
- [X] Flyt lidar ned i system mappen

## Launch file [0 / 1]
- [ ] Lav en launch fil der kører alt og at man  kan vælge imellem controllere

## Generelt [0 / 3]
- [ ] Tilføj briefs over alt og fix de stedder den både er i cpp og hpp

- [ ] Kig på call back groups i forhold til bedre perfomance / mere sikkert ved multi threading

- [ ] Sæt publishers og subscribers ind i en yaml så de er ens overalt