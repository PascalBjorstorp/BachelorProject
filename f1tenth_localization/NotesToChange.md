# Task


## To check
1. 
What to check:
Does AMCL “snap” after corners?
Does it lag in chicanes?
If yes → your motion model covariance is not honest enough

3. Change the way we measure speed
Instead of 
v = erpm
Have
Use IMU acceleration ONLY for short-term correction

v_imu = integrate(ax)
v = (1 - w) * v_erpm + w * v_imu

where w increases with slip



4. Already fused to ekf but check covariance


## Localization [0 / 2] 
- [ ] In AMCL node make sure that even with the new minimum distance needed to move it will still converge at start

- [ ] Make ekf pose also output a Velocity for a controller either calculate itself or directly transfer from odom so controller only needs to subscribe to one thing

## Matlab [3 / 3]
- [X] Change matlab plot for the speed bags to exclude the first 3 seconds atleast to not get the starting face included

- [X] Change matlab plot for the speed bags to not place start and stop positions

- [X] Change matlab plot to also include deviation from line

## Scan splitter [1 / 1]
- [X] Make scan splitter actually work better and more robust in terms of spotting obstacles

## Lateral planner [1 / 2]
- [X] Make lateral planner work with the new map so it doesn't create random routes

- [X] Make it work with dynamic obstacles and not just static ones

- [ ] Make it work with the same curvature speed regulation as PP

## Pipeline and benchmark testing [0 / 2]
- [ ] Make the pipeline monitor track CPU per core usage properly. Maybe even track pr process

- [ ] Ændrer så den ikke tjekker på drive men på commands/motor_speed/ (Eller hvad den nu hedder)

- [ ] Lav så pipeline monitor holder styr på at den scan der kommer ind matcher med den drive commando der bliver genereret ud fra den

## System ændringer [ 1 / 1]
- [X] Flyt lidar ned i system mappen

## Launch file [1 / 1]
- [SCRAPPED] Lav en launch fil der kører alt og at man  kan vælge imellem controllere
	Det gav ikke mening siden den ikke kan stoppe controlleren med det samme

## Vesc system [0 / 1]
 - [ ] Prøv at bliv bedre til at detektere drifting
		- Prøv med bedre model
		- Prøv at bruge EKF til IMU og model


## Generelt [0 / 3]
- [ ] Tilføj briefs over alt og fix de stedder den både er i cpp og hpp

- [ ] Kig på call back groups i forhold til bedre perfomance / mere sikkert ved multi threading

- [ ] Sæt publishers og subscribers ind i en yaml så de er ens overalt

- [ ] Kig på om jetson kan være permanent høj clock frekvens


## Rapport [0 / 2]

### Skrivning [0 / 1]
- [ ] Undersøg GMCL som et muligt emne i rapporten.

### Test [0 / 1]
- [ ] Lav en figur, hvor AMCL sammenlignes med Cartographer, Nav2 AMCL og MCL.

Sammenligningskriterier:
- Position offset
- Vinkel offset
- Evt. plot af spredning over tid
- Beregningstid


## Kilder
file:///sensors-25-02471-v3%20(1).pdf