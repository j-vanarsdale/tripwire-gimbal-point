## API Script
Currently the script auto turns on ads-b targetting through args and can't be changed.

Things to do:
- Change from closest aircraft track to manual targetting track
- Add way to stop and start ads-b tracking
- Possibly: switch between kalman and dead reckoning? and manual? (frontend)
- Even more possibly: add adjustable update rate (frontend)
- Even more possibly: add adjustable self position smoothing (frontend)
## ADS-B Script
Has an untested kalman filter class and a sort of tested dead reckoning class. Currently the dead reckoning is used inside the gimbal point.

Constants
- MAX_EXTRAPOLATION_S = 20.0
- LEAD_TIME_S = 1.0

Used for future look to when position is stale. 

LEAD_TIME_S is a time interval in seconds added to look ahead of the aircraft. 

MAX_EXTRAPOLATION_S is max time that it calculates position for if there is no updates from ads-b.


## Old files
**GIMBAL_POINT.py**  <-- Default script for pure lat/lon/alt targetting

**GPC.py**           <-- Testing script with fixed location mode and manual targetting mode

**GPC_Curses.py**    <-- Old vibecoded curses version -- ignore

**GPC_Demo.py**      <-- Rotates through NSEW look every 5 seconds
