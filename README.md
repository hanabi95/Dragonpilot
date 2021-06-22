<b/>**Release Notes**</b>

This branch is only for GM Chevolet Bolt EV based on latest released version from dragonpilot

# Features

  - New panda code supports comma/custom made harness for black panda
  - Update pedal code in panda and DBC for Comma Pedal
  - Add Battery Charging Logic (thanks to 양민님)
  - <b>Support Comma pedal for longitudinal control but this does not guarantee fully to provide the Stop & Go </b>
    1) Only Lateral control by OP
       - Engage : main switch on
       - Disengage : brake or main switch off
    2) Only Longitudinal control by OP (comma pedal shall be installed)
       - Engage : accel(resume) button but main switch must be kept off
       - Speed control : accel or decel button (+/- 5km/h)
       - Disengage : Driver braking, cancel button or or main switch on(only lateral control enable)
    3) Both Lateral and Longitudinal control by OP (comma pedal shall be installed)
       - Engage : set(decel) button but main switch must be kept off
       - Speed control : accel or decel button (+/- 5km/h)
       - Disengage : Driver braking, cancel button or main switch on(only lateral control enable)       
