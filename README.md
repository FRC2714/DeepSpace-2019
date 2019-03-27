# 2019Competition [![Build Status](https://travis-ci.org/FRC2714/2019Competition.svg?branch=spline-generation)](https://travis-ci.org/FRC2714/2019Competition)

<img src="https://github.com/FRC2714/2019Competition/blob/master/pictures/logo%20for%20github.svg" width="200" height="200" />


Each subsystem contains unique commands defined internally within 
the subsystem class itself. This allows for better organization of commands 
relative to their mechanisms. 

## Subsystems

#### DriveTrain
##### Commands
- driver_control
- brake_mode
- closed_loop_tank
- set_angular_offset
- debug_print
- add_forwards_spline
- add_forwards_spline_dynamic
- add_backwards_spline
- add_backwards_spline_dynamic
- start_path
- delay_tester
- drive_to_target
- wait
- vision_align
- auton_vision_align
- set_current_position
- cancel_all

#### Arm
##### Commands
- jog_up
- jog_down
- arm_to_position
- start_position
- floor_cargo_position
- floor_hatch_position
- station_position
- auton_lower_hatch
- go_to_position
- lower_score
- cargo_station_score
- middle_score
- upper_score
- back_score
- extake

#### Intake
##### Commands
- cargo_true
- hatch_true
- intake_stop
- cargo_intake
- hatch_floor_intake
- hatch_station_intake
- set_cargo_mode
- set_hatch_mode
- hatch_intake
- climber_up

#### Climber
##### Commands (Out of Date)
- lifter_down
- lifter_up
- pusher_out
- pusher_in
- get_climber_positions
- send_climb
- halt_climb

## Button Configuration

<img src="https://github.com/FRC2714/2019Competition/blob/master/pictures/ButtonBoxLayout_3-13-19.png" width="750" height="750" />
