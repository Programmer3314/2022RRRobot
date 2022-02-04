// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Steps
 * 1) Start
 * 2) Hone its motors so its in the right position
 * 3) Idlee
 *  -----Button Hit-------------
 * 4) First Extend- extends lead hook until it touches the first bar, overshoot
 * 5) First Pullup - Lead hook pulls robot up, thus bringing the stationary hooks up
 * 6) Check - Check the stationary hook moved out and back on bar; check for secure grip on bar
 * 7) Second Extend- extends lead hook after a said amount of encoders, make sure the hook overshoots 
 * the second bar
 * 8) Second Pullup - Lead hook pulls the robot up, which will result it in swinging on the second bar
 * 9) Check - Check the stationary hook moved out and back on bar; check for secure grip on bar
 * 10) Third Extend Part One - extend the lead hook a little bit due to the robot swinging a lot, wait 
 * until robot swing calms down
 * 11) Third Extend Part Two - extend lead hook until it overshoots the third bar, which should be 
 * under the third bar
 * 12) Third Pullup - Lead hook pulls the robot until the stationary hook is off the second bar
 * 
 * 
 */



package frc.robot;

/** Add your docs here. */
public class ClimbStateMachine {}
