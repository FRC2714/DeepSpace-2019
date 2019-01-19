<b>FRC 2714 Public Competition Code</b>

Current Test Coverage of Master Branch: 70%
Feel free to submit issues/feature requests to this repository!

Configuration:
3-Spark Max DriveTrain
Motion Profiling on Spark Max

How to tune robot:
1. Get robot to drive
2. Open the SmartDashboard
3. Set everything to 0
4. Set a velocity
5. Set a P value for both sides
6. Change the P value until the velocity oscillates
7. Multiply P value by 0.6
8. Change the I value until the actual velocity is accurate
9. Plot the actual velocity for multiple velocities on each side
10. Set the FF to the slope
11. Repeat steps 5-8
