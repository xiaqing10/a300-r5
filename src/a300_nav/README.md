# A300 Nav2

Reserved for the autonomous navigation phase.

Nav2 must not bypass the A300 safety layer. The final architecture is:

Nav2 /cmd_vel -> A300 safety layer -> base controller.