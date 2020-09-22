Hi there,
    If you're reading this, then you have been given the unfortunate position as programmer on the FTC team 8400 The Perfect Paradox,
    (or whatever they're calling it these days). I've tried to keep it organized, but its an absolute mess and you have to deal with it.
    Here's how this whole thing works:

    I. This project is split between auto and teleop. In the teleop folder, you'll find a "robot" class and a actual teleop class. The robot class
    is used to set up all of the different functions of the robot for teleop and initialize them, while the teleop opmode is just for running the,
    in game. This should cause you much trouble as its pretty well lined up and easy to figure out.

    II. Auto is a lot more complex. This FTC App has been set up to run RoadRunner, a libray used to generate and follow trajectorys controlled
    by motion profiles. this is very complex and takes a lot of work to do. More infor can be found about that at the links below.
    The robot is set up similarly in the robot file located in the drive package in roadrunner. However, theres a few more fun things to
    figure out there.

        Made with love,

        Marc Bulloch



        xoxoxo