# Ece470: Frat Daddy
We are programming a robotic arm to play beer pong and run a simulation in CoppeliaSim. The problem would start similarly to trying to teach a robotic arm to shoot baskets, by using
reinforcement learning on what force and angle it needs to throw a ping pong ball to get it into a
standard solo cup at a programmed location. From there, the program can be expanded to teach the
robot to aim for any of the cups on the table until they are all eliminated.
## Project Update 1
Our first step is to get comfortable using our simulator.
We created a basic testing environment that we plan to use for the rest of the project and learned to control the movement of the UR3 arm.
We ran into trouble when trying to close the gripper once its proximity sensor was intersected by the ping pong ball, so we followed some tutorials to learn how to use a proximity sensor to stop a conveyor belt.
This still helped to get us more comfortable using CoppeliaSim but shows that we might need to use our own proximity sensor for the gripper.
It also is starting to become clear that coding in a different program and then interfacing with the simulator will probably be easier once our code becomes more complex.
<br /><br />Video link: https://youtu.be/3ew4tXnCdFk
## Project Update 2
We transitioned to using Python to remotely control the simulation, which seems like a much better choice for complex programs.
We're still having trouble reading the proximity sensor on the gripper and we think it's due to an issue with how it was built in the simulator, but we're chosing to work around it because we don't need the gripper's sensor for our idea.
<br /><br />We're able to feed the robot joint angles and have it move to lift the ping pong ball without hitting it off the table by carefully choosing the order in which to move each joint. To "grab" the ball, we temporarily made the ball a child object of the gripper so that they would move together.
Our next step is to work on giving the ball some initial velocity right after disconnecting it from the gripper, so that it would be thrown towards the cup. Then we can begin adjusting the magnitude of the velocity and the angle of the base of the robot to throw the ball to different locations.
<br /><br />To run the Python code, run the simulation (open the .ttt file in CoppeliaSim and press play) and then run fratDaddy_control.py from the terminal.
<br /><br />Video link: https://youtu.be/c4oVpIOJp3I
## Project Update 3
We were able to give the ping pong an initial velocity to throw it towards the cup and we manually changed the base angle and magnitude of the velocity so that it would land in the cup. We made some changes to the way we were simulating gripping the ball to make the system more repeatable and reliable. Adding a proximity sensor inside the cup allows us to print to the console that the ball has landed inside it, although the initialization of the sensor needs to happen after throwing the ball or else it brings it a lot of inconsistency to our simulation. We haven't figured out why that happens yet.
<br /><br />The code for controlling the arm was organized better into functions that will make the next step of implementing a neural network easier. The neural network will randomize the base angle and ball velocity to learn what combinations will land it into one of a full set of 10 cups.
<br /><br />Video link: https://youtu.be/nqU8xzZwBuU
## Final Project
We ended up using a brute force technique to find base angle and ball velocity pairs that would land the ball in each cup.
We put each input into a weighted list and increased the probability of the robot choosing that pair in a game every time that shot works for the intended cup.
<br /><br />Ultimately, we judged the success of our robot based on both how well it could play a full game of beer pong and how reliably each shot can make it into the desired cup outside of a game. We tested the reliability of each input and were able to find inputs that consistently landed in about 7 of the 10 cups. The few cups that we don’t have reliable inputs for often require that other cups are still in play so that the ping pong can bounce off of them. With our current dataset, the robot can usually eliminate about 7 or 8 cups within 50 shots, including the shots where the simulation has an error when lifting the ball. We consider this fairly successful, especially as we continue to train the robot more and understand that most of the missed shots are due to errors in our simulation and not errors in our data.
<br /><br />To run the Python code, open the simulation in CoppeliaSim, run fratDaddy_fullGame.py from the terminal, and follow its instructions.
<br /><br />Video link: https://youtu.be/_PW6flCHniM
