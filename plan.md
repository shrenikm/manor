# Next Plan for Manor

I want to do a full redesign of how the current systems are set up for manipulation.
The end goal is still the same -- to be able to run the robot both in simulation and on the hardware through the same code.

The initial idea was to have a "pliant" (Drake plant but pliant as it's flexible for hardware and sim) but it's currently mainly a monolithic system.
We currently also don't take any sensor inputs, etc.

I want the system to be split into multiple modules that both publish and subscribe to messages. Why?

1. I want to use this project for both classical motion/control and also more modern learning based policies (VLAs, etc)
2. To run ML policies, I need to run it on a remote server (with GPU) so this needs to be distributed system
3. I currently don't have any inputs (images, depth, etc) so I need to integrate this into the entire system as well
