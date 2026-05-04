Next up: complete hardware integration.

We know that the stuff in lite6_cli is able to effectively operate the robot. Next is to make sure that the things in lite6 driver aligns with this.

Specifically:

- Please make sure that the stuff in the driver aligns with the stuff in the cli (that we know works). But please note that the things in the cli is very hacky atm so please clean up, consolidate logic, etc and then port into the driver to ensure that things are working. We use the cli as the base because I have validated that everything works as expected on actual hardware, although the code quality is a bit suspect.
- Make sure that we are able to prime and unprime the robot. So when the driver is constructed (as part of talos or kylos), it must prime. When the policy is stopped, it must unprime
- For the unpriming thinng let's also combine this with safety:
  - I want some safety integrated into the robot so that running stuff on the real robot is safe
  - I want it designed so that if metis is stopped or the frequency of commands out of metis is too low, the robot will automatically stop moving and unprime. For the lite6 this means switching the mode, unpriming (which means going back to zero position) and waiting there (doesn't nee to disconnect). This must happen either if Metis suddenly has a lapse in commands or it is outright killed while a policy is running. We don't want a rogue policy sending commands. Ideally this is based on the timestamp of the commands for the staleness of commands and for the thing where metis is killed, maybe we can still use the same staleness in timestmap logic as it's clean. Let's make a plan for how to tackle this.
  - Obvioulsy if aegis as a whole is also stopped, we must unprime. Again no reason to disconnect
