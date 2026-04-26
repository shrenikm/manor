# Plan 5 for Manor

## Policies and controllers

Create policies/ and controllers/ directories in metis and kyber respectively. These will hold all of the policies and controllers.

Each policy and controller will be in a separate file.

## Policies

- Move the current policies -- identity and zero velocity policy into separate files inside policy
- Rename the Policy protocol to MetisPolicy. Also move this into policy_manager inside policies/
- Inside policy_manager create a MetisPolicyType enum that will be used to refer to each policy instead of passing the policy instances everywhere.
- In the manager create a MetisPolicyManager factory class that will have a classmethod for each defined policy. It will also have a generic from_policy_type() method which will be used downstream everywhere to create polciies from enums. Different policies may have different arguments, so I guess we can do a **kwargs input to this main function and then pass the relevant arguments to the specific policy classmethod. I hate using **kwargs but I don't see a better way to do this. If you have suggestions I'm open to them.
- The Metis config will now have the policy enum type instead of the policy isntance. Also we'll have another argument for the arguments to policy construction. Maybe this argument is a dict? depends on what we decide for the previous point.

## Controllers

Follow the same thing as policies. controllers/ directory, etc.

One question here though is that I think we need the Drake models to do some of the controllers? How do we wanna handle this? Do we pass in the multibodyplant etc? Does that even work?

## Config yamls

And now finally, the policy and controlelrs where the only non primitive arguments in the configs. With these replaced by enums, we can now construct the configs also from yamls.
What I want is one place to store all the aegis config yamls. I'll let you figure out the best place for this.

For now let's have one aegis_config.yaml which we should be able to parse in the AegisConfig similar to how we do env_configs.
But now this will have all the configs:
- mode
- manipulator variant
- env config 
- subsystem configs
- etc (everything needs to be yaml configurable)
And basically for the env config we would extract that part of the yaml and pass it into the env config parser.
Similar for the other configs, we extract the subsystem configs and pass it into the individual subsystem config parsers. don't parse all of them in aegis itself I want the parsing logic to be distributed so that it's an option to build individual subsystem configs with isolated subsystem yamls if we need to in the future.

