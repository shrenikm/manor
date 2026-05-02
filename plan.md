Ok so next task, I don't really like how the current aegis config yamls are set up.

We have individual yamls for individual policies but this defeats the purpose.

My plan:

1. Let's have a single base yaml for each manipulator type. So currently just lite6_ac.yaml
2. This will have all of the params except for the policy and controller configs
3. Under config/aegis/ let's have two directories to store the individual policy and controller configs. So config/aegis/policies/ and config/aegis/controllers/
4. Create a config yaml for each existing policy and controller
5. In the base yaml, under metis config, we will only have the publish hz and a "policy_type" that will be a string that coresponds to the type of policy. Same for controller inside kyber_config
6. Adn then in the individual policy and controller yamls we will store the policy and controller specific configs. So for example, the policy_config under metis_config will no longer exist and will instead be moved into policies/
7. Note that there will be some changes, eg: when moved into the separate yaml, the policy/controller configs will not have "type" as we will know the tyupe from the name of the yaml itself. Note that the name must be <policy/controller_type_name>_ac.yaml. And then the metis/kyber will have policy/controller_type: <policy/controller_type_name>
8. Make sure that all the parsing accounts for this. DO NOT allow aegis to be run if the controller/policy does not have a yaml defined. There must be no default values outside, everything must happen through the yaml. Obivously while parsing, we need to account for the _ac suffix in the yaml filename etc.
9. Make sure we write tests for the updated parsing. Also write a test that lists out every single policy and controller implementation and check that yamls exist for them matching the policy/controller type name (in the enum). Also add any other tests you can think of.
