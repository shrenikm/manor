# Final part of the refactor

Ok now for the final part of the deep refactor, completely getting rid of old code! To do this we need to create parallel functionality with the new system
  1. There is a simple_pick_and_place.py file in algorithms. Let's get rid of algorithms, and this simple pick and place now becomes a Metis policy! So make sure we do the same thing
  (I think it was hardcoded positions of joints to pick up hardcoded blocks in sim if I remember correctly) but just through a policy, sending eef or joint commands I'll let you figure
  it out
  2. In analysis we have the choreographer. So I still want this analysis script here but the choreographer can become another metis policy. We alerady have a yaml fo the choreographer
  so we can pull all of tha tinto the yaml fo the new choreographer policy. But I think we're making plots etc somewhere for the choreographer, I want all of that code still in
  analysis. Feelf ree to clean things up, align it ith our current codebase, etc
  3. In inspection/ we can remove the check gripper control, as we already have a policy for that. For visualizing the pliant diagram, we can remove that as well. Later on if we need
  to we can crate this functionality for Aegis. Butt I want to keep visualize manipulator. I don't remember how it works, but please preserve the functionality but using the new stuff
  if we need to
  4. We can get rid of the deprecated_lite6/ directory now I think. No need to preserve anything there, just make sure that all the usages are also updated, etc or even removed if not
  relevant/needed
  5. IN common/ I think we can get rid of everything in pliant/ as we now have Aegis. In definitions/ we have control_definitions.py that we can get rid of. Again make sure that all
  usages are updated/purged. Stuff in control/ is good and useful I think
  6. In general remove other old code, update their references, etc
