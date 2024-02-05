This was the fourth assignment I did for my Computer Animation class at Texas A&M
The goal was to create a chain of n-links and use different optimizers to perform inverse kinematics to resolve the angles.

I implemented a Gradient Descent, BFGS (Broyden–Fletcher–Goldfarb–Shanno), and Newton's Method optimizer.
For the actual calculation of the angles, I did not use Newton's Method becuase it required calculating the Hessian matrix.

'r' resets all angles
spacebar toggles IK - tracks links to mouse position
