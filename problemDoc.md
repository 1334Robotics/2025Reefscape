# Swerve wheels turning in the wrong direction
## 2025-02-01
## Written by: Evan Sadler

This problem started when we noticed that turning with the robot looked extremely wierd. \
It was drifting around instead of spinning in a clean circle.

We found that the front right wheel was spinning counter-clockwise when everything else was spinning clockwise. \
We fixed that by readjusting the encoder offsets in the deploy/swerve/modules folder.

That fix made the terrible problem of a single wheel spinning in the wrong direction to a slightly more managable one of the pairs of opposite wheels spinning opposite to each other, but only while turning (front right and back left spun in the same direction, and front left and back right spun in the same direction).

We redid the config for the swerve drives, and that made everything align. \
The problem then was that right was forwards, but we just edited the DriveCommand to fix that offset.