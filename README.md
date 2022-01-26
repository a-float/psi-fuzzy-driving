# psi-fuzzy-driving

Simulation of cars driving in a 2D space, avoiding obstacles using a fuzzy controller.  
Fuzzy logic implemented via SciKit-Fuzzy package.

Cars try to stay in the middle of the road, slow down while turning and try to reverse when they face a wall.

Driver inputs:
  - current vehicle speed
  - current vehicle angular velocity
  - distance to the nearest obstacle to the left of the vehicle
  - distance to the nearest obstacle in front of the vehicle
  - distance to the nearest obstacle to the right of the vehicle
  
Driver outputs:
  - vehicle acceleration
  - vehicle angular acceleration
