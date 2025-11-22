# Approximation
- Assume the rocket is a cylinder going up.
- Assume there is a constant force F acting on the low-side
- Assume torque control, so the motor can gimbal in any direction to make the main force work in the opposite direction.
- Assume force from graivty, pulling the rocket down (assume certain weight of the cylinder)
- Assume force from air resistance, pulling the rocket down (f(height))

## Model
- Get the transfer function between the gimbal orientation and the force (assuming a constant force F / Momentum M coming out of the rocket engine)

### Non-rotation units
- $Impulse\ J = \Delta\ p =  F*dt = M*v_2 - M*v_1 = \frac{kg*m}{s}$
- $Force\ F = m*a = \frac{kg*m}{s^2}$

### Rotational units
- $Angular\ Impulse  =  dM*dt = I*\omega_2 - I*\omega_1 = \frac{kg*m^2}{s}$
- $Angulear\ Momentum = I*\omega = \frac{kg*m^2}{s}$	