# Challenge Drop #4: Rocket Ascent Guidance Control System

Special industry challenge presented by Momentum Aerospace, a new rocket startup in Berlin (founder on the jury)

This is your chance to build something that could end up in a real rocket.
Your mission is to design and implement a real-time ascent guidance control module that converts high-level commands into actuator outputs.

# The Task

Develop control software in Python or C that takes a user-defined input vector, either an attitude command or an acceleration command, and generates actuator commands for a rocket during ascent.

Choose one control architecture:
- Fin tabs
- Direct TVC
- TVC using jet vanes
- Or any other control method you believe is viable

## Your implementation should:
- Generate stable actuator commands
- Demonstrate your control logic such as PID, LQR or nonlinear control
- Show how it integrates into an ascent guidance loop

### Hardware In The Loop Preparation

Include basic hardware code suitable for later HITL tests with a custom PCB.
You do not need to design the PCB, but a rough sketch is welcome if time allows.

# Scope And Timebox

## Estimated effort: up to 8 hours
Focus on core functionality, clarity and engineering reasoning.

### Deliverables

- Python or C control loop
- Actuator command generation logic
- Architecture explanation or diagram
- Optional PCB interface sketch
- Optional simple simulation showing system response

### Why This Is Real

This challenge mirrors an actual engineering task at Momentum Aerospace.
Their founder will sit on the jury, so a strong solution may influence real launcher development.