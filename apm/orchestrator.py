'''
The brain that carries out the main logic of the Autonomous Pacemaker.

Main states:
- Idle: waiting for user input to start or configure parameters
- Config: allowing user to set parameters (e.g. pace profile, distance tolerance)
- Starting: triggered by user, initializing hardware and software components
- Running: main loop, following pace profile and maintaining speed and distance
- Stopping: triggered by user or error, stopping all components
- Error: if stopped due to error, waiting for user to reset

'''