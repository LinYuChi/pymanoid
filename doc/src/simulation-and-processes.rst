************************
Simulation and processes
************************

Processes
=========

Simulations schedule calls to a number of "processes" into an infinite loop,
which represents the control loop of the robot. A process is a simple wrapper
around an :func:`on_tick` function, which is called at each iteration of the
simulation loop.

.. autoclass:: pymanoid.sim.Process
    :members:

Simulation
==========

The simulation object is both a global environment and a process scheduler. As
an environment, it is passed as argument when calling the :func:`on_tick`
functions of child processes, and also contains a number of fields, such as
``dt`` (simulation time step) or ``gravity`` (gravity vector in the world
frame).

.. autoclass:: pymanoid.sim.Simulation
    :members:

Camera recording
================

.. autoclass:: pymanoid.sim.CameraRecorder
    :members:
