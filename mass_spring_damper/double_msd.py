#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# https://github.com/RobotLocomotion/drake/tree/master/tutorials

import matplotlib.pyplot as plt
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph,
                         StartMeshcat, MeshcatVisualizer,
                         MultibodyPlant, Parser, Simulator,
                         LogVectorOutput)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import BasicVector, VectorSystem
import numpy as np
from pandas import DataFrame

def main():
    # Create a simple block diagram containing our system.
    builder = DiagramBuilder()
    # create a meshcat visualizer
    meshcat = StartMeshcat()

    # Create a MultibodyPlant containing a model of the world.
    plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=0.0001)

    # Parse the SDF file.
    parser = Parser(plant)
    parser.AddModels('mass_spring_damper/double_mass_spring_damper.sdf')

    # Positioning the model in the world frame.
    rotation = RollPitchYaw(0, 0, 0).ToRotationMatrix()  # No rotation.
    translation = [0, 0, 0.5]  # Translation along x, y, z axes.
    X = RigidTransform(rotation, translation) # Define the pose.

    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), X) # Weld the base frame to the world frame.

    # Finalize the plant.
    plant.Finalize()

    # Add the visualizer.
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # set up the logger 
    logger = LogVectorOutput(plant.get_state_output_port(), builder)

    # Create a simulator.
    diagram = builder.Build()
    simulator = Simulator(diagram)

    # Set the initial conditions.
    context = simulator.get_mutable_context()

    # Set initial state
    q0 = np.array([1., -1.])
    v0 = np.zeros(plant.num_velocities())
    x0 = np.hstack((q0, v0))

    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetPositionsAndVelocities(plant_context, x0)

    # Simulate for 10 seconds.
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(10.0)


    log = logger.FindLog(simulator.get_context())
    data = []
    data = DataFrame(
            {
                "t": log.sample_times(),
                "x1": log.data()[0, :],
                "x2": log.data()[1, :],
                "xdot1": log.data()[2, :],
                "xdot2": log.data()[3, :],
            }
        )
    
    # plot the data
    fig, ax = plt.subplots(2, 2)
    ax[0, 0].set_xlabel("t")
    ax[0, 0].set_ylabel("x1")
    ax[1, 0].set_xlabel("t")
    ax[1, 0].set_ylabel("x2")
    ax[0, 1].set_xlabel("t")
    ax[0, 1].set_ylabel("xdot1")
    ax[1, 1].set_xlabel("t")
    ax[1, 1].set_ylabel("xdot2")

    ax[0, 0].plot(data.t.to_numpy(), data.x1.T.to_numpy())
    ax[1, 0].plot(data.t.to_numpy(), data.x2.T.to_numpy())
    ax[0, 1].plot(data.t.to_numpy(), data.xdot1.T.to_numpy())
    ax[1, 1].plot(data.t.to_numpy(), data.xdot2.T.to_numpy())


    plt.show()

if __name__ == "__main__":
    main()