#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph,
                         StartMeshcat, MeshcatVisualizer,
                         MultibodyPlant, Parser, Simulator)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import BasicVector, VectorSystem
import numpy as np

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
    parser.AddModels('mass_spring_damper/mass_spring_damper.sdf')


    # Positioning the model in the world frame.
    rotation = RollPitchYaw(0, 0, 0).ToRotationMatrix()  # No rotation.
    translation = [0, 0, 0.5]  # Translation along x, y, z axes.
    X = RigidTransform(rotation, translation) # Define the pose.

    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), X) # Weld the base frame to the world frame.

    # Finalize the plant.
    plant.Finalize()

    # Add the visualizer.
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Create a simulator.
    simulator = Simulator(builder.Build())

    # Set the initial conditions.
    context = simulator.get_mutable_context()

    # Set initial state
    q0 = np.array([1.,])
    v0 = np.zeros(plant.num_velocities())
    x0 = np.hstack((q0, v0))

    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetPositionsAndVelocities(plant_context, x0)

    # Simulate for 10 seconds.
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(10.0)

if __name__ == "__main__":
    main()