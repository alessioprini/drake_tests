#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# https://github.com/RobotLocomotion/drake/tree/master/tutorials

import matplotlib.pyplot as plt
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph,
                         StartMeshcat, MeshcatVisualizer,
                         MultibodyPlant, Parser, Simulator,
                         LogVectorOutput,
                         PackageMap,
                         RotationMatrix)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import BasicVector, VectorSystem
import numpy as np
from pandas import DataFrame
import os

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

    # Create a PackageMap and add the package.
    package_map = PackageMap()
    
    cwd = os.path.dirname(os.path.abspath(__file__)) 
    package_map.PopulateFromFolder(cwd)
    
    # Add the package map to the parser.
    parser.package_map().AddMap(package_map)

    # parser.AddModels('urdf/roloc.urdf')
    parser.AddModels(os.path.join(cwd, 'urdf', 'roloc.urdf'))

    X_WB = RigidTransform(
        RotationMatrix(RollPitchYaw(0, 0, 0)),  # No rotation.
        [0, 0, 0.005]  # Translation of 1 meter in the z direction.
    )

    # Add the model to the plant.
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), X_WB) # Weld the base frame to the world frame.
    parser.AddModels(os.path.join(cwd, 'urdf', 'workpiece.urdf'))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("box")) # Weld the box frame to the world frame.

        # Finalize the plant.
    plant.Finalize()

    # Add the visualizer.
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # Set the initial conditions.
    context = simulator.get_mutable_context()
    # Simulate for 10 seconds.
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(5.0)

if __name__ == "__main__":
    main()