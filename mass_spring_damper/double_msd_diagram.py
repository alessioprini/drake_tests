#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pydrake.all import (
    DiagramBuilder, 
    AddMultibodyPlantSceneGraph,
    StartMeshcat, 
    MeshcatVisualizer,
    MultibodyPlant, 
    Parser, 
    Simulator,
    LogVectorOutput, 
    Diagram)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import BasicVector, VectorSystem
import numpy as np


# Create a simple block diagram containing our system.
class MSD(Diagram):
    def __init__(self, sdf_file = 'mass_spring_damper/double_mass_spring_damper.sdf',
                  fixed_frame = 'base',
                  weld_transform = RigidTransform(RollPitchYaw(0, 0, 0).ToRotationMatrix(), [0,0,0]),
                  visualize = True):
        Diagram.__init__(self)
        self._builder = DiagramBuilder()

        meshcat = None
        if visualize == True:
            # create a meshcat visualizer
            meshcat = StartMeshcat()


        self._plant, self._scene_graph = AddMultibodyPlantSceneGraph(
                                            self._builder, time_step=0.0001)
        

        # Parse the SDF file.
        parser = Parser(self._plant)
        parser.AddModels(sdf_file)

        # Positioning the model in the world frame.
        self._plant.WeldFrames(self._plant.world_frame(),
                                self._plant.GetFrameByName(fixed_frame), weld_transform) # Weld the base frame to the world frame.

        self._plant.Finalize()
        if meshcat != None:
            # Add the visualizer.
            MeshcatVisualizer.AddToBuilder(self._builder, 
                                       self._scene_graph, meshcat)

        self._actuation_input_port = self._builder.ExportInput(self._plant.get_actuation_input_port(),
                                  "actuation_input_port")

        self._state_output_port = self._builder.ExportOutput(self._plant.get_state_output_port(),
                                   "state_output_port")
               
        self._builder.BuildInto(self)


    def set_initial_conditions(self, plant_context ,q0 = [0.,0.], v0 = [0.,0.]):
        x0 = np.hstack((q0, v0))
        self._plant.SetPositionsAndVelocities(plant_context, x0)


    def get_state_output_port(self):
        return self.get_output_port(self._state_output_port)
    
    def get_actuation_input_port(self):
        return self.get_input_port(self._actuation_input_port)

def main():
    msd = MSD()
    simulator = Simulator(msd)
    context = simulator.get_mutable_context()

    # Simulate for 10 seconds.
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(10.0)

if __name__ == "__main__":
    main()

