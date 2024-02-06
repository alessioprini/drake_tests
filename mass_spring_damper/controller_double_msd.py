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
    Diagram,
    Adder, 
    Multiplexer,
    Demultiplexer,
    ConstantVectorSource)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import BasicVector, VectorSystem
import numpy as np
from double_msd_diagram import MSD
from pydrake.systems.controllers import PidController

def controlled_system(kp, ki, kd):
    builder = DiagramBuilder()
    msd = MSD()
    builder.AddSystem(msd)
    # I want to control just one dof - the mass1_mass2_joint in particulars
    pid = builder.AddSystem(PidController(kp=[kp], ki=[ki], kd=[kd]))
    torque = [0.0]  # Replace with your desired torque.
    torque_source = builder.AddSystem(ConstantVectorSource(torque))

    desired_state = [0.0, 0.0]  # Replace with your desired state.
    desired_state_source = builder.AddSystem(ConstantVectorSource(desired_state))

    torque_multiplexer = builder.AddSystem(Multiplexer(2))
    demux_state = builder.AddSystem(Demultiplexer(4))
    state_multiplexer = builder.AddSystem(Multiplexer(2))

    # connect a target torque = 0 to the multiplexer at index 0
    builder.Connect(torque_source.get_output_port(0), torque_multiplexer.get_input_port(1))
    # connect the PID controller to the multiplexer at index 1
    builder.Connect(pid.get_output_port_control(), torque_multiplexer.get_input_port(0))


    # connect the demultiplexer to the MSD ti split the state into two
    builder.Connect(msd.get_state_output_port(), demux_state.get_input_port())
    builder.Connect(demux_state.get_output_port(0), state_multiplexer.get_input_port(0))
    builder.Connect(demux_state.get_output_port(2), state_multiplexer.get_input_port(1))

    # connect the state multiplexer to the PID controller
    builder.Connect(state_multiplexer.get_output_port(0), pid.get_input_port_estimated_state())

    # connect the desired state to the PID controller
    builder.Connect(desired_state_source.get_output_port(), pid.get_input_port_desired_state())

    # Connect the PID controller to the plant.
    builder.Connect(torque_multiplexer.get_output_port(), msd.get_actuation_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = msd._plant.GetMyMutableContextFromRoot(context)

    msd.set_initial_conditions(plant_context, [1., -1.])


    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(10.0)

def main():
    controlled_system(10.0, 0.0, 0.0)
    controlled_system(100.0, 10.0, 10.0)

if __name__ == "__main__":
    main()
 