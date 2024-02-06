import numpy as np
from pydrake.all import *

simulation_time = 2.0
# Desired duration of the simulation [s].
simulation_time = 2.0
# Desired real time rate.
realtime_rate = 1.0
# Discrete time step for the system [s]. Must be positive.
time_step = 1e-3
# Young's modulus of the deformable body [Pa].
E = 3e6
# Poisson's ratio of the deformable body, unitless.
nu = 0.4
# Mass density of the deformable body [kg/m³].
density = 920
# Stiffness damping coefficient for the deformable body [1/s].
beta = 0.01



def main():
    builder = DiagramBuilder()

    plant_config = MultibodyPlantConfig()
    plant_config.time_step = time_step
    # Deformable simulation only works with SAP solver.
    plant_config.discrete_contact_approximation = "sap"

    plant, scene_graph = AddMultibodyPlant(plant_config, builder)

    rigid_proximity_props = ProximityProperties()
    surface_friction = CoulombFriction(1.15, 1.15)
    rigid_proximity_props.AddProperty("hydroelastic",
                                      "resolution_hint", 0.1)
    AddContactMaterial(friction = surface_friction,
                       properties = rigid_proximity_props)

    workpiece_dimensions = [1.0, 1.0, 0.4]
    ground = Box(workpiece_dimensions[0], workpiece_dimensions[1], workpiece_dimensions[2])

    X_WG = RigidTransform(RotationMatrix(), [0, 0, -workpiece_dimensions[2]/2])
    plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                    "ground_collision", rigid_proximity_props)
    
    plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                                 "ground_visual", [0.7, 0.5, 0.4, 0.8])



    # Parse the URDF file.
    parser = Parser(plant)
    # Create a PackageMap and add the package.
    package_map = PackageMap()
    
    cwd = os.path.dirname(os.path.abspath(__file__)) 
    package_map.PopulateFromFolder(cwd)
    
    # Add the package map to the parser.
    parser.package_map().AddMap(package_map)

    # parser.AddModels('urdf/roloc.urdf')
    parser.AddModels(os.path.join(cwd, 'urdf', 'cartesian.urdf'))

    X_WB = RigidTransform(
        RotationMatrix(RollPitchYaw(0, 0, 0)),  # No rotation.
        [0, 0, 0.005]  # Translation of 1 meter in the z direction.
    )

    # Add the model to the plant.
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), X_WB) # Weld the base frame to the world frame.



    owned_deformable_model = DeformableModel(plant)

    deformable_config = DeformableBodyConfig()
    deformable_config.set_youngs_modulus(E)
    deformable_config.set_poissons_ratio(nu)
    deformable_config.set_mass_density(density)
    deformable_config.set_stiffness_damping_coefficient(beta)

    scale = 0.01
    roloc_mesh = Mesh("/home/prini/Documents/pyDrake/pydrake_tests/deformable/mesh/roloc.vtk", scale)
    kL = 0.09 * scale
    X_WB = RigidTransform(RotationMatrix(), [0, 0, 0.5 + kL/2])
    roloc_instance = GeometryInstance(X_WB, roloc_mesh, "deformable_roloc")

    deformable_proximity_props = ProximityProperties()
    AddContactMaterial(friction = surface_friction,
                       properties = deformable_proximity_props)
    roloc_instance.set_proximity_properties(deformable_proximity_props)

    unused_resolution_hint = 1.0
    roloc_id = owned_deformable_model.RegisterDeformableBody(
        roloc_instance, deformable_config, unused_resolution_hint)

    plant.AddPhysicalModel(owned_deformable_model)

    deformable_model = owned_deformable_model

    X_RD = RigidTransform(
        RotationMatrix(RollPitchYaw(0, 0, 0)),  # No rotation.
        [0, 0, 0.0]  # Translation of 1 meter in the z direction.
    )


    # get the spindle frame
    spindle_frame = plant.GetFrameByName("spindle")
    spindle_body = plant.GetBodyByName("spindle")


    cylinder = Cylinder(0.1, 0.2)

    deformable_model.AddFixedConstraint(roloc_id, spindle_body, X_RD,
                                                cylinder, X_RD)


    plant.Finalize()


    builder.Connect(
        deformable_model.vertex_positions_port(),
        scene_graph.get_source_configuration_port(plant.get_source_id()))
    
    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    dv = DrakeVisualizer().AddToBuilder(builder, scene_graph)

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate)
    # meshcat.AddButton("Stop Simulation", "Escape")

    simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)



if __name__ == "__main__":
    main()