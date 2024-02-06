import numpy as np

from pydrake.all import *
# meshcat = StartMeshcat()

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

def run_demo():
    builder = DiagramBuilder()

    plant_config = MultibodyPlantConfig()
    plant_config.time_step = time_step
    # Deformable simulation only works with SAP solver.
    plant_config.discrete_contact_approximation = "sap"

    plant, scene_graph = AddMultibodyPlant(plant_config, builder)

    # Minimum required proximity properties for rigid bodies to interact with
    # deformable bodies.
    # 1. A valid Coulomb friction coefficient, and
    # 2. A resolution hint. (Rigid bodies need to be tesselated so that collision
    # queries can be performed against deformable geometries.)
    rigid_proximity_props = ProximityProperties()
    # Set the friction coefficient close to that of rubber against rubber.
    surface_friction = CoulombFriction(1.15, 1.15)
    rigid_proximity_props.AddProperty("hydroelastic",
                                      "resolution_hint", 0.1)
    AddContactMaterial(friction = surface_friction,
                       properties = rigid_proximity_props)
    
    # Set up the ground.
    ground = Box(4, 4, 4)
    X_WG = RigidTransform(RotationMatrix(), [0, 0, -2])
    plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                    "ground_collision", rigid_proximity_props)
    
    plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                                 "ground_visual", [0.7, 0.5, 0.4, 0.8])

    # TODO: Add gripper

    # Set up a deformable torus.
    owned_deformable_model = DeformableModel(plant)

    deformable_config = DeformableBodyConfig()
    deformable_config.set_youngs_modulus(E)
    deformable_config.set_poissons_ratio(nu)
    deformable_config.set_mass_density(density)
    deformable_config.set_stiffness_damping_coefficient(beta)

    # Load the geometry and scale it down to 65% (to showcase the scaling
    # capability and to make the torus suitable for grasping by the gripper).
    scale = 0.01
    # torus_mesh = Mesh("./torus.vtk", scale)
    torus_mesh = Mesh("/home/prini/Documents/pyDrake/pydrake_tests/deformable/mesh/roloc.vtk", scale)
    # Minor diameter of the torus inferred from the vtk file.
    kL = 0.09 * scale
    # Set the initial pose of the torus such that its bottom face is touching the
    # ground.
    X_WB = RigidTransform(RotationMatrix(), [0, 0, 0.3 + kL/2])
    torus_instance = GeometryInstance(X_WB, torus_mesh, "deformable_roloc")

    # Minimumly required proximity properties for deformable bodies:
    # A valid Coulomb friction coefficient.
    deformable_proximity_props = ProximityProperties()
    AddContactMaterial(friction = surface_friction,
                       properties = deformable_proximity_props)
    torus_instance.set_proximity_properties(deformable_proximity_props)

    # Registration of all deformable geometries ostensibly requires a resolution
    # hint parameter that dictates how the shape is tesselated. In the case of a
    # `Mesh` shape, the resolution hint is unused because the shape is already
    # tessellated.
    # Though unused, we still asserts the resolution hint is positive.
    unused_resolution_hint = 1.0
    owned_deformable_model.RegisterDeformableBody(
        torus_instance, deformable_config, unused_resolution_hint)

    plant.AddPhysicalModel(owned_deformable_model)

    deformable_model = owned_deformable_model

    # All rigid and deformable models have been added. Finalize the plant.
    plant.Finalize()

    # It's essential to connect the vertex position port in DeformableModel to
    # the source configuration port in SceneGraph when deformable bodies are
    # present in the plant.
    builder.Connect(
        deformable_model.vertex_positions_port(),
        scene_graph.get_source_configuration_port(plant.get_source_id()));
    
    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    DrakeVisualizer().AddToBuilder(builder, scene_graph)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate)
    # meshcat.AddButton("Stop Simulation", "Escape")

    simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)

    # while meshcat.GetButtonClicks("Stop Simulation") < 1:
    #     simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
        
    # meshcat.DeleteButton("Stop Simulation")
    # meshcat.DeleteAddedControls()

run_demo()