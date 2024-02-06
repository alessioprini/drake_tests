#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/deformable_model.h>
#include <drake/multibody/plant/multibody_plant_config.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include "drake/multibody/plant/coulomb_friction.h"
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/geometry_properties.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/math/rotation_matrix.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/parsing/package_map.h>
#include <drake/multibody/plant/physical_model.h>
#include <drake/geometry/geometry_instance.h>
#include <iostream>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/controllers/pid_controller.h>



class PrintStateSystem : public drake::systems::LeafSystem<double> {
public:
    PrintStateSystem() {
        this->DeclareVectorInputPort("state", drake::systems::BasicVector<double>(2));
    }

    void DoCalcTimeDerivatives(const drake::systems::Context<double>& context, drake::systems::ContinuousState<double>* derivatives) const override {
        const drake::systems::BasicVector<double>* state = this->EvalVectorInput(context, 0);
        auto state_value = state->get_value();
        std::cout << "State: " << state_value(0) << " " << state_value(1) << std::endl;
    }
};

int main() {
    // Create a DiagramBuilder.
    drake::systems::DiagramBuilder<double> builder;
    double time_step = 0.001;  // Set your time step here.

    drake::multibody::MultibodyPlantConfig plant_config;
    plant_config.time_step = time_step;
    plant_config.discrete_contact_approximation = "sap";

    // Add a MultibodyPlant to the diagram.
    auto plant_result = drake::multibody::AddMultibodyPlant(plant_config, &builder);
    // plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);

    auto rigid_proximity_props = drake::geometry::ProximityProperties();
    auto surface_friction = drake::multibody::CoulombFriction(1.15, 1.15);

    rigid_proximity_props.AddProperty("hydroelastic",
                                      "resolution_hint", 0.1);
    drake::geometry::AddContactMaterial(std::nullopt, std::nullopt, surface_friction,
                                        &rigid_proximity_props);

    double width = 1.0, height = 1.0, depth = 0.4;  // Set the dimensions of the box here.
    auto ground = drake::geometry::Box(width, height, depth);


    drake::math::RigidTransform<double> X_WG(drake::math::RotationMatrix<double>(), Eigen::Vector3d(0, 0, -depth/2));

    plant_result.plant.RegisterCollisionGeometry(plant_result.plant.world_body(), X_WG, ground,
                                    "ground_collision", rigid_proximity_props);
    

    Eigen::Vector4d color(0.7, 0.5, 0.4, 0.8);

    plant_result.plant.RegisterVisualGeometry(plant_result.plant.world_body(), X_WG, ground,
                                 "ground_visual", color);


    // Parse the URDF file.
    drake::multibody::Parser parser(&plant_result.plant, &plant_result.scene_graph);
    // Create a PackageMap and add the package.
    drake::multibody::PackageMap package_map;


    std::string package_name = "deformable";  // Set your package name here.
    std::string package_directory = "..";  // Set the path to your package directory here.
    package_map.PopulateFromFolder(package_directory);
        
    // Add the package map to the parser.
    parser.package_map().AddMap(package_map);

    std::string model_file = "../urdf/cartesian.urdf";  // Set the path to your URDF file here.
    parser.AddModels(model_file);


    drake::math::RigidTransform<double> X_WB(drake::math::RotationMatrix<double>(), Eigen::Vector3d(0, 0, 0.005));

    // Add the model to the plant.
    plant_result.plant.WeldFrames(plant_result.plant.world_frame(), 
                            plant_result.plant.GetFrameByName("base"), X_WB); // Weld the base frame to the world frame.



    auto owned_deformable_model = std::make_unique<drake::multibody::DeformableModel<double>>(&plant_result.plant);
    const drake::multibody::DeformableModel<double>* model = owned_deformable_model.get();

    drake::multibody::fem::DeformableBodyConfig<double> deformable_config;

    double E = 3e6;
    // Poisson's ratio of the deformable body, unitless.
    double nu = 0.4;
    // Mass density of the deformable body [kg/m³].
    double density = 920;
    // Stiffness damping coefficient for the deformable body [1/s].
    double beta = 0.01;

    deformable_config.set_youngs_modulus(E);
    deformable_config.set_poissons_ratio(nu);
    deformable_config.set_mass_density(density);
    deformable_config.set_stiffness_damping_coefficient(beta);

    double scale = 0.01;
    drake::geometry::Mesh roloc_mesh(package_map.GetPath(package_name) + "/mesh/roloc.vtk", scale);  // Set the path to your mesh file here.

    double kL = 0.09 * scale;
    drake::math::RigidTransform<double> X_WR(drake::math::RotationMatrix<double>(), Eigen::Vector3d(0, 0, 0.5 + kL/2));

    auto roloc_instance = std::make_unique<drake::geometry::GeometryInstance>(X_WR, roloc_mesh, "deformable_roloc");

    auto deformable_proximity_props = drake::geometry::ProximityProperties();

    drake::geometry::AddContactMaterial(std::nullopt, std::nullopt, surface_friction,
                                        &deformable_proximity_props);

    roloc_instance->set_proximity_properties(
        std::move(deformable_proximity_props));

    double unused_resolution_hint = 1.0;
    
    auto roloc_id = owned_deformable_model->RegisterDeformableBody(
        std::move(roloc_instance), deformable_config, unused_resolution_hint);


    auto constraint_cylinder = drake::geometry::Cylinder(0.1, 0.1);
    drake::math::RigidTransform<double> X_CON(drake::math::RotationMatrix<double>(), Eigen::Vector3d(0, 0, -0.1));

    owned_deformable_model->AddFixedConstraint(roloc_id, plant_result.plant.GetBodyByName("spindle"), X_CON, constraint_cylinder, X_CON);

    plant_result.plant.AddPhysicalModel(std::move(owned_deformable_model));


    plant_result.plant.Finalize();


    // Define the initial and final positions.

    // Define the initial and final times.
    double initial_time = 0.0;
    double duration = 5.0;

    double initial_position = 0.0;
    double final_position = -0.35;


    // Create two pairs of times and positions.
    const std::vector<double> breaks = { initial_time, initial_time + duration/2, initial_time + duration};    
    std::vector<Eigen::MatrixXd> samples(breaks.size());
    for (auto& sample : samples) {
        sample.resize(2, 1);
    }
    samples[0](0, 0) = initial_position;
    samples[0](1, 0) = 0.0;

    samples[1](0, 0) = final_position;
    samples[1](1, 0) = 0.0;

    samples[2](0, 0) = final_position;
    samples[2](1, 0) = 0.0;
    


    // Create a PiecewisePolynomial trajectory from the waypoints.
    auto trajectory = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);

    int row = 0;
    int col = 0;

    auto trajectory_source = builder.AddSystem<drake::systems::TrajectorySource>(trajectory);

    // Create a PidController.
    Eigen::VectorXd kp(1);
    kp(0) = 1000000.0;
    Eigen::VectorXd ki(1);
    ki(0) = 0.0; // ki = 0.0, kd = 0.1; (for now, ki is not used in the pid controller, so it is set to 0.0.
    Eigen::VectorXd kd(1);
    kd(0) = 0;

    auto pid_controller = builder.AddSystem<drake::systems::controllers::PidController<double>>(kp, ki, kd);

    builder.Connect(trajectory_source->get_output_port(), pid_controller->get_input_port_desired_state());
    builder.Connect(plant_result.plant.get_state_output_port(), pid_controller->get_input_port_estimated_state());
    builder.Connect(pid_controller->get_output_port_control(), plant_result.plant.get_actuation_input_port());

    // auto print_state_system = builder.AddSystem<PrintStateSystem>();
    // builder.Connect(trajectory_source->get_output_port(), print_state_system->get_input_port(0));

    builder.Connect(
      model->vertex_positions_port(),
      plant_result.scene_graph.get_source_configuration_port(plant_result.plant.get_source_id().value()));



    drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, plant_result.scene_graph, nullptr);

    // Build the diagram.
    auto diagram = builder.Build(); 

    // Create a Simulator for the diagram.
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.Initialize();
    simulator.set_target_realtime_rate(1.0);

    // Simulate for 1 second.
    simulator.AdvanceTo(duration);

    return 0;
}