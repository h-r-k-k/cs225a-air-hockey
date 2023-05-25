/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller";
const std::string Puck_Pos = "sai2::sim::puck::pos";
const std::string Puck_Vel = "sai2::sim::puck::vel";

const std::string MALLET_JOINT_ANGLES_KEY = "sai2::sim::mallet::q";
const std::string MALLET_JOINT_VELOCITIES_KEY = "sai2::sim::mallet::dq";
const std::string MALLET_JOINT_TORQUES_COMMAND_KEY = "sai2::sim::mallet::actuators::fgc";


