/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "timer/LoopTimer.h"
#include <GL/glew.h>
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "../redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/model/panda/panda_arm.urdf";
const string mallet_file = "./resources/model/test_objects/mallet.urdf";



enum State 
{
	POSTURE = 0, 
	DEFEND = 1,
	ATTACK = 2	
};

int main() {
	// From simulation
	// dynamic objects information
	const vector<string> object_names = {"puck"};
	vector<Vector3d> object_pos;
	vector<Vector3d> object_lin_vel;
	vector<Quaterniond> object_ori;
	vector<Vector3d> object_ang_vel;
	const int n_objects = object_names.size();

	double Mallet_X_Pos;
	double Mallet_Y_Pos;

	// initial state 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	auto mallet = new Sai2Model::Sai2Model(mallet_file, false);

	//robot
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	//mallet
	mallet->_q = redis_client.getEigenMatrixJSON(MALLET_JOINT_ANGLES_KEY);
	mallet->_dq = redis_client.getEigenMatrixJSON(MALLET_JOINT_VELOCITIES_KEY);
	mallet->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	// prepare mallet controller
	int mallet_dof = mallet->dof();
	VectorXd mallet_command_torques = VectorXd::Zero(mallet_dof);
	MatrixXd mallet_N_prec = MatrixXd::Identity(mallet_dof, mallet_dof);


	// pose task
	const string control_link = "link7";
	const string mallet_control_link = "link6";
	const Vector3d control_point = Vector3d(0, 0, 0.145);
	const Vector3d mallet_control_point = Vector3d(0, 0, -0.015);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	auto mallet_posori_task = new Sai2Primitives::PosOriTask(mallet, mallet_control_link, mallet_control_point);

	posori_task->_use_interpolation_flag = false;
	posori_task->_use_velocity_saturation_flag = false;
	mallet_posori_task->_use_interpolation_flag = false;
	mallet_posori_task->_use_velocity_saturation_flag = false;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	VectorXd mallet_posori_task_torques = VectorXd::Zero(mallet_dof);

	posori_task->_kp_pos = 300.0;
	posori_task->_kv_pos = 50.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 30.0;

	mallet_posori_task->_kp_pos = 300.0;
	mallet_posori_task->_kv_pos = 50.0;
	mallet_posori_task->_kp_ori = 200.0;
	mallet_posori_task->_kv_ori = 30.0;

	Matrix3d desired_orientation;
	desired_orientation << 	1,  0,  0,
							0, -1,  0,
							0,  0, -1;
	
	Matrix3d mallet_desired_orientation;
	mallet_desired_orientation << 	1,  0,  0,
									0, -1,  0,
									0,  0, -1;
	// set the current EE posiiton as the desired EE position
	Vector3d x_desired = Vector3d::Zero(3);
	robot->position(x_desired, control_link, control_point);
	mallet->position(x_desired, mallet_control_link, mallet_control_point);
	Vector3d x_init = x_desired;

	// joint task robot
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = false;
	// joint_task->_saturation_velocity << 0.2,0.2,0.2;

	// joint task mallet
	auto mallet_joint_task = new Sai2Primitives::JointTask(mallet);
	mallet_joint_task->_use_interpolation_flag = true;
	mallet_joint_task->_use_velocity_saturation_flag = false;
	// mallet_joint_task->_saturation_velocity << 0.2,0.2,0.2;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 300.0;
	joint_task->_kv = 40.0;

	mallet_joint_task->_kp = 300.0;
	mallet_joint_task->_kv = 40.0;

	VectorXd q_init_desired(dof);
	q_init_desired << 0.0, 0.0, 0.0, -90.0, 0.0, 90.0, 0.0;

	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;
	// containers
	Vector3d mallet_ee_pos;
	Matrix3d mallet_ee_rot;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToReadCallback(0, MALLET_JOINT_ANGLES_KEY, mallet->_q);
	redis_client.addEigenToReadCallback(0, MALLET_JOINT_VELOCITIES_KEY, mallet->_dq);
	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addEigenToWriteCallback(0, MALLET_JOINT_TORQUES_COMMAND_KEY, mallet_command_torques);
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	double mallet_X_offset = stod(redis_client.get(MALLET_Y_POS_KEY)) - 0.0;
	double mallet_Y_offset = stod(redis_client.get(MALLET_X_POS_KEY)) - 0.0;

	runloop = true;

	// Temp Variable
	double stay_timer = 0.0;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
		mallet->updateModel();
		// update puck velocity
		int i = 0;
		Vector3d Pos;
		Vector3d Vel;

		Pos = redis_client.getEigenMatrixJSON(Puck_Pos);
		Vel = redis_client.getEigenMatrixJSON(Puck_Vel);

		cout << "Position of Puck is," << Pos[0] << "," << Pos[1] << "," << Pos[2] << endl;  
		cout << "Velocity of Puck is," << Vel[0] << "," << Vel[1] << "," << Vel[2] << endl;  

		// Vector3d Mallet_Pos = Vector3d(-0.4,0.0,0.0);
		double MappingFactor = 1.5;
		Mallet_Y_Pos = stod(redis_client.get(MALLET_X_POS_KEY));
		Mallet_X_Pos = stod(redis_client.get(MALLET_Y_POS_KEY));
		double  mallet_Y_scaled = 0.0 + MappingFactor * (Mallet_Y_Pos - mallet_Y_offset - 0.0);
		double  mallet_X_scaled = 0.0 + MappingFactor * (Mallet_X_Pos - mallet_X_offset - 0.0);
		// dy = MappingFactor * (Mallet_Y_Pos - Mallet_Y_Pos_His);
		// dx = MappingFactor * (Mallet_X_Pos - Mallet_X_Pos_His);
		// cout << "dx" << dx << "," << "dy" << dy << endl;


		double Mallet_Z_Pos = 0.0;
		cout << "Mallet x-pos: " << Mallet_X_Pos << ", y-pos: " << Mallet_Y_Pos  << ", z-pos: " << Mallet_Z_Pos << endl;
		Vector3d Mallet_Pos = Vector3d(mallet_X_scaled, mallet_Y_scaled, Mallet_Z_Pos);
		mallet_posori_task->reInitializeTask();
		mallet->position(mallet_ee_pos, mallet_control_link, mallet_control_point);
		mallet_posori_task->_desired_position =  Mallet_Pos; // x: 0.3 ~ 0.6(edge at blue line)
		mallet_posori_task->_desired_orientation = mallet_desired_orientation;
		mallet_N_prec.setIdentity();
		mallet_posori_task->updateTaskModel(mallet_N_prec);
		mallet_posori_task->computeTorques(mallet_command_torques);
		cout << "Torque is:" << mallet_command_torques[0] << "," << mallet_command_torques[1] << "," << mallet_command_torques[2] << "," << mallet_command_torques[3] << "," << mallet_command_torques[4] << "," << mallet_command_torques[5] << endl;


		if (state == POSTURE) {
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			if ( (robot->_q - q_init_desired).norm() < 0.15 ) {
				cout << "Posture to Defend Mode" << endl;
				// joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				robot->position(ee_pos, control_link, control_point);
				// posori_task->_desired_position =   x_init + Vector3d(0.2, 0.0, -0.4);
				// cout << posori_task->_desired_position << "\n" << endl;
				posori_task->_desired_position =  Vector3d(0.4, 0.1, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
				// posori_task->_desired_position =  Vector3d(0.4, 0.15, 0.6); // x: 0.3 ~ 0.6(edge at blue line)
				// posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				posori_task->_desired_orientation = desired_orientation;

				state = DEFEND;
			}
		} else if (state == DEFEND) {
			cout <<"time:" << time << "  (At Defend State)" << endl;
			if (stay_timer == 0.0) {
				stay_timer = time;
			}
			
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;

			if (time >= (stay_timer + 5.0)) {
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				state = ATTACK;
			}
			
		} else if (state == ATTACK) {
			cout << "At Attack State" << endl;
			robot->position(ee_pos, control_link, control_point);
			posori_task->_desired_position =  Vector3d(0.7, 0.1, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
			posori_task->_desired_velocity =  Vector3d(0.3, 0.3, 0); // x: 0.3 ~ 0.6(edge at blue line)
			posori_task->_desired_orientation = desired_orientation;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;

			state = DEFEND;
		
		}

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating
	redis_client.setEigenMatrixJSON(MALLET_JOINT_TORQUES_COMMAND_KEY, 0 * mallet_command_torques);  // back to floating

	return 0;
}
