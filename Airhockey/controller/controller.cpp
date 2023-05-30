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
// #include <GL/glew.h>
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdlib>
#include <stdio.h>      /* printf */
#include <math.h>       /* signbit, sqrt */

using namespace std;
using namespace Eigen;

bool runloop = false;
void sighandler(int){runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "../redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/model/panda/panda_arm.urdf";
const string mallet_file = "./resources/model/test_objects/mallet.urdf";


double PuckMotionPrediction(Vector3d P_init, Vector3d V_init, double X_target){
	double H = 0.51;
	double k = V_init[1]/V_init[0];
	double b = P_init[1] - k*P_init[0];

	// VectorXd b_vec = VectorXd::Ones(X_target.size());
	// VectorXd Y = k*X_target+b*b_vec;
	// // cout << "Y:" << Y[0] << "," << Y[1] << "," << Y[2]<<endl;
	// VectorXd Y_target = VectorXd::Zero(X_target.size());

	// for (int m = 0; m < X_target.size(); m++) {
	// 	int n = (int)(Y[m]/(H/2));
	// 	// cout << "n(with m)=" << m << ":" << n << endl;
	// 	if (n == 0){
	// 		Y_target[m] = Y[m];
	// 	};
	// 	int a = ceil(abs(n/2.0));
	// 	// cout << "a(with m)=" << m << ":" << a << endl;;
	// 	if (a % 2 == 1){
	// 		Y_target[m] = ((n > 0) - (n < 0))*a*H-Y[m];
	// 	}
	// 	else{
	// 		Y_target[m] = Y[m]-((n > 0) - (n < 0))*a*H;
	// 	}
	// }
	// return Y_target;

	double Y = k*X_target+b;
	// cout << "Y:" << Y[0] << "," << Y[1] << "," << Y[2]<<endl;
	double Y_target;
	int n = (int)(Y/(H/2));
	// cout << "n(with m)=" << m << ":" << n << endl;
	if (n == 0){
		Y_target = Y;
	};
	int a = ceil(abs(n/2.0));
	// cout << "a(with m)=" << m << ":" << a << endl;;
	if (a % 2 == 1){
		Y_target = ((n > 0) - (n < 0))*a*H-Y;
	}
	else{
		Y_target = Y-((n > 0) - (n < 0))*a*H;
	}
	
	return Y_target;
}

enum State 
{
	START    = 0, 
	WAIT     = 1,
	STRATEGY = 2,
	ATTACK   = 3,
	DEFEND   = 4	
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
	int state = START;
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

	// prepare Robot controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	// prepare mallet controller
	int mallet_dof = mallet->dof();
	VectorXd mallet_command_torques = VectorXd::Zero(mallet_dof);
	MatrixXd mallet_N_prec = MatrixXd::Identity(mallet_dof, mallet_dof);

	// pose task (Robot and Mallet)
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

	posori_task->_kp_pos = 500.0;
	posori_task->_kv_pos = 50.0;
	posori_task->_kp_ori = 500.0;
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

	// set the current EE posiiton as the desired EE position (robot and mallet)
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

	VectorXd mallet_joint_task_torques = VectorXd::Zero(mallet_dof);
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
	timer.setLoopFrequency(600); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;

	// Temp Variable
	double stay_timer = 0.0;

	// offset value to map the marker's start position to the initial puck location.
	double mallet_X_offset = stod(redis_client.get(MALLET_Y_POS_KEY)) - 0.55;
	double mallet_Y_offset = -1.0 * stod(redis_client.get(MALLET_X_POS_KEY)) - 0.0;

//--------------------------------------------------------------------------------------------
// Extra Variable Defined for Airhocky Project
	// Puck : Position and Velocity
	Vector3d Pos;
	Vector3d Vel;
	// Marker : Mapping 
	double MappingFactor = 3.5;
	VectorXd mallet_q_desired(mallet_dof);
	// strategy : default values for attack and defend mode
	double v_thres = 1.0;		// 0.5
	double x_defend = -0.6;
	double x_attack = -0.4;
	double x_attack_delta = 0.05;	// 0.1
	double y_defend, y_attack, y_attack_puck;

	// State counter for debug purpose
	double wait_counter = 0;
	double strategy_counter = 0;
	double attack_counter = 0;

//---------------------------------------------------------------------------------------------
	while (runloop) {

		// Y_target = PuckMotionPrediction(control_point,control_point,X_target);
		// cout << "TrajectorTesting:" << Y_target[0] << "," << Y_target[1] << "," << Y_target[2] << "," << Y_target[3] << "," << Y_target[4] << endl;

		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
		mallet->updateModel();

		// update puck position and velocity (World Frame)
		Pos = redis_client.getEigenMatrixJSON(Puck_Pos);
		Vel = redis_client.getEigenMatrixJSON(Puck_Vel);
		cout << "Position of Puck is," << Pos[0] << "," << Pos[1] << "," << Pos[2] << endl;  
		cout << "Velocity of Puck is," << Vel[0] << "," << Vel[1] << "," << Vel[2] << endl;  

		// read the tracking position of marker
		Mallet_Y_Pos = -1.0 * stod(redis_client.get(MALLET_X_POS_KEY));
		Mallet_X_Pos = stod(redis_client.get(MALLET_Y_POS_KEY));
		double  mallet_Y_scaled = 0.0 + MappingFactor * 1.0 * (Mallet_Y_Pos - mallet_Y_offset - 0.0);
		double  mallet_X_scaled = 0.55 + MappingFactor * 1.5 *(Mallet_X_Pos - mallet_X_offset - 0.55);
		if (mallet_X_scaled >= 0.618) {
			mallet_X_scaled = 0.615;
		}
		
	// Mallet Controller: Scaled mapping of Marker Position
		double Mallet_Z_Pos = 0.0;
		cout << "Original Marker x-pos: " << Mallet_X_Pos << ", y-pos: " << Mallet_Y_Pos  << ", z-pos: " << Mallet_Z_Pos << endl;
		cout << "Scaled Marker x-pos: " << mallet_X_scaled << ", y-pos: " << mallet_Y_scaled  << ", z-pos: " << Mallet_Z_Pos << endl;
		mallet_q_desired << 0.0, mallet_Y_scaled , mallet_X_scaled - 0.55, 0.0, 0.0, 0.0;		
		mallet_joint_task->_desired_position = mallet_q_desired;

		mallet_N_prec.setIdentity();
		mallet_joint_task->updateTaskModel(mallet_N_prec);
		mallet_joint_task->computeTorques(mallet_joint_task_torques);
		mallet_command_torques = mallet_joint_task_torques;
		mallet->position(mallet_ee_pos, mallet_control_link, mallet_control_point);
		cout << "Actual Mallet X-pos: " << mallet_ee_pos[0] + 0.55 << ", y-pos: " << mallet_ee_pos[1] << ", z-pos: " << mallet_ee_pos[2] << endl;

	// Robot Side State 
		if (state == START) {
			// posori_task->reInitializeTask();
			// robot->position(ee_pos, control_link, control_point);
			// posori_task->_desired_position =  Vector3d(0.4, 0.1, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
			// posori_task->_desired_orientation = desired_orientation;

			// state = DEFEND;

			// update desired joint orientation
			joint_task->_desired_position = q_init_desired;

			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			if ( (robot->_q - q_init_desired).norm() < 0.15 ) {
				posori_task->reInitializeTask();
				robot->position(ee_pos, control_link, control_point);
				posori_task->_desired_position =  Vector3d(ee_pos[0], ee_pos[1], 0.56); // x: 0.3 ~ 0.6(edge at blue line)
				posori_task->_desired_orientation = desired_orientation;
				posori_task->_desired_velocity =  Vector3d(0.0, 0.0, 0.0); 
				state = WAIT;
				wait_counter++;
			}
			cout << "(START):" << endl;
			// if ( (robot->_q - q_init_desired).norm() < 0.15 ) {
			// 	cout << "Posture to Defend Mode" << endl;
			// 	posori_task->reInitializeTask();
			// 	robot->position(ee_pos, control_link, control_point);
			// 	posori_task->_desired_position =  Vector3d(0.4, 0.1, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
			// 	posori_task->_desired_orientation = desired_orientation;

			// 	state = DEFEND;
			// }
		} else if (state == WAIT){
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;

			if (Vel[0] < 0.0 && Pos[0] < 0.0){
				state = STRATEGY;
				// predict the point of defend & attack
				y_defend = PuckMotionPrediction(Pos, Vel, x_defend);
				y_attack = PuckMotionPrediction(Pos, Vel, x_attack);
				strategy_counter++; 
			}
			cout << "(WAIT):" << "Puck_x "<< Pos[0] << endl;

		} else if (state == STRATEGY){
			// Maintain Current PosOrien/Joint
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;
			
			if (Vel.norm() < v_thres){
				if (Pos[0] < x_attack + x_attack_delta){
					state = ATTACK;
					y_attack_puck = Pos[1];
					// posori_task->reInitializeTask();
					attack_counter++;
				}
			}
			else {
				state = DEFEND;
			}
			cout << "(STRATEGY):" << "Puck velocity norm: " << Vel.norm() << endl;

		} else if (state == ATTACK) {
			robot->position(ee_pos, control_link, control_point);
			posori_task->_desired_position =  Vector3d(x_attack + 1.0, y_attack, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
			posori_task->_desired_velocity =  Vector3d(0.2, 0.0, 0.0); // x: 0.3 ~ 0.6(edge at blue line)
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

			if (Vel[0] > 0.0 && Pos[0] < 0.0) {
				state = WAIT;
			}

			cout << "(ATTACK):" << endl;
			cout << "EE desired position: x: " << x_attack + 1.0 << " y: " << y_attack << endl;
			cout << "EE actual  position: x: " << ee_pos[0] << " y: " << ee_pos[1] << endl;

		} else if (state == DEFEND ) {
			robot->position(ee_pos, control_link, control_point);
			posori_task->_desired_position =  Vector3d(x_defend + 1.0, y_defend, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
			posori_task->_desired_velocity =  Vector3d(0.0, 0.0, 0.0); // x: 0.3 ~ 0.6(edge at blue line)
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

			if (Vel[0] > 0.0 && Pos[0] < 0.0) {
				state = WAIT;
			}

			cout << "(DEFEND):" << endl;
		}

		// } else if (state == DEFEND) {
		// 	cout <<"time:" << time << "  (At Defend State)" << endl;
		// 	if (stay_timer == 0.0) {
		// 		stay_timer = time;
		// 	}
			
		// 	// update task model and set hierarchy
		// 	N_prec.setIdentity();
		// 	posori_task->updateTaskModel(N_prec);
		// 	N_prec = posori_task->_N;
		// 	joint_task->updateTaskModel(N_prec);

		// 	// compute torques
		// 	posori_task->computeTorques(posori_task_torques);
		// 	joint_task->computeTorques(joint_task_torques);
		// 	command_torques = posori_task_torques + joint_task_torques;

		// 	if (time >= (stay_timer + 5.0)) {
		// 		joint_task->reInitializeTask();
		// 		posori_task->reInitializeTask();
		// 		state = ATTACK;
		// 	}
			
		// } else if (state == ATTACK) {
		// 	cout << "At Attack State" << endl;
		// 	robot->position(ee_pos, control_link, control_point);
		// 	posori_task->_desired_position =  Vector3d(0.7, 0.1, 0.56); // x: 0.3 ~ 0.6(edge at blue line)
		// 	posori_task->_desired_velocity =  Vector3d(0.0, 0.5, 0.0); // x: 0.3 ~ 0.6(edge at blue line)
		// 	posori_task->_desired_orientation = desired_orientation;

		// 	// update task model and set hierarchy
		// 	N_prec.setIdentity();
		// 	posori_task->updateTaskModel(N_prec);
		// 	N_prec = posori_task->_N;
		// 	joint_task->updateTaskModel(N_prec);

		// 	// compute torques
		// 	posori_task->computeTorques(posori_task_torques);
		// 	joint_task->computeTorques(joint_task_torques);
		// 	command_torques = posori_task_torques + joint_task_torques;

		// 	state = DEFEND;
		
		// }

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
