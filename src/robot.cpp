#include "robot.hpp"
#include "plan.hpp"

using namespace std;




cosCurve s1(1.0, 2 * PI, 0);




namespace robot
{

	auto ModelSetPos::prepareNrt()->void
	{
        for (auto& m : motorOptions()) m = aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelSetPos::executeRT()->int
	{
		/////example1//////

		double input_angle[12] =
		{ 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0,
		0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

		double input_angle1[6] =
		{ 0, 0, 0, 0, 0, 0 };

		double input_angle2[6] =
		{ 0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 ->white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 ->blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


		//double eepA1[6] = { 0 };
		//double eepBa[12] = { 0 };

		//model_a1.setInputPos(input_angle1);
		//if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
		//model_a1.getOutputPos(eepA1);

		////std::cout << "Arm1" << std::endl;
		////aris::dynamic::dsp(1, 6, eepA1);


		//double ee1_pm[16]{ 0 };
		//eeA1.getMpm(ee1_pm);
		////mout() << "pm:" << std::endl;
		////aris::dynamic::dsp(1, 16, ee1_pm);
		//
		//dualArm.setInputPos(input_angle);
		//if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;
		//dualArm.getOutputPos(eepBa);

		////std::cout << "dual" << std::endl;
		////aris::dynamic::dsp(1, 12, eepBa);

		//double Arm1_angle[6]{ 0 };
		//double Dual_angle[12]{ 0 };


		//model_a1.setOutputPos(eepA1);
		//if (model_a1.inverseKinematics())std::cout << "inverse failed" << std::endl;
		//model_a1.getInputPos(Arm1_angle);

		////std::cout << "inverse Arm1" << std::endl;
		////aris::dynamic::dsp(1, 6, Arm1_angle);


		//dualArm.setOutputPos(eepBa);
		//if (dualArm.inverseKinematics())std::cout << "inverse failed" << std::endl;
		//dualArm.getInputPos(Dual_angle);
		//
		////std::cout << "inverse dual" << std::endl;
		////aris::dynamic::dsp(1, 12, Dual_angle);


		////position
		//double a1_stcp[6]{};
		//double a2_stcp[12]{};
		////velocity
		//double a1_vtcp[6]{};
		//double a2_vtcp[12]{};

		////get ee
		//

		//eeA1.getP(a1_stcp);
		////std::cout << "Arm1 ee pos" << std::endl;
		////aris::dynamic::dsp(1, 6, a1_stcp);

		////std::cout << "Arm1 ee vel" << std::endl;
		//eeA1.getV(a1_vtcp);
		////aris::dynamic::dsp(1, 6, a1_vtcp);

		//eeA1.updP();

		////both arm move
		//auto baMove = [&](double* pos_) {

		//	dualArm.setOutputPos(pos_);

		//	if (dualArm.inverseKinematics())
		//	{
		//		throw std::runtime_error("Inverse Kinematics Position Failed!");
		//	}


		//	double x_joint[12]{ 0 };

		//	dualArm.getInputPos(x_joint);


		//	for (std::size_t i = 0; i < 12; ++i)
		//	{
		//		controller()->motorPool()[i].setTargetPos(x_joint[i]);
		//	}

		//};

		////single arm move 1-->white 2-->blue
		//auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

		//	model_.setOutputPos(pos_);

		//	if (model_.inverseKinematics())
		//	{
		//		throw std::runtime_error("Inverse Kinematics Position Failed!");
		//	}


		//	double x_joint[6]{ 0 };

		//	model_.getInputPos(x_joint);

		//	if (type_ == 0)
		//	{
		//		for (std::size_t i = 0; i < 6; ++i)
		//		{
		//			controller()->motorPool()[i].setTargetPos(x_joint[i]);
		//		}
		//	}
		//	else if (type_ == 1)
		//	{
		//		for (std::size_t i = 0; i < 6; ++i)
		//		{
		//			controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
		//		}
		//	}
		//	else
		//	{
		//		throw std::runtime_error("Arm Type Error");
		//	}
		//};



		//double posTest[6]{ 0.402373, 0.124673, 0.136492, 6.283185, 0.000000, 3.141593 };
		//saMove(posTest, model_a1, 1);

		//double test[12]{};
		//dualArm.getOutputPos(test);

		//std::cout << "test" << std::endl;
		//aris::dynamic::dsp(1, 12, test);





		////example1 end/////

		////GC test////

		//double force_data_1[6]{ 0.126953, 0, 0.410156, 0.00234375, 0.00390625, 0.00117188 };
		//double force_data_2[6]{ -0.136719, 0.078125, 0.664062, 0.00507813, 0.00351563, 0.00117188 };
		//double force_data_3[6]{ -0.0976562, -0.195312, 0.410156, 0.00507813, 0.00273437, 0.00117188 };

		double force_data_1[6]{ 2.0856,	6.357,	4.906,	0.0423, - 0.0283,	0.0808 };
		double force_data_2[6]{ 2.1013,	4.5775,	7.9831,	0.0378, - 0.0286,	0.0807 };
		double force_data_3[6]{ 3.0116,	5.8671,	6.4544,	0.0428 ,- 0.0324,	0.0806 };
		

		static double pos2_1[12]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };
		static double pos2_2[12]{ 0, 0, -5 * PI / 6, PI / 2, PI / 2, PI / 2 };
		static double pos2_3[12]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, PI / 2 };

		//double ee_pm_1[16]{ -1.95049e-15, -3.56239e-15, 1, 0.393,
		//					-3.49721e-15, 1, 3.42057e-15, 2.49707e-16,
		//					-1, -3.48575e-15, -4.9181e-15, 0.642,
		//					0, 0, 0, 1 };
		//double ee_pm_2[16]{ -1, 3.22243e-15, -3.33438e-15, 0.32,
		//					3.23975e-15, 1, 3.23109e-15, 2.35869e-16,
		//					3.45997e-15, 3.23109e-15, -1, 0.569,
		//					0, 0, 0, 1 };
		//double ee_pm_3[16]{ -1, 1.02138e-14, -3.36091e-15, 0.32,
		//					-3.37713e-15, -3.17117e-15, 1, 0.073,
		//					1.02143e-14, 1, 3.17117e-15, 0.642,
		//					0, 0, 0, 1 };

		double ee_pm_1[16]{ 0 };
		double ee_pm_2[16]{ 0 };
		double ee_pm_3[16]{ 0 };

		model_a2.setInputPos(pos2_1);
		if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
		eeA2.getMpm(ee_pm_1);

		model_a2.setInputPos(pos2_2);
		if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
		eeA2.getMpm(ee_pm_2);

		model_a2.setInputPos(pos2_3);
		if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
		eeA2.getMpm(ee_pm_3);


		double t_vector[9]{ 0 };
		double f_vector[9]{ 0 };

		double f_matrix[54]{ 0 };
		double r_matrix[54]{ 0 };

		double p_vector[6]{ 0 };
		double l_vector[6]{ 0 };



		double ee_rm_1[9]{ 0 };
		double ee_rm_2[9]{ 0 };
		double ee_rm_3[9]{ 0 };

		double current_force[6]{ 0 };

		aris::dynamic::s_pm2rm(ee_pm_1, ee_rm_1);
		aris::dynamic::s_pm2rm(ee_pm_2, ee_rm_2);


		aris::dynamic::s_pm2rm(ee_pm_3, ee_rm_3);

		GravComp gc;

		gc.getTorqueVector(force_data_1, force_data_2, force_data_3, t_vector);
		gc.getForceVector(force_data_1, force_data_2, force_data_3, f_vector);

		gc.getFMatrix(force_data_1, force_data_2, force_data_3, f_matrix);
		gc.getRMatrix(ee_rm_1, ee_rm_2, ee_rm_3, r_matrix);

		gc.getPLMatrix(f_matrix, t_vector, p_vector);
		gc.getPLMatrix(r_matrix, f_vector, l_vector);




		double current_pose_a1[16] = { -1, 6.63139e-15, -4.70536e-15, 0.32,
										1.33547e-15, 0.707107, 0.707107, 0.0516188,
										8.30063e-15, 0.707107, -0.707107, 0.590381,
										0, 0, 0, 1 };

		double test_force1[6]{ -0.0976562, -0.0976562, 0.595703, 0.00546875, 0.00234375, 0.00117188 };



		double comp_f[6]{ 0 };
		gc.getCompFT(ee_pm_1, l_vector, p_vector, comp_f);


		double final_force[6]{ 0 };
		for (int i = 0; i < 6; i++)
		{
			final_force[i] = force_data_1[i] + comp_f[i];
		}

		mout() << "ff: " << std::endl;
		aris::dynamic::dsp(1, 6, final_force);





	


		return 0;
	}
	ModelSetPos::ModelSetPos(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_set\"/>");
	}
	ModelSetPos::~ModelSetPos() = default;



	auto ModelForward::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelForward::executeRT()->int
	{
		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
        static double init_pos[12]{};
        double input_angle[12]{};
        double ee_pos[12]{};
		
		if (count() == 1)
		{

            double begin_angle[12]{ 0 };

			for (int i = 0; i < 12; i++)
			{
				begin_angle[i] = controller()->motorPool()[i].targetPos();
			}

			aris::dynamic::dsp(1, 12, begin_angle);

			//this->master()->logFileRawName("move");
			mout() << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			mout() << "input" << std::endl;

			modelBase()->getOutputPos(init_pos);

			aris::dynamic::dsp(1, 12, init_pos);

		}



			ee_pos[0] = init_pos[0] + 0.01 * s1.getTCurve(count());
		    ee_pos[1] = init_pos[1] - 0.01 * s1.getTCurve(count());

		    ee_pos[2] = init_pos[2];
		    ee_pos[3] = init_pos[3];
		    ee_pos[4] = init_pos[4];
		    ee_pos[5] = init_pos[5];

			ee_pos[6] = init_pos[6];
			ee_pos[7] = init_pos[7];
			ee_pos[8] = init_pos[8];
			ee_pos[9] = init_pos[9];
			ee_pos[10] = init_pos[10];
			ee_pos[11] = init_pos[11];


		modelBase()->setOutputPos(ee_pos);
		if (modelBase()->inverseKinematics())
		{
			throw std::runtime_error("Inverse Kinematics Position Failed!");
		}



		modelBase()->getInputPos(input_angle);

		for (int i = 0; i < 12; i++)
		{
			controller()->motorPool()[i].setTargetPos(input_angle[i]);
		}

		if (count() % 100 == 0)
		{
		mout() <<"Arm1:" << input_angle[0]*180/PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
			<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI <<"\n"
			<<"Arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
			<< input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << count() << std::endl;
		}
		


		return 10000 - count();

	}
	ModelForward::ModelForward(const std::string& name)
	{

        aris::core::fromXmlString(command(),
            "<Command name=\"m_forward\">"
            "</Command>");
	}
	ModelForward::~ModelForward() = default;



	auto ModelTest::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelTest::executeRT() -> int
	{

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 ->white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 ->blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);


		double force_data1[6]{ 0.126953, 0, 0.410156, 0.00234375, 0.00390625, 0.00117188 };
		double force_data2[6]{ -0.136719, 0.078125, 0.664062, 0.00507813, 0.00351563, 0.00117188 };
		double force_data3[6]{ -0.0976562, -0.195312, 0.410156, 0.00507813, 0.00273437, 0.00117188 };


		double pose1[9]{ -1.95049e-15, -3.56239e-15, 1,
						-3.49721e-15, 1, 3.42057e-15,
						-1, -3.48575e-15, -4.9181e-15 };

		double pose2[9]{ -1, 3.22243e-15, -3.33438e-15,
						3.23975e-15, 1, 3.23109e-15,
						3.45997e-15, 3.23109e-15, -1 };

		double pose3[9]{ -1, 1.02138e-14, -3.36091e-15,
						-3.37713e-15, -3.17117e-15, 1,
						1.02143e-14, 1, 3.17117e-15 };

		//EE Force Compensate Caculation
		//Refer to 10.16383/j.aas.2017.c150753

		//p = [x, y, z, k1, k2, k3]T
		//m = [mx1, my1, mz1, ... mx3, my3, mz3]T
		//p = inv(F.t * F) * F.t * m


		//l = [Lx, Ly, Lz, Fx0, Fy0, Fz0]T
		//f = [Fx1, Fy1, Fz1, ... Fx3, Fy3, Fz3]T
		//l = inv(R.t * R) * R.t * f

		//Get m
		auto getTorqueVector = [](double force_data1_[6], double force_data2_[6], double force_data3_[6], double torque_vector_[9]) {
			std::copy(force_data1_ + 3, force_data1_ + 6, torque_vector_);
			std::copy(force_data2_ + 3, force_data2_ + 6, torque_vector_ + 3);
			std::copy(force_data3_ + 3, force_data3_ + 6, torque_vector_ + 6);
		};


		//Get f
		auto getForceVector = [](double force_data1_[6], double force_data2_[6], double force_data3_[6], double force_vector_[9]) {
			std::copy(force_data1_, force_data1_ + 3, force_vector_);
			std::copy(force_data2_, force_data2_ + 3, force_vector_ + 3);
			std::copy(force_data3_, force_data3_ + 3, force_vector_ + 6);
		};


		//Temp To Caculate F
		auto getTempFMatrix = [](double force_data_[6], double temp_[18]) {

			double tempInit[18] = { 0, force_data_[2], -force_data_[1], 1, 0, 0,
									-force_data_[2], 0, force_data_[0], 0, 1, 0,
									force_data_[1], -force_data_[0], 0, 0, 0, 1 };

			std::copy(tempInit, tempInit + 18, temp_);

		};

		//Temp To Caculate R
		auto getTempRMatrix = [](double pose_matrix_[9], double temp_[18]) {

			double tempInit[18] = { pose_matrix_[0], pose_matrix_[3], pose_matrix_[6], 1, 0, 0,
									pose_matrix_[1], pose_matrix_[4], pose_matrix_[7], 0, 1, 0,
									pose_matrix_[2], pose_matrix_[5], pose_matrix_[8], 0, 0, 1 };

			std::copy(tempInit, tempInit + 18, temp_);

		};

		//Get F
		auto getFMatrix = [getTempFMatrix](double force_data1_[6], double force_data2_[6], double force_data3_[6], double f_matrix_[54]) {

			double temp1[18] = { 0 };
			double temp2[18] = { 0 };
			double temp3[18] = { 0 };

			getTempFMatrix(force_data1_, temp1);
			getTempFMatrix(force_data2_, temp2);
			getTempFMatrix(force_data3_, temp3);

			std::copy(temp1, temp1 + 18, f_matrix_);
			std::copy(temp2, temp2 + 18, f_matrix_ + 18);
			std::copy(temp3, temp3 + 18, f_matrix_ + 36);

		};

		//Get R
		auto getRMatrix = [getTempRMatrix](double pose_matrix1_[9], double pose_matrix2_[9], double pose_matrix3_[9], double r_matrix_[54]) {

			double temp1[18] = { 0 };
			double temp2[18] = { 0 };
			double temp3[18] = { 0 };

			getTempRMatrix(pose_matrix1_, temp1);
			getTempRMatrix(pose_matrix2_, temp2);
			getTempRMatrix(pose_matrix3_, temp3);

			std::copy(temp1, temp1 + 18, r_matrix_);
			std::copy(temp2, temp2 + 18, r_matrix_ + 18);
			std::copy(temp3, temp3 + 18, r_matrix_ + 36);

		};



		// Caculate Least Square To Get P && L Vector
		auto getPLVector = [](double f_r_matrix_[54], double torque_force_data_[9], double P_L[6]) {

			//   m : �У�  n : �� 
			//   rank: ���Բ���д
			//   U :        m x n
			//  tau : max(m,n) x 1
			//    p : max(m,n) x 1
			//    x :        n x m

			// refer to math_matrix.hpp in aris

			double U[54]{ 0 };
			double Inv_[54]{ 0 };
			double tau[9]{ 0 };


			aris::Size p[9];
			aris::Size rank;

			// utp decomption, 1e-5 ����
			aris::dynamic::s_householder_utp(9, 6, f_r_matrix_, U, tau, p, rank, 1e-6);
			// inverse
			aris::dynamic::s_householder_up2pinv(9, 6, rank, U, p, Inv_, 1e-6);
			// multiple
			aris::dynamic::s_mm(6, 1, 9, Inv_, torque_force_data_, P_L);

		};

		double P[6]{ 0 };
		double torque[9]{ 0 };
		double f_matrix[54]{ 0 };

		getTorqueVector(force_data1, force_data2, force_data3, torque);
		getFMatrix(force_data1, force_data2, force_data3, f_matrix);
		getPLVector(f_matrix, torque, P);
		aris::dynamic::dsp(1, 6, P);


		double L[6]{ 0 };
		double force[9]{ 0 };
		double r_matrix[54]{ 0 };

		getForceVector(force_data1, force_data2, force_data3, force);
		getRMatrix(pose1, pose2, pose3, r_matrix);
		getPLVector(r_matrix, force, L);
		aris::dynamic::dsp(1, 6, L);

		aris::dynamic::dsp(1, 54, r_matrix);

		double tool_x = P[0];
		double tool_y = P[1];
		double tool_z = P[2];

		double k1 = P[3];
		double k2 = P[4];
		double k3 = P[5];

		double Lx = L[0];
		double Ly = L[1];
		double Lz = L[2];

		double Fx0 = L[3];
		double Fy0 = L[4];
		double Fz0 = L[5];

		double tool_load = sqrt(Lx * Lx + Ly * Ly + Lz * Lz);

		//std::cout << "load: " << tool_load << std::endl;

		double U = std::asin(-Ly / tool_load);
		double V = std::atan(-Lx / Ly);

		double Mx0 = k1 - Fy0 * tool_z - Fz0 * tool_y;
		double My0 = k2 - Fz0 * tool_x - Fx0 * tool_z;
		double Mz0 = k3 - Fx0 * tool_y - Fy0 * tool_x;

		//get ee
		//auto& eeA1 = model_a1.generalMotionPool().at(0);
		//auto& eeA2 = model_a2.generalMotionPool().at(0);

		//auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		//auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		//double current_pose_a1[16]{ 0 };
		//double current_pose_a2[9]{ 0 };

		//double input_angle1[6] =
		//{ 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0 };
		//double eepA1[6] = { 0 };

		//model_a1.setInputPos(input_angle1);
		//if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
		//model_a1.getOutputPos(eepA1);



		//eeA1.getMpm(current_pose_a1);

		//eeA2.getMpm(current_pose_a2);
		//aris::dynamic::dsp(1, 16, current_pose_a1);

		double current_pose_a1[16] = { -1, 6.63139e-15, -4.70536e-15, 0.32,
										1.33547e-15, 0.707107, 0.707107, 0.0516188,
										8.30063e-15, 0.707107, -0.707107, 0.590381,
										0, 0, 0, 1 };

		double current_pose_a2[16] = { -0.5, 1.97435e-14, 0.866025, 0.38322,
										0.433013, 0.866025, 0.25, 0.01825,
										-0.75, 0.5, -0.433013, 0.61039,
										0, 0, 0, 1 };

		double current_pose_a3[16] = { 0.766044, -3.87809e-15, 0.642788, 0.366923,
										0.321394, 0.866025, -0.383022, -0.0279606,
										-0.55667, 0.5, 0.663414, 0.690429,
										0, 0, 0, 1 };

		double current_pose_a4[16] = { 0.866025, 8.73219e-16, 0.5, 0.3565,
										0.0868241, 0.984808, -0.150384, -0.010978,
										-0.492404, 0.173648, 0.852869, 0.704259,
										0, 0, 0, 1 };


		auto getCompFT = [](double current_pose_[16], double L_[6], double P_[6], double comp_f_[6]) {

			double current_rotate[9]{ 0 };
			double inv_rotate[9]{ 0 };

			double G_vector[3]{ 0 };

			double F_vector[3]{ 0 };
			double L_vector[3]{ 0 };

			double Mass_center[3]{ 0 };
			double K_vector[3]{ 0 };

			double Mg[3]{ 0 };
			double M0[3]{ 0 };

			std::copy(L_ + 3, L_ + 6, F_vector);//fx0 fy0 fz0
			//aris::dynamic::dsp(1, 3, F_vector);
			std::copy(L_, L_ + 3, L_vector);// lx ly lz -> Gx Gy Gz
			std::copy(P_, P_ + 3, Mass_center);//x0 y0 z0 -> Mgx Mgy Mgz
			std::copy(P_ + 3, P_ + 6, K_vector);//k1 k2 k3 -> Mx0 My0 Mz0

			aris::dynamic::s_pm2rm(current_pose_, current_rotate);
			aris::dynamic::s_inv_rm(current_rotate, inv_rotate);

			aris::dynamic::s_rm_dot_v3(inv_rotate, L_vector, G_vector);


			//aris::dynamic::dsp(1, 3, G_vector);

			//comp x y z
			for (int i = 0; i < 3; i++)
			{
				comp_f_[i] = -F_vector[i] - G_vector[i];
			}


			Mg[0] = -G_vector[2] * Mass_center[1] - G_vector[1] * Mass_center[2];
			Mg[1] = -G_vector[0] * Mass_center[2] - G_vector[2] * Mass_center[0];
			Mg[2] = -G_vector[1] * Mass_center[0] - G_vector[0] * Mass_center[1];

			M0[0] = K_vector[0] - F_vector[1] * Mass_center[2] - F_vector[2] * Mass_center[1];
			M0[1] = K_vector[1] - F_vector[2] * Mass_center[0] - F_vector[1] * Mass_center[2];
			M0[2] = K_vector[2] - F_vector[0] * Mass_center[1] - F_vector[0] * Mass_center[0];


			//comp mx my mz
			for (int i = 3; i < 6; i++)
			{
				comp_f_[i] = -M0[i - 3] - Mg[i - 3];
			}


		};

		double ee_pm_1[16]{ -1.95049e-15, -3.56239e-15, 1, 0.393,
							-3.49721e-15, 1, 3.42057e-15, 2.49707e-16,
							-1, -3.48575e-15, -4.9181e-15, 0.642,
							0, 0, 0, 1 };


		double comp1[6]{ 0 };

		getCompFT(ee_pm_1, L, P, comp1);
		aris::dynamic::dsp(1, 6, comp1);


		double test_force1[6]{ -0.0976562, -0.0976562, 0.595703, 0.00546875, 0.00234375, 0.00117188 };
		double comp_f[6]{ 0 };



		for (int i = 0; i < 6; i++)
		{
			comp_f[i] = force_data1[i] + comp1[i];
			std::cout << "lambda test " << i << " force: " << comp_f[i] << std::endl;
		}








		//double current_rotation_m1[9]{ 0 };
		//double current_rotation_m2[9]{ 0 };

		//double current_rotation_m3[9]{ 0 };
		//double current_rotation_m4[9]{ 0 };


		//aris::dynamic::s_pm2rm(current_pose_a1, current_rotation_m1);
		//aris::dynamic::s_pm2rm(current_pose_a2, current_rotation_m2);

		//aris::dynamic::s_pm2rm(current_pose_a3, current_rotation_m3);
		//aris::dynamic::s_pm2rm(current_pose_a4, current_rotation_m4);


		//double inv_current_rotation_m1[9]{ 0 };
		//double inv_current_rotation_m2[9]{ 0 };

		//double inv_current_rotation_m3[9]{ 0 };
		//double inv_current_rotation_m4[9]{ 0 };

		//aris::dynamic::s_inv_rm(current_rotation_m1, inv_current_rotation_m1);


		//aris::dynamic::s_inv_rm(current_rotation_m2, inv_current_rotation_m2);

		//aris::dynamic::s_inv_rm(current_rotation_m3, inv_current_rotation_m3);
		//aris::dynamic::s_inv_rm(current_rotation_m4, inv_current_rotation_m4);
		//




		//double L_vector[3] = { Lx,Ly,Lz };

		//double G_vector1[3]{ 0 };
		//double G_vector2[3]{ 0 };

		//double G_vector3[3]{ 0 };
		//double G_vector4[3]{ 0 };

		//

		//double test_force2[6]{ 0.0390625, - 0.0878906, 0.527344, 0.00390625, 0.00429688, 0.00117188 };

		//double test_force3[6]{ 0.0683594, - 0.078125, 0.205078, 0.00390625, 0.00234375, 0.00117188 };

		//double test_force4[6]{ 0.0390625, 0, 0.136719, 0.00390625, 0.00234375, 0.0015625 };



		//aris::dynamic::s_rm_dot_v3(inv_current_rotation_m1, L_vector, G_vector1);


		//aris::dynamic::dsp(1, 3, G_vector1);

		//aris::dynamic::s_rm_dot_v3(inv_current_rotation_m2, L_vector, G_vector2);

		//aris::dynamic::s_rm_dot_v3(inv_current_rotation_m3, L_vector, G_vector3);
		//aris::dynamic::s_rm_dot_v3(inv_current_rotation_m4, L_vector, G_vector4);


		//std::cout << "111 " << -Fx0 - G_vector1[0] << std::endl;

		//double Fex1 = test_force1[0] - Fx0 - G_vector1[0];
		//double Fey1 = test_force1[1] - Fy0 - G_vector1[1];
		//double Fez1 = test_force1[2] - Fz0 - G_vector1[2];

		//double Fex2 = test_force2[0] - Fx0 - G_vector2[0];
		//double Fey2 = test_force2[1] - Fy0 - G_vector2[1];
		//double Fez2 = test_force2[2] - Fz0 - G_vector2[2];

		//std::cout << "Fex1: " << Fex1 << std::endl;
		//std::cout << "Fey1: " << Fey1 << std::endl;
		//std::cout << "Fez1: " << Fez1 << std::endl;

		////std::cout << "Fex2: " << Fex2 << std::endl;
		////std::cout << "Fey2: " << Fey2 << std::endl;
		////std::cout << "Fez2: " << Fez2 << std::endl;


		//double Mgx1 = -G_vector1[2] * tool_y - G_vector1[1] * tool_z;
		//double Mgy1 = -G_vector1[0] * tool_z - G_vector1[2] * tool_x;
		//double Mgz1 = -G_vector1[1] * tool_x - G_vector1[0] * tool_y;

		//double Mgx2 = -G_vector2[2] * tool_y - G_vector2[1] * tool_z;
		//double Mgy2 = -G_vector2[0] * tool_z - G_vector2[2] * tool_x;
		//double Mgz2 = -G_vector2[1] * tool_x - G_vector2[0] * tool_y;

		//double Mex1 = test_force1[3] - Mx0 - Mgx1;
		//double Mey1 = test_force1[4] - My0 - Mgy1;
		//double Mez1 = test_force1[5] - Mz0 - Mgz1;

		//double Mex2 = test_force2[3] - Mx0 - Mgx2;
		//double Mey2 = test_force2[4] - My0 - Mgy2;
		//double Mez2 = test_force2[5] - Mz0 - Mgz2;


		////std::cout << "Mex1: " << Mex1 << std::endl;
		////std::cout << "Mey1: " << Mey1 << std::endl;
		////std::cout << "Mez1: " << Mez1 << std::endl;






		//double Fex3 = test_force3[0] - Fx0 - G_vector3[0];
		//double Fey3 = test_force3[1] - Fy0 - G_vector3[1];
		//double Fez3 = test_force3[2] - Fz0 - G_vector3[2];

		//double Fex4 = test_force4[0] - Fx0 - G_vector4[0];
		//double Fey4 = test_force4[1] - Fy0 - G_vector4[1];
		//double Fez4 = test_force4[2] - Fz0 - G_vector4[2];

		////std::cout << "Fex3: " << Fex3 << std::endl;
		////std::cout << "Fey3: " << Fey3 << std::endl;
		////std::cout << "Fez3: " << Fez3 << std::endl;

		////std::cout << "Fex4: " << Fex4 << std::endl;
		////std::cout << "Fey4: " << Fey4 << std::endl;
		////std::cout << "Fez4: " << Fez4 << std::endl;


		//double Mgx3 = -G_vector3[2] * tool_y - G_vector3[1] * tool_z;
		//double Mgy3 = -G_vector3[0] * tool_z - G_vector3[2] * tool_x;
		//double Mgz3 = -G_vector3[1] * tool_x - G_vector3[0] * tool_y;

		//double Mgx4 = -G_vector4[2] * tool_y - G_vector4[1] * tool_z;
		//double Mgy4 = -G_vector4[0] * tool_z - G_vector4[2] * tool_x;
		//double Mgz4 = -G_vector4[1] * tool_x - G_vector4[0] * tool_y;

		//double Mex3 = test_force3[3] - Mx0 - Mgx3;
		//double Mey3 = test_force3[4] - My0 - Mgy3;
		//double Mez3 = test_force3[5] - Mz0 - Mgz3;

		//double Mex4 = test_force4[3] - Mx0 - Mgx4;
		//double Mey4 = test_force4[4] - My0 - Mgy4;
		//double Mez4 = test_force4[5] - Mz0 - Mgz4;


		////std::cout << "Mex3: " << Mex3 << std::endl;
		////std::cout << "Mey3: " << Mey3 << std::endl;
		////std::cout << "Mez3: " << Mez3 << std::endl;

		////std::cout << "Mex4: " << Mex4 << std::endl;
		////std::cout << "Mey4: " << Mey4 << std::endl;
		////std::cout << "Mez4: " << Mez4 << std::endl;






		return 0;
	}
	ModelTest::ModelTest(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_test\"/>");
	}
	ModelTest::~ModelTest() = default;




	auto ModelGet::prepareNrt() -> void
	{
		option() |= NOT_PRINT_CMD_INFO;
	}
	auto ModelGet::executeRT() -> int
	{
        std::cout<<"size of slave pool:"<<ecMaster()->slavePool().size()<<std::endl;
        float force[6]={0};
        auto get_force_data = [&](float* data)
        {
            for(std::size_t i = 0; i<6; ++i)
            {
                ecMaster()->slavePool()[12].readPdo(0x6020, 0x01 + i, data + i, 32);
            }
        };

        get_force_data(force);
        std::cout<<"force data:"<<std::endl;
        aris::dynamic::dsp(1,6,force);

		return 0;
	}
	ModelGet::ModelGet(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_get\"/>");
	}
    ModelGet::~ModelGet() = default;




	auto ModelInit::prepareNrt()->void {

		for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


	}
	auto ModelInit::executeRT()->int {


		double eePos[12] = { 0 };

		static double move = 0.0001;
		static double tolerance = 0.00009;

		static double init_pos[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

		modelBase()->setInputPos(init_pos);

        if (modelBase()->forwardDynamics())
		{
			throw std::runtime_error("Forward Kinematics Position Failed!");
		}


		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].targetPos();
		}


		auto motorsPositionCheck = [=]()
		{
			for(int i = 0; i < 12; i++)
			{
				if(std::fabs(current_angle[i]-init_pos[i])>=move)
				{
					return false;
				}
			}

			for (int i = 0; i < 12; i++)
			{
				controller()->motorPool()[i].setTargetPos(init_pos[i]);
			}

			return true;
		};



		for (int i = 0; i < 12; i++)
		{
			if (current_angle[i] <= init_pos[i] - move)
			{
				controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
			}
			else if (current_angle[i] >= init_pos[i] + move)
			{
				controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
			}
		}
	


	

		if (count() % 1000 == 0)
		{
			mout()<<current_angle[0]<<current_angle[1]<<current_angle[2]<<current_angle[3]<<current_angle[4]
			<<current_angle[5]<<current_angle[6]<<current_angle[7]<<current_angle[8]<<current_angle[9]
			<<current_angle[10]<<current_angle[11]<<std::endl;
		}


		if (motorsPositionCheck())
		{

			mout()<<"Back to Init Position"<<std::endl;
			modelBase()->setInputPos(current_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			
			mout()<<"current angle: \n"<<current_angle[0]<<current_angle[1]<<current_angle[2]<<current_angle[3]<<current_angle[4]
			<<current_angle[5]<<current_angle[6]<<current_angle[7]<<current_angle[8]<<current_angle[9]
			<<current_angle[10]<<current_angle[11]<<std::endl;

			modelBase()->getOutputPos(eePos);
			mout() << "current end position: \n" <<eePos[0]<<eePos[1]<<eePos[2]<<eePos[3]<<eePos[4]<<eePos[5]
			<<eePos[6]<<eePos[7]<<eePos[8]<<eePos[9]<<eePos[10]<<eePos[11]<<std::endl;


			return 0;
		}
		else
		{
			if(count()==28000)
			{
				mout()<<"Over Time"<<std::endl;
			}
			
			return 28000 - count();
		}
	}
	ModelInit::ModelInit(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_init\"/>");
	}
	ModelInit::~ModelInit() = default;

	auto ModelMoveX::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelMoveX::executeRT()->int
	{
		m_ = int32Param("model");
		d_ = doubleParam("distance");
		o_ = doubleParam("orientation");


		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
		static double init_pos[12]{};
		double input_angle[12]{};
		double ee_pos[12]{};
		double current_pos[12]{};
		double current_angle[12]{};

		if (count() == 1)
		{
			double begin_angle[12]{0};

			// double begin_angle[12]
			// { 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0,
			// 0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

			for (int i = 0; i < 12; i++)
			{
				begin_angle[i] = controller()->motorPool()[i].targetPos();
			}

			std::cout << "init angle:" << std::endl;
			aris::dynamic::dsp(1, 12, begin_angle);

			//this->master()->logFileRawName("move");
			std::cout << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			modelBase()->getOutputPos(init_pos);

			std::cout << "init position:" << std::endl;
			aris::dynamic::dsp(1, 12, init_pos);

		}


		auto eemove = [&](double* pos_) {
			modelBase()->setOutputPos(pos_);
			if (modelBase()->inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[12]{ 0 };


			modelBase()->getInputPos(x_joint);


			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(x_joint[i]);
			}

		};


		modelBase()->getOutputPos(ee_pos);



		if (m_ == 0)
		{
			ee_pos[0] += 0.00001;

		}
		else if (m_ == 1)
		{
			ee_pos[6] += 0.00001;
		}
		else
		{
			std::cout << "model out of range; 0 ---> arm1 (white); 1 ---> arm2 (blue)" << std::endl;
		}


		eemove(ee_pos);

		modelBase()->getInputPos(input_angle);


		if (count() % 100 == 0)
		{
			mout() << "arm1:" << input_angle[0] * 180 / PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
				<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI << "\n"
				<< "arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
				<< input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << std::endl;



		}


		//aris::dynamic::dsp(1,12,ee_pos);
		return (d_ * 100) - count();

	}
	ModelMoveX::ModelMoveX(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_x\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	<Param name=\"distance\" default=\"10.0\" abbreviation=\"d\"/>"
			"	<Param name=\"orientation\" default=\"1.0\" abbreviation=\"o\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ModelMoveX::~ModelMoveX() = default;





	

	auto ModelComP::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		


	}
	auto ModelComP::executeRT()->int
	{


		imp_->m_ = int32Param("model");



		static double move = 0.0001;

		////dual transform modelbase into multimodel
		//auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		////at(0) -> Arm1 -> white
		//auto& arm1 = dualArm.subModels().at(0);
		////at(1) -> Arm2 -> blue
		//auto& arm2 = dualArm.subModels().at(1);

		////transform to model
		//auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		//auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		////End Effector
		//auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		//auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		auto& arm2 = dualArm.subModels().at(0);

		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		GravComp gc;

		// Only One Arm Move Each Command
		auto jointMove = [&](double target_mp_[6], int m_)
		{
			double mp[12];
			for (std::size_t i = (0 + 6 * m_); i < (6 + 6 * m_); ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i - 6 * m_] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i - 6 * m_];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i - 6 * m_];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};



		auto getForceData = [&](double* data_, int m_)
		{

			int raw_force[6]{ 0 };
			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (m_ == 0)
			{
				data_[0] = -data_[0];
				data_[1] = -data_[1];

				data_[3] = -data_[3];
				data_[4] = -data_[4];
			}
			else if (m_ == 1)
			{
				data_[0] = -data_[0];
				data_[1] = -data_[1];

				data_[3] = -data_[3];
				data_[4] = -data_[4];

			}
			else
			{
				mout() << "Wrong Model" << std::endl;
			}


		};


		auto motorsPositionCheck = [](double current_sa_angle_[6], double target_pos_[6])
		{
			for (int i = 0; i < 6; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= move)
				{
					return false;
				}
			}

			return true;
		};

		auto caculateAvgForce = [=](double force_data_[6], double temp_force_[6], int count_)
		{
			if (count() < imp_->current_stop_time + imp_->stop_time)
			{

				if (count() % 500 == 0 && imp_->accumulation_count < 10)
				{
					double temp2[6]{ 1,2,3,4,5,6 };
					//getForceData(temp2, 0);

					for (int i = 0; i < 6; i++)
					{
						temp_force_[i] = temp_force_[i] + temp2[i];
					}


					imp_->accumulation_count = imp_->accumulation_count + 1;
					mout() << imp_->accumulation_count << std::endl;
				}

			}
			else if (count() == imp_->current_stop_time + imp_->stop_time)
			{
				mout() << "stop! " << "count(): " << count() << std::endl;
				imp_->accumulation_count = 0;
				for (int i = 0; i < 6; i++)
				{
					force_data_[i] = temp_force_[i] / 10.0;
				}
				mout() << "Force Data " << count_ << '\n' << force_data_[0] << '\t' << force_data_[1] << '\t' << force_data_[2] << '\t'
					<< force_data_[3] << '\t' << force_data_[4] << '\t' << force_data_[5] << std::endl;
			}
			else
			{
				mout() << "Flag Change " << count_ << std::endl;
				imp_->stop_flag = false;
			}
		};




		double current_angle[6] = { 0 };

		for (int i = 0; i < 6; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}


		if (imp_->stop_flag)
		{
			if (imp_->stop_count == 1)
			{

				caculateAvgForce(imp_->force_data_1, imp_->temp_force1, 1);

			}
			else if (imp_->stop_count == 2)
			{

				caculateAvgForce(imp_->force_data_2, imp_->temp_force2, 2);

			}
			else if (imp_->stop_count == 3)
			{

				caculateAvgForce(imp_->force_data_3, imp_->temp_force3, 3);

			}
			else
			{
				mout() << "Stop Count Wrong: " << imp_->stop_count << " stop flag: " << imp_->stop_flag << std::endl;
				return 0;
			}

			return 80000 - count();
		}
		else
		{

			// arm 2 blue arm
			
				// 	Init Pos
				static double angle2_1[6]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

				static double angle2_2[6]{ 0, 0, -5 * PI / 6, PI / 2, PI / 2, PI / 2 };

				static double angle2_3[6]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, PI / 2 };

				double current_sa_angle[6]{ 0 };

				std::copy(current_angle , current_angle + 6, current_sa_angle);

				//if (count() % 1000 == 0)
				//{
				//	mout() << current_sa_angle[0] << '\t' << current_sa_angle[1] << '\t' << current_sa_angle[2] << '\t'
				//		<< current_sa_angle[3] << '\t' << current_sa_angle[4] << '\t' << current_sa_angle[5] << std::endl;

				//}


				if (!imp_->target1_reached)
				{
					model_a2.setInputPos(angle2_1);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

					jointMove(angle2_1, 0);

					if (motorsPositionCheck(current_sa_angle, angle2_1))
					{
						mout() << "Target 1 Reached" << std::endl;


						eeA2.getMpm(imp_->ee_pm_1);

						imp_->target1_reached = true;
						imp_->stop_count = 1;
						imp_->current_stop_time = count();
						imp_->stop_flag = true;
						mout() << "current stop time: " << imp_->current_stop_time << std::endl;
					}

				}
				else if (imp_->target1_reached && !imp_->target2_reached)
				{
					model_a2.setInputPos(angle2_2);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

					jointMove(angle2_2, 0);

					if (motorsPositionCheck(current_sa_angle, angle2_2))
					{
						mout() << "Target 2 Reached" << std::endl;

						eeA2.getMpm(imp_->ee_pm_2);

						imp_->target2_reached = true;
						imp_->stop_count = 2;
						imp_->current_stop_time = count();
						imp_->stop_flag = true;
						mout() << "current stop time: " << imp_->current_stop_time << std::endl;
					}
				}
				else if (imp_->target2_reached && !imp_->target3_reached)
				{
					model_a2.setInputPos(angle2_3);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

					jointMove(angle2_3, 0);

					if (motorsPositionCheck(current_sa_angle, angle2_3))
					{
						mout() << "Target 3 Reached" << std::endl;

						eeA2.getMpm(imp_->ee_pm_3);

						imp_->target3_reached = true;
						imp_->stop_count = 3;
						imp_->current_stop_time = count();
						imp_->stop_flag = true;
						mout() << "current stop time: " << imp_->current_stop_time << std::endl;

					}
				}
				else if (imp_->target3_reached && !imp_->target4_reached)
				{
					// Back To Init
					model_a2.setInputPos(angle2_1);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

					jointMove(angle2_1, 0);

					if (motorsPositionCheck(current_sa_angle, angle2_1))
					{
						mout() << "Back To Init Pos" << std::endl;
						mout() << "Current Angle: " << current_sa_angle[0] << '\t' << current_sa_angle[1] << '\t' << current_sa_angle[2] << '\t'
							<< current_sa_angle[3] << '\t' << current_sa_angle[4] << '\t' << current_sa_angle[5] << std::endl;

						imp_->target4_reached = true;

					}

				}
				else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached && !imp_->force_test_begin)
				{
					double t_vector[9]{ 0 };
					double f_vector[9]{ 0 };

					double f_matrix[54]{ 0 };
					double r_matrix[54]{ 0 };


					double ee_rm_1[9]{ 0 };
					double ee_rm_2[9]{ 0 };
					double ee_rm_3[9]{ 0 };

					double current_force[6]{ 0 };

					aris::dynamic::s_pm2rm(imp_->ee_pm_1, ee_rm_1);
					aris::dynamic::s_pm2rm(imp_->ee_pm_2, ee_rm_2);
					aris::dynamic::s_pm2rm(imp_->ee_pm_3, ee_rm_3);

					

					gc.getTorqueVector(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, t_vector);
					gc.getForceVector(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, f_vector);

					gc.getFMatrix(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, f_matrix);
					gc.getRMatrix(ee_rm_1, ee_rm_2, ee_rm_3, r_matrix);

					gc.getPLMatrix(f_matrix, t_vector, imp_->p_vector);
					gc.getPLMatrix(r_matrix, f_vector, imp_->l_vector);

					double current_ee_pm[16]{ 0 };
					eeA2.getMpm(current_ee_pm);

					gc.getCompFT(current_ee_pm, imp_->l_vector, imp_->p_vector, imp_->comp_f);
					//getForceData(current_force, 0);

					mout() << "Current Force After Compensation:" << '\n' << current_force[0] + imp_->comp_f[0] << '\t' << current_force[1] + imp_->comp_f[1] << '\t'
						<< current_force[2] + imp_->comp_f[2] << '\t' << current_force[3] + imp_->comp_f[3] << '\t'
						<< current_force[4] + imp_->comp_f[4] << '\t' << current_force[5] + imp_->comp_f[5] << std::endl;

					mout() << "Current End Pos:" << '\n' << current_ee_pm[0] << '\t' << current_ee_pm[1] << '\t'
						<< current_ee_pm[2] << '\t' << current_ee_pm[3] << '\t'
						<< current_ee_pm[4] << '\t' << current_ee_pm[5] << std::endl;

					imp_->force_test_begin = true;
					mout() << "Force Test Start" << std::endl;

				}
				else if (imp_->force_test_begin)
				{
					double ee_pm[16]{ 0 };
					eeA2.getMpm(ee_pm);

					gc.getCompFT(ee_pm, imp_->l_vector, imp_->p_vector, imp_->comp_f);

					double current_force[6]{ 0 };
					//getForceData(current_force, 0);

					if (count() % 100 == 0)
					{
						mout() << "Current Force After Compensation:" << '\n' << current_force[0] + imp_->comp_f[0] << '\t' << current_force[1] + imp_->comp_f[1] << '\t'
							<< current_force[2] + imp_->comp_f[2] << '\t' << current_force[3] + imp_->comp_f[3] << '\t'
							<< current_force[4] + imp_->comp_f[4] << '\t' << current_force[5] + imp_->comp_f[5] << std::endl;
					}
				}

				if (count() == 100000)
				{

					mout() << "Over Time" << std::endl;

					mout() << "P Vector: " << '\n' << imp_->p_vector[0] << '\t' << imp_->p_vector[1] << '\t' << imp_->p_vector[2] << '\t' << imp_->p_vector[3] << '\t'
						<< imp_->p_vector[4] << '\t' << imp_->p_vector[5] << '\t' << imp_->p_vector[6] << std::endl;

					mout() << "L Vector: " << '\n' << imp_->l_vector[0] << '\t' << imp_->l_vector[1] << '\t' << imp_->l_vector[2] << '\t' << imp_->l_vector[3] << '\t'
						<< imp_->l_vector[4] << '\t' << imp_->l_vector[5] << '\t' << imp_->l_vector[6] << std::endl;

						
				}

				return 100000 - count();

		}












	}
	ModelComP::ModelComP(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_comp\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ModelComP::~ModelComP() = default;


	auto ForceAlign::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;




	}
	auto ForceAlign::executeRT()->int
	{

		static double offset = 0.0001;
		static double init_angle[6] = { 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		////dual transform modelbase into multimodel
		//auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		////at(0) -> Arm1 -> white
		//auto& arm1 = dualArm.subModels().at(0);
		////at(1) -> Arm2 -> blue
		//auto& arm2 = dualArm.subModels().at(1);

		////transform to model
		//auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		//auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		////End Effector
		//auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		//auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		auto& arm2 = dualArm.subModels().at(0);

		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		GravComp gc;



		double current_vel[6]{ 0 };
		double current_pos[6]{ 0 };

		double current_force[6]{ 0 };
		double current_angle[6] = { 0 };
		double current_pm[16]{ 0 };

		double comp_force[6]{ 0 };
		double actual_force[6]{ 0 };

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[8 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
				mout() << imp_->init_force[0] << '\t' << imp_->init_force[1] << '\t' << imp_->init_force[2] << '\t'
					<< imp_->init_force[3] << '\t' << imp_->init_force[4] << '\t' << imp_->init_force[5] << std::endl;
			}
			else
			{
				for (std::size_t i = 0; i < 6; ++i)
				{

					data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->init_force[i];

				}
			}

			if (m_ == 0)
			{
				//data_[0] = -data_[0];
				//data_[1] = -data_[1];

				//data_[3] = -data_[3];
				//data_[4] = -data_[4];
			}
			else if (m_ == 1)
			{
				data_[0] = -data_[0];
				data_[1] = -data_[1];

				data_[3] = -data_[3];
				data_[4] = -data_[4];

			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}


		};


		// Only One Arm Move Each Command
		auto jointMove = [&](double target_mp_[6], int m_)
		{
			double mp[12];
			for (std::size_t i = (0 + 6 * m_); i < (6 + 6 * m_); ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i - 6 * m_] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i - 6 * m_];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i - 6 * m_];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};

		auto motorsPositionCheck = [](double current_sa_angle_[6], double target_pos_[6])
		{
			for (int i = 0; i < 6; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= offset)
				{
					return false;
				}
			}

			return true;
		};
		//single arm move 1-->white 2-->blue
		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

			model_.setOutputPos(pos_);

			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[6]{ 0 };

			model_.getInputPos(x_joint);

			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};


		//Get Current Angle
		for (int i = 0; i < 6; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}

		if (count() % 1000 == 0)
		{
			mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
				<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;
		}

		model_a2.setInputPos(current_angle);
		if (model_a2.forwardKinematics())
		{
			throw std::runtime_error("Forward Kinematics Position Failed!");
		}



		eeA2.getV(current_vel);
		eeA2.getP(current_pos);
		eeA2.getMpm(current_pm);

		if (!imp_->init)
		{
			model_a2.setInputPos(init_angle);
			if (model_a2.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}

			jointMove(init_angle, 0);
			if (motorsPositionCheck(current_angle, init_angle))
			{
				getForceData(imp_->init_force, 0, imp_->init);
				mout() << "Back To Init" << std::endl;

				imp_->init = true;
			}
		}
		else
		{
			//Contact Check

			//Get Actual Force
			getForceData(current_force, 0, imp_->init);
			gc.getCompFT(current_pm, imp_->l_vector, imp_->p_vector, comp_force);
			for (int i = 0; i < 6; i++)
			{
				actual_force[i] = comp_force[i] + current_force[i] - imp_->init_force[i];
			}

			if (!imp_->contact_check)
			{
				if (abs(actual_force[2]) > 2)
				{
					imp_->contact_check = true;
					imp_->contact_count = count();
					// Set Disred Pos 
					imp_->x_d = current_pos[0];
					mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
				}
				else
				{

					current_pos[0] -= 0.00002;
					saMove(current_pos, model_a2, 0);

				}

			}
			else if (imp_->contact_check)
			{
				double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
				double a = (imp_->desired_force - actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
				double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

				current_pos[0] = x;
				saMove(current_pos, model_a2, 0);

				if (count() % 100 == 0)
				{
					//mout() << "count(): " << count() << std::endl;
					mout() << "force: " << actual_force[0] << '\t' << actual_force[1] << '\t' << actual_force[2] << '\t'
						<< actual_force[3] << '\t' << actual_force[4] << '\t' << actual_force[5] << std::endl;
				}

			}
		}
		//Over Time Exit
		if (count() == 80000)
		{
			mout() << "Over Time" << std::endl;
		}

		return 80000 - count();

	}
	ForceAlign::ForceAlign(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_fa\"/>");
	}
	ForceAlign::~ForceAlign() = default;





	ARIS_REGISTRATION{
		aris::core::class_<ModelInit>("ModelInit")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelForward>("ModelForward")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelGet>("ModelGet")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelMoveX>("ModelMoveX")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelSetPos>("ModelSetPos")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelComP>("ModelComP")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelTest>("ModelTest")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ForceAlign>("ForceAlign")
			.inherit<aris::plan::Plan>();

	}





}





