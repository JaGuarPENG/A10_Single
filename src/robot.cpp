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
		{ 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0 };

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


		double eepA1[6] = { 0 };
		double eepBa[12] = { 0 };

		model_a1.setInputPos(input_angle1);
		if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
		model_a1.getOutputPos(eepA1);

		std::cout << "Arm1" << std::endl;
		aris::dynamic::dsp(1, 6, eepA1);
		
		dualArm.setInputPos(input_angle);
		if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;
		dualArm.getOutputPos(eepBa);

		std::cout << "dual" << std::endl;
		aris::dynamic::dsp(1, 12, eepBa);

		double Arm1_angle[6]{ 0 };
		double Dual_angle[12]{ 0 };


		model_a1.setOutputPos(eepA1);
		if (model_a1.inverseKinematics())std::cout << "inverse failed" << std::endl;
		model_a1.getInputPos(Arm1_angle);

		std::cout << "inverse Arm1" << std::endl;
		aris::dynamic::dsp(1, 6, Arm1_angle);


		dualArm.setOutputPos(eepBa);
		if (dualArm.inverseKinematics())std::cout << "inverse failed" << std::endl;
		dualArm.getInputPos(Dual_angle);
		
		std::cout << "inverse dual" << std::endl;
		aris::dynamic::dsp(1, 12, Dual_angle);


		//position
		double a1_stcp[6]{};
		double a2_stcp[12]{};
		//velocity
		double a1_vtcp[6]{};
		double a2_vtcp[12]{};

		//get ee
		auto& eeA1 = model_a1.generalMotionPool().at(0);
		auto& eeA2 = model_a2.generalMotionPool().at(0);

		eeA1.getP(a1_stcp);
		std::cout << "Arm1 ee pos" << std::endl;
		aris::dynamic::dsp(1, 6, a1_stcp);

		std::cout << "Arm1 ee vel" << std::endl;
		eeA1.getV(a1_vtcp);
		aris::dynamic::dsp(1, 6, a1_vtcp);

		eeA1.updP();

		//both arm move
		auto baMove = [&](double* pos_) {

			dualArm.setOutputPos(pos_);

			if (dualArm.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[12]{ 0 };

			dualArm.getInputPos(x_joint);


			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(x_joint[i]);
			}

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



		double posTest[6]{ 0.402373, 0.124673, 0.136492, 6.283185, 0.000000, 3.141593 };
		saMove(posTest, model_a1, 1);

		double test[12]{};
		dualArm.getOutputPos(test);

		std::cout << "test" << std::endl;
		aris::dynamic::dsp(1, 12, test);





		////example1 end/////
	


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
		static double initPos[12]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -90, 0,
							0, 0, -5 * PI / 6, 5 * PI / 6, 90, 0 };

		modelBase()->setInputPos(initPos);
		if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;

		double eePos[12]{ 0 };

		modelBase()->getOutputPos(eePos);
		std::cout << "init" << std::endl;
		aris::dynamic::dsp(1, 12, eePos);

		double finaleePos[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			finaleePos[i] = eePos[i];
		}

		finaleePos[0] = eePos[0] + 0.5;
		std::cout << "final" << std::endl;
		aris::dynamic::dsp(1, 12, finaleePos);



		modelBase()->setOutputPos(finaleePos);
		if (modelBase()->inverseKinematics())std::cout << "inverse failed" << std::endl;
		double finalpos[12] = { 0 };
		modelBase()->getInputPos(finalpos);

		aris::dynamic::dsp(1, 12, finalpos);

		double finalangle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			finalangle[i] = finalpos[i] * 180 / PI;
		}

		std::cout << "final angle" << std::endl;

		aris::dynamic::dsp(1, 12, finalangle);

		
		

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





	struct ModelComP::Imp {
		bool target1_reached = false;
		bool target2_reached = false;
		bool target3_reached = false;

		int m_;
	};

	auto ModelComP::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		


	}
	auto ModelComP::executeRT()->int
	{


		imp_ -> m_ = int32Param("model");
		
		

		static double move = 0.0003;

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


		// Only One Arm Move Each command
		auto jointMove = [&](double target_mp_[6]){
        	double mp[6];
        	double max_speed = 0.0001;
			for (std::size_t i =0; i< 6;++i)
			{
				//start back to zero"
				if(controller()->motorPool()[i].actualPos() - target_mp_[i] < 8/180*PI){
					if(controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if(controller()->motorPool()[i].targetPos() <=target_mp_[i] -0.0001){

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else{
						mp[i] = target_mp_[i];

					}
				}else{
					if(controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if(controller()->motorPool()[i].targetPos() <=target_mp_[i] - 0.0001){

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else{
						mp[i] = target_mp_[i];

					}

				}
				aris::dynamic::dsp(1, 6, mp);
            	controller()->motorPool()[i].setTargetPos(mp[i]);
        	}
    	};


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

		auto cal_average = [&](double *data, int size) -> double {
			double sum = 0.0;  // 初始化和为0

			// 遍历数组，计算和
			for (int i = 0; i < size; ++i) {
				sum += data[i];
			}

			// 返回平均值
			return sum / size;
		};

		auto get_force_data = [&](double* data_)
		{

			int raw_force[6]{ 0 };
			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[18].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

		};


		auto motorsPositionCheck = [](double current_sa_angle_[6], double target_pos_[6])
		{
			for (int i = 0; i < 12; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= move)
				{
					return false;
				}
			}

			return true;
		};


		static double init_pos[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].targetPos();
		}
	




		// arm 1 white arm
		if (imp_->m_ == 0)
		{
			//static double angle1_1[6]{0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0};
		
			//static double angle1_2[6]{0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0};

			//static double angle1_3[6]{0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0};

			//test
			static double angle1_1[12]{ 0, 0, -0.0005, 0.0005, 0.001, 0.001 };

			static double angle1_2[12]{ 0, 0, -0.0005, 0.0015, 0.0015, 0.002 };

			static double angle1_3[12]{ 0, 0, -0.0005, 0.0015, 0.0032, 0.004 };


			double current_sa_angle[6]{ 0 };
			std::copy(current_angle, current_angle + 6, current_sa_angle);

			if (count() % 1000 == 0)
			{
				aris::dynamic::dsp(1, 6, current_sa_angle);

			}


			

			if(!imp_->target1_reached)
			{
				model_a1.setInputPos(angle1_1);
				if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
				
				jointMove(angle1_1);

				if(motorsPositionCheck(current_sa_angle, angle1_1))
				{
					mout() << "Target 1 Reached" << std::endl;
					imp_->target1_reached = true;
				}

			}
			else if(imp_->target1_reached && !imp_->target2_reached)
			{
				model_a1.setInputPos(angle1_2);
				if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
				
				jointMove(angle1_2);

				if(motorsPositionCheck(current_sa_angle, angle1_2))
				{
					mout() << "Target 2 Reached" << std::endl;
					imp_->target2_reached = true;
				}
			}
			else if(imp_->target2_reached && !imp_->target3_reached)
			{
				model_a1.setInputPos(angle1_3);
				if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
				
				jointMove(angle1_3);

				if(motorsPositionCheck(current_sa_angle, angle1_3))
				{
					mout() << "Target 3 Reached" << std::endl;
					imp_->target3_reached = true;
					return 0;
				}
			}
			
			if (count() == 50000)
			{
				aris::dynamic::dsp(1, 12, current_angle);

				mout() << "Over Time" << std::endl;
			}

			return 10 - count();
			


		}

		// arm 2 blue arm
		else if(imp_->m_ == 1)
		{
			// 	Init Pos
			static double pos2_1[12]{0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2};
		
			static double pos2_2[12]{0, 0, -5 * PI / 6, PI / 2, PI / 2, PI / 2};

			static double pos2_3[12]{0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, PI / 2};

			
			

			return 0;
			




		}
		// wrong model
		else
		{
			mout()<<"Wrong Model"<<std::endl;
			throw std::runtime_error("Arm Type Error");
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

	}





}





