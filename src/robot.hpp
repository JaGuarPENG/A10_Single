#ifndef PLAN_H
#define PLAN_H

#include <aris.hpp>

namespace robot
{



	class ModelSetPos :public aris::core::CloneObject<ModelSetPos, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelSetPos();
		explicit ModelSetPos(const std::string& name = "ModelSetPos");

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct ModelSetPos::Imp {

			double p_vector[6]{ 0 };
			double l_vector[6]{ 0 };
		};
	};



	class ModelForward :public aris::core::CloneObject<ModelForward, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelForward();
		explicit ModelForward(const std::string& name = "ModelForward");
	private:
	};



	class ModelGet : public aris::core::CloneObject<ModelGet, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelGet();
		explicit ModelGet(const std::string& name = "ModelGet");
		
	};


	class ModelInit : public aris::core::CloneObject<ModelInit, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelInit();
		explicit ModelInit(const std::string& name = "ModelInit");
    private:
	};




	class ModelTest :public aris::core::CloneObject<ModelTest, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelTest();
		explicit ModelTest(const std::string& name = "ModelTest");
	private:
		double cef_;
	};


	class ModelMoveX :public aris::core::CloneObject<ModelMoveX, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelMoveX();
		explicit ModelMoveX(const std::string& name = "ModelMoveX");

	private:
		int m_;
		double d_;
		double o_;
	};


class ModelComP :public aris::core::CloneObject<ModelComP, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelComP();
		explicit ModelComP(const std::string& name = "ModelComP");

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct ModelComP::Imp {
			bool target1_reached = false;
			bool target2_reached = false;
			bool target3_reached = false;
			bool target4_reached = false;

			bool force_test_begin = false;

			bool stop_flag = false;
			int stop_count = 0;
			int stop_time = 5500;
			int current_stop_time = 0;

			int accumulation_count = 0;

			//temp data to stroage 10 times of actual force
			double temp_force1[6] = { 0 };
			double temp_force2[6] = { 0 };
			double temp_force3[6] = { 0 };

			double force_data_1[6] = { 0 };
			double force_data_2[6] = { 0 };
			double force_data_3[6] = { 0 };

			double ee_pm_1[16]{ 0 };
			double ee_pm_2[16]{ 0 };
			double ee_pm_3[16]{ 0 };

			double comp_f[6]{ 0 };

			double p_vector[6]{ 0 };
			double l_vector[6]{ 0 };

			int m_;
		};

	};

class ForceAlign :public aris::core::CloneObject<ForceAlign, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceAlign();
		explicit ForceAlign(const std::string& name = "ForceAlign");

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct ForceAlign::Imp {

			bool init = false;
			bool contact_check = false;

			double comp_f[6]{ 0 };
			double init_force[6]{ 0 };

			double p_vector[6]{ 0 };
			double l_vector[6]{ 0 };

			double x_d;
			double Ke = 220000;
			double K = 3;
			
			double B[6]{ 0.25,0.25,0.7,0.0,0.0,0.0 };
			double M[6]{ 0.1,0.1,0.1,0.1,0.1,0.1 };

			double desired_force = 5;
			int contact_count = 0;

		};

};


}



#endif
