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

	};




}



#endif
