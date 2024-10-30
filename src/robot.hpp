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
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct ModelTest::Imp {

			//Flag
			bool init = false;
			bool contact_check = false;

			//Force Compensation Parameter
			double comp_f[6]{ 0 };

			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };

			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };

			//Current Vel
			double v_c[6]{ 0 };

			//Impedence Parameter
			double a1_K[6]{ 100,100,100,5,5,5 };
			double a1_B[6]{ 100,100,100,5,5,5 };
			double a1_M[6]{ 1,1,1,2,2,2 };

			double a2_K[6]{ 100,100,100,15,15,15 };
			double a2_B[6]{ 100,100,100,15,15,15 };
			double a2_M[6]{ 1,1,1,10,10,10 };

			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

			//Counter
			int contact_count = 0;

			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };

			//Test
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;
		};

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

			bool init = false;

			bool stop_flag = false;
			int stop_count = 0;
			int stop_time = 2200;
			int current_stop_time = 0;

			int accumulation_count = 0;

			//temp data to stroage 10 times of actual force
			

			// For Arm 1
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			double arm1_temp_force_1[6] = { 0 };
			double arm1_temp_force_2[6] = { 0 };
			double arm1_temp_force_3[6] = { 0 };

			double arm1_force_data_1[6] = { 0 };
			double arm1_force_data_2[6] = { 0 };
			double arm1_force_data_3[6] = { 0 };

			double arm1_ee_pm_1[16]{ 0 };
			double arm1_ee_pm_2[16]{ 0 };
			double arm1_ee_pm_3[16]{ 0 };

			double arm1_comp_f[6]{ 0 };
			double arm1_init_force[6]{ 0 };


			//For Arm 2
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			double arm2_temp_force_1[6] = { 0 };
			double arm2_temp_force_2[6] = { 0 };
			double arm2_temp_force_3[6] = { 0 };

			double arm2_force_data_1[6] = { 0 };
			double arm2_force_data_2[6] = { 0 };
			double arm2_force_data_3[6] = { 0 };

			double arm2_ee_pm_1[16]{ 0 };
			double arm2_ee_pm_2[16]{ 0 };
			double arm2_ee_pm_3[16]{ 0 };

			double arm2_comp_f[6]{ 0 };
			double arm2_init_force[6]{ 0 };

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

			double arm1_init_force[6]{ 0 };
			double arm2_init_force[6]{ 0 };

			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			double x_d;

			double Ke = 220000;
			double K = 3;

			double B[6]{ 0.25,0.25,0.7,0.0,0.0,0.0 };
			double M[6]{ 0.1,0.1,0.1,0.1,0.1,0.1 };

			double desired_force = -5;
			int contact_count;

			//Simulation
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;

		};

	};




	class ForceKeep :public aris::core::CloneObject<ForceKeep, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceKeep();
		explicit ForceKeep(const std::string& name = "ForceKeep");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct ForceKeep::Imp {

			//Flag
			bool init = false;
			bool contact_check = false;

			//Force Compensation Parameter
			double comp_f[6]{ 0 };

			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };

			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };

			//Current Vel
			double v_c[6]{ 0 };

			//Impedence Parameter
			double a1_K[6]{ 100,100,100,5,5,5 };
			double a1_B[6]{ 100,100,100,5,5,5 };
			double a1_M[6]{ 1,1,1,2,2,2 };

			double a2_K[6]{ 100,100,100,15,15,15 };
			double a2_B[6]{ 100,100,100,15,15,15 };
			double a2_M[6]{ 1,1,1,10,10,10 };

			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

			//Counter
			int contact_count = 0;

			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };

			//Test
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;
		};

	};


	class ForceDrag :public aris::core::CloneObject<ForceDrag, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceDrag();
		explicit ForceDrag(const std::string& name = "ForceDrag");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct ForceDrag::Imp {

			//Flag
			bool init = false;
			bool contact_check = false;

			//Force Compensation Parameter
			double comp_f[6]{ 0 };

			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };

			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };

			//Current Vel
			double v_c[6]{ 0 };

			//Impedence Parameter
			double K[6]{ 100,100,100,15,15,15 };
			double B[6]{ 100,100,100,15,15,15 };
			double M[6]{ 1,1,1,10,10,10 };

			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

			//Counter
			int contact_count = 0;

			//Test
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;

			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };
		};

	};

	class PegInHole :public aris::core::CloneObject<PegInHole, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegInHole();
		explicit PegInHole(const std::string& name = "PegInHole");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		struct PegInHole::Imp {

			//Flag
			bool init = false;
			bool stop = false;
			bool phase1 = false;
			bool phase2 = false;
			bool phase3 = false;
			bool phase4 = false;

			//Force Compensation Parameter
			double comp_f[6]{ 0 };

			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };

			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };

			//Current Vel
			double v_c[6]{ 0 };

			//Impedence Parameter
			double K[6]{ 100,100,100,15,15,15 };
			double B[6]{ 100,100,100,15,15,15 };
			double M[6]{ 1,1,1,10,10,10 };

			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

			//Counter
			int contact_count = 0;
			int current_count = 0;

			//Test
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;

			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };
		};

	};

}



#endif
