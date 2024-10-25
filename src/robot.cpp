#include "robot.hpp"
#include "plan.hpp"
#include "gravcomp.hpp"

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



		///gravity compensation test///
		GravComp gc;

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

		double current_pose_a1[16] = { -1, 6.63139e-15, -4.70536e-15, 0.32,
										1.33547e-15, 0.707107, 0.707107, 0.0516188,
										8.30063e-15, 0.707107, -0.707107, 0.590381,
										0, 0, 0, 1 };
		double test_force1[6]{ -0.0976562, -0.0976562, 0.595703, 0.00546875, 0.00234375, 0.00117188 };

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
		double comp_f[6]{ 0 };

		// aris::dynamic::s_pm2rm(ee_pm_1,ee_rm_1);
		// aris::dynamic::s_pm2rm(ee_pm_2,ee_rm_2);
		// aris::dynamic::s_pm2rm(ee_pm_3,ee_rm_3);

		gc.getTorqueVector(force_data1, force_data2, force_data3, t_vector);
		gc.getForceVector(force_data1, force_data2, force_data3, f_vector);

		gc.getFMatrix(force_data1, force_data2, force_data3, f_matrix);
		gc.getRMatrix(pose1, pose2, pose3, r_matrix);

		gc.getPLMatrix(f_matrix, t_vector, p_vector);
		gc.getPLMatrix(r_matrix, f_vector, l_vector);

		gc.getCompFT(current_pose_a1, l_vector, p_vector, comp_f);
		//getForceData(current_force);

		for (int i = 0; i < 6; i++)
		{
			current_force[i] = test_force1[i] + comp_f[i];
		}

		aris::dynamic::dsp(1, 6, current_force);


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
			mout() << "Arm1:" << input_angle[0] * 180 / PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
				<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI << "\n"
				<< "Arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
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
		GravComp gc;

		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		static double angle1[12] =
		{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

		static double angle2[12] =
		{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 3, 0 ,
		0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

		static double angle3[12] =
		{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 ,
		0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };

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

		double arm1_ee_pm_1[16]{ 0 };
		double arm1_ee_pm_2[16]{ 0 };
		double arm1_ee_pm_3[16]{ 0 };
		double arm1_current_ee_pm[16]{ 0 };


		double arm2_ee_pm_1[16]{ 0 };
		double arm2_ee_pm_2[16]{ 0 };
		double arm2_ee_pm_3[16]{ 0 };
		double arm2_current_ee_pm[16]{ 0 };

		dualArm.setInputPos(angle1);
		if (dualArm.forwardKinematics())
		{
			mout() << "Error" << std::endl;
		}

		eeA1.getMpm(arm1_ee_pm_1);
		eeA2.getMpm(arm2_ee_pm_1);

		dualArm.setInputPos(angle2);
		if (dualArm.forwardKinematics())
		{
			mout() << "Error" << std::endl;
		}

		eeA1.getMpm(arm1_ee_pm_2);
		eeA2.getMpm(arm2_ee_pm_2);

		dualArm.setInputPos(angle3);
		if (dualArm.forwardKinematics())
		{
			mout() << "Error" << std::endl;
		}

		eeA1.getMpm(arm1_ee_pm_3);
		eeA2.getMpm(arm2_ee_pm_3);

		dualArm.setInputPos(init_angle);
		if (dualArm.forwardKinematics())
		{
			mout() << "Error" << std::endl;
		}

		eeA1.getMpm(arm1_current_ee_pm);
		eeA2.getMpm(arm2_current_ee_pm);





		//Arm 1
		

		double arm1_force_data_1[6]{ -0.204, - 0.0033, - 1.2455, - 0.0001,	0.0016, - 0.000};
		double arm1_force_data_2[6]{ -1.7928, - 1.5529,	2.6597, - 0.0075,	0.0067, - 0.0007 };
		double arm1_force_data_3[6]{ -0.5153,	0.9186,	1.5177,	0.0016,	0.0033,	0.0018 };




		double arm1_t_vector[9]{ 0 };
		double arm1_f_vector[9]{ 0 };

		double arm1_f_matrix[54]{ 0 };	
		double arm1_r_matrix[54]{ 0 };

		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };


		double arm1_ee_rm_1[9]{ 0 };
		double arm1_ee_rm_2[9]{ 0 };
		double arm1_ee_rm_3[9]{ 0 };

		double arm1_current_force[6]{ -0.003,	0.018, - 0.018, - 0.001,	0,	0.001 };
		double arm1_comp_f[6]{ 0 };

		aris::dynamic::s_pm2rm(arm1_ee_pm_1, arm1_ee_rm_1);
		aris::dynamic::s_pm2rm(arm1_ee_pm_2, arm1_ee_rm_2);
		aris::dynamic::s_pm2rm(arm1_ee_pm_3, arm1_ee_rm_3);



		gc.getTorqueVector(arm1_force_data_1, arm1_force_data_2, arm1_force_data_3, arm1_t_vector);
		gc.getForceVector(arm1_force_data_1, arm1_force_data_2, arm1_force_data_3, arm1_f_vector);

		gc.getFMatrix(arm1_force_data_1, arm1_force_data_2, arm1_force_data_3, arm1_f_matrix);
		gc.getRMatrix(arm1_ee_rm_1, arm1_ee_rm_2, arm1_ee_rm_3, arm1_r_matrix);

		gc.getPLMatrix(arm1_f_matrix, arm1_t_vector, arm1_p_vector);
		gc.getPLMatrix(arm1_r_matrix, arm1_f_vector, arm1_l_vector);

		

		gc.getCompFT(arm1_current_ee_pm, arm1_l_vector, arm1_p_vector, arm1_comp_f);


		//getForceData(arm1_current_force, 0, imp_->init);
		mout() << "Current Arm1 Force After Compensation:" << '\n' << arm1_current_force[0] + arm1_comp_f[0] << '\t' << arm1_current_force[1] + arm1_comp_f[1] << '\t'
			<< arm1_current_force[2] + arm1_comp_f[2] << '\t' << arm1_current_force[3] + arm1_comp_f[3] << '\t'
			<< arm1_current_force[4] + arm1_comp_f[4] << '\t' << arm1_current_force[5] + arm1_comp_f[5] << std::endl;


		//Arm 2
		

		double arm2_force_data_1[6]{ 0.3995, - 0.0099, - 2.3944,	0.0002,	0.0057,	0.0002 };
		double arm2_force_data_2[6]{ 3.5219, - 3.0621,	5.1777,	0.0329,	0.045, - 0.0019 };
		double arm2_force_data_3[6]{ 0.9812,	1.753,	2.9958, - 0.0238,	0.0139, - 0.0002 };




		double arm2_t_vector[9]{ 0 };
		double arm2_f_vector[9]{ 0 };

		double arm2_f_matrix[54]{ 0 };
		double arm2_r_matrix[54]{ 0 };

		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };


		double arm2_ee_rm_1[9]{ 0 };
		double arm2_ee_rm_2[9]{ 0 };
		double arm2_ee_rm_3[9]{ 0 };

		double arm2_current_force[6]{ 0.009, - 0.01,	0.028, - 0.002,	0.002,	0};
		double arm2_comp_f[6]{ 0 };

		aris::dynamic::s_pm2rm(arm2_ee_pm_1, arm2_ee_rm_1);
		aris::dynamic::s_pm2rm(arm2_ee_pm_2, arm2_ee_rm_2);
		aris::dynamic::s_pm2rm(arm2_ee_pm_3, arm2_ee_rm_3);



		gc.getTorqueVector(arm2_force_data_1, arm2_force_data_2, arm2_force_data_3, arm2_t_vector);
		gc.getForceVector(arm2_force_data_1, arm2_force_data_2, arm2_force_data_3, arm2_f_vector);

		gc.getFMatrix(arm2_force_data_1, arm2_force_data_2, arm2_force_data_3, arm2_f_matrix);
		gc.getRMatrix(arm2_ee_rm_1, arm2_ee_rm_2, arm2_ee_rm_3, arm2_r_matrix);

		gc.getPLMatrix(arm2_f_matrix, arm2_t_vector, arm2_p_vector);
		gc.getPLMatrix(arm2_r_matrix, arm2_f_vector, arm2_l_vector);

		

		gc.getCompFT(arm2_current_ee_pm, arm2_l_vector, arm2_p_vector, arm2_comp_f);

		//getForceData(arm2_current_force, 0, imp_->init);

		gc.savePLVector(arm1_p_vector, arm2_p_vector, arm1_l_vector, arm2_l_vector);


		

		//mout() << "Current Arm1 End Pos:" << '\n' << arm1_current_ee_pm[0] << '\t' << arm1_current_ee_pm[1] << '\t'
		//	<< arm1_current_ee_pm[2] << '\t' << arm1_current_ee_pm[3] << '\t'
		//	<< arm1_current_ee_pm[4] << '\t' << arm1_current_ee_pm[5] << std::endl;

		mout() << "Current Arm2 Force After Compensation:" << '\n' << arm2_current_force[0] + arm2_comp_f[0] << '\t' << arm2_current_force[1] + arm2_comp_f[1] << '\t'
			<< arm2_current_force[2] + arm2_comp_f[2] << '\t' << arm2_current_force[3] + arm2_comp_f[3] << '\t'
			<< arm2_current_force[4] + arm2_comp_f[4] << '\t' << arm2_current_force[5] + arm2_comp_f[5] << std::endl;

		//mout() << "Current Arm2 End Pos:" << '\n' << arm2_current_ee_pm[0] << '\t' << arm2_current_ee_pm[1] << '\t'
		//	<< arm2_current_ee_pm[2] << '\t' << arm2_current_ee_pm[3] << '\t'
		//	<< arm2_current_ee_pm[4] << '\t' << arm2_current_ee_pm[5] << std::endl;



		return 0;
	}
	ModelTest::ModelTest(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_t\"/>");
	}
	ModelTest::~ModelTest() = default;




	auto ModelGet::prepareNrt() -> void
	{
		option() |= NOT_PRINT_CMD_INFO;
	}
	auto ModelGet::executeRT() -> int
	{
		std::cout << "size of slave pool:" << ecMaster()->slavePool().size() << std::endl;
		float force[6] = { 0 };
		auto get_force_data = [&](float* data)
		{
			for (std::size_t i = 0; i < 6; ++i)
			{
				ecMaster()->slavePool()[12].readPdo(0x6020, 0x01 + i, data + i, 32);
			}
		};

		get_force_data(force);
		std::cout << "force data:" << std::endl;
		aris::dynamic::dsp(1, 6, force);

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
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

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
			for (int i = 0; i < 12; i++)
			{
				if (std::fabs(current_angle[i] - init_pos[i]) >= move)
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
			mout() << current_angle[0] << current_angle[1] << current_angle[2] << current_angle[3] << current_angle[4]
				<< current_angle[5] << current_angle[6] << current_angle[7] << current_angle[8] << current_angle[9]
				<< current_angle[10] << current_angle[11] << std::endl;
		}


		if (motorsPositionCheck())
		{

			mout() << "Back to Init Position" << std::endl;
			modelBase()->setInputPos(current_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;

			mout() << "current angle: \n" << current_angle[0] << current_angle[1] << current_angle[2] << current_angle[3] << current_angle[4]
				<< current_angle[5] << current_angle[6] << current_angle[7] << current_angle[8] << current_angle[9]
				<< current_angle[10] << current_angle[11] << std::endl;

			modelBase()->getOutputPos(eePos);
			mout() << "current end position: \n" << eePos[0] << eePos[1] << eePos[2] << eePos[3] << eePos[4] << eePos[5]
				<< eePos[6] << eePos[7] << eePos[8] << eePos[9] << eePos[10] << eePos[11] << std::endl;


			return 0;
		}
		else
		{
			if (count() == 28000)
			{
				mout() << "Over Time" << std::endl;
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
			double begin_angle[12]{ 0 };

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


		GravComp gc;
		static double tolerance = 0.0001;

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


		// Only One Arm Move Each Command
		auto saJointMove = [&](double target_mp_[6], int m_)
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

		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};

		//Ethercat Warning
		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (int i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
				{
					mout() << "Force Sensor Error" << std::endl;
				}


				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}
			if (!init_)
			{
				if (m_ == 0)
				{
					mout() << "Compensate Init Force A1" << std::endl;
					mout() << "Init 1 : " << imp_->arm1_init_force[0] << '\t' << imp_->arm1_init_force[1] << '\t' << imp_->arm1_init_force[2] << '\t'
						<< imp_->arm1_init_force[3] << '\t' << imp_->arm1_init_force[4] << '\t' << imp_->arm1_init_force[5] << std::endl;
				}
				else if (m_ == 1)
				{
					mout() << "Compensate Init Force A2" << std::endl;
					mout() << "Init 2 : " << imp_->arm2_init_force[0] << '\t' << imp_->arm2_init_force[1] << '\t' << imp_->arm2_init_force[2] << '\t'
						<< imp_->arm2_init_force[3] << '\t' << imp_->arm2_init_force[4] << '\t' << imp_->arm2_init_force[5] << std::endl;
				}
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};

		auto caculateAvgForce = [=](double arm1_force_data_[6], double arm2_force_data_[6], double arm1_temp_force_[6], double arm2_temp_force_[6], int count_)
		{
			if (count() < imp_->current_stop_time + imp_->stop_time)
			{

				if (count() % 200 == 0 && imp_->accumulation_count < 10)
				{
					double temp1[6]{ 1,2,3,4,5,6 };
					double temp2[6]{ 1,2,3,4,5,6 };

					//getForceData(temp1, 0, imp_->init);
					//getForceData(temp2, 1, imp_->init);

					for (int i = 0; i < 6; i++)
					{
						arm1_temp_force_[i] += temp1[i];
						arm2_temp_force_[i] += temp2[i];
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
					arm1_force_data_[i] = arm1_temp_force_[i] / 10.0;
					arm2_force_data_[i] = arm2_temp_force_[i] / 10.0;
				}
				mout() << "Arm 1 Force Data " << count_ << '\n' << arm1_force_data_[0] << '\t' << arm1_force_data_[1] << '\t' << arm1_force_data_[2] << '\t'
					<< arm1_force_data_[3] << '\t' << arm1_force_data_[4] << '\t' << arm1_force_data_[5] << std::endl;
				mout() << "Arm 2 Force Data " << count_ << '\n' << arm2_force_data_[0] << '\t' << arm2_force_data_[1] << '\t' << arm2_force_data_[2] << '\t'
					<< arm2_force_data_[3] << '\t' << arm2_force_data_[4] << '\t' << arm2_force_data_[5] << std::endl;
			}
			else
			{
				mout() << "Flag Change " << count_ << std::endl;
				imp_->stop_flag = false;
			}
		};

		
		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (imp_->stop_flag)
		{
			if (imp_->stop_count == 1)
			{

				caculateAvgForce(imp_->arm1_force_data_1, imp_->arm2_force_data_1, imp_->arm1_temp_force_1, imp_->arm2_temp_force_1, 1);
				

			}
			else if (imp_->stop_count == 2)
			{

				caculateAvgForce(imp_->arm1_force_data_2, imp_->arm2_force_data_2, imp_->arm1_temp_force_2, imp_->arm2_temp_force_2, 2);

			}
			else if (imp_->stop_count == 3)
			{

				caculateAvgForce(imp_->arm1_force_data_3, imp_->arm2_force_data_3, imp_->arm1_temp_force_3, imp_->arm2_temp_force_3, 3);

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
			static double init_angle[12] =
			{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

			static double angle1[12] =
			{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 ,
			0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			static double angle2[12] =
			{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 3, 0 ,
			0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			static double angle3[12] =
			{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 ,
			0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };


			//// Arm 1 Angle
			//static double init_angle1[6]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 };

			//static double angle1_1[6]{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 };

			//static double angle1_2[6]{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0 };

			//static double angle1_3[6]{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 };




			//// Arm 2 Angle
			//static double init_angle2[6]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

			//static double angle2_1[6]{ 0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			//static double angle2_2[6]{ 0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			//static double angle2_3[6]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };

			if (!imp_->init)
			{
				dualArm.setInputPos(init_angle);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(init_angle);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(init_angle[i]);
				}

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{

					//getForceData(imp_->arm1_init_force, 0, imp_->init);
					//getForceData(imp_->arm2_init_force, 1, imp_->init);

					mout() << "Init Complete" << std::endl;

					imp_->init = true;
				}

			}

			if (!imp_->target1_reached && imp_->init)
			{
				dualArm.setInputPos(angle1);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(angle1);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(angle1[i]);
				}

				if (motorsPositionCheck(current_angle, angle1, 12))
				{
					mout() << "Target 1 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_1);
					eeA2.getMpm(imp_->arm2_ee_pm_1);

					imp_->target1_reached = true;
					imp_->stop_count = 1;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}

			}
			else if (imp_->target1_reached && !imp_->target2_reached)
			{
				dualArm.setInputPos(angle2);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(angle2);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(angle2[i]);
				}

				if (motorsPositionCheck(current_angle, angle2, 12))
				{
					mout() << "Target 2 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_2);
					eeA2.getMpm(imp_->arm2_ee_pm_2);

					imp_->target2_reached = true;
					imp_->stop_count = 2;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}
			}
			else if (imp_->target2_reached && !imp_->target3_reached)
			{
				dualArm.setInputPos(angle3);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(angle3);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(angle3[i]);
				}

				if (motorsPositionCheck(current_angle, angle3, 12))
				{
					mout() << "Target 3 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_3);
					eeA2.getMpm(imp_->arm2_ee_pm_3);

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
				dualArm.setInputPos(init_angle);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(init_angle);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(init_angle[i]);
				}

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{
					mout() << "Back To Init Pos" << std::endl;
					imp_->target4_reached = true;

				}

			}
			else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
			{





				//Arm 1
				double arm1_t_vector[9]{ 0 };
				double arm1_f_vector[9]{ 0 };

				double arm1_f_matrix[54]{ 0 };
				double arm1_r_matrix[54]{ 0 };


				double arm1_ee_rm_1[9]{ 0 };
				double arm1_ee_rm_2[9]{ 0 };
				double arm1_ee_rm_3[9]{ 0 };

				double arm1_current_force[6]{ 0 };

				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_1, arm1_ee_rm_1);
				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_2, arm1_ee_rm_2);
				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_3, arm1_ee_rm_3);



				gc.getTorqueVector(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_t_vector);
				gc.getForceVector(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_f_vector);

				gc.getFMatrix(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_f_matrix);
				gc.getRMatrix(arm1_ee_rm_1, arm1_ee_rm_2, arm1_ee_rm_3, arm1_r_matrix);

				gc.getPLMatrix(arm1_f_matrix, arm1_t_vector, imp_->arm1_p_vector);
				gc.getPLMatrix(arm1_r_matrix, arm1_f_vector, imp_->arm1_l_vector);

				double arm1_current_ee_pm[16]{ 0 };
				eeA1.getMpm(arm1_current_ee_pm);

				gc.getCompFT(arm1_current_ee_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, imp_->arm1_comp_f);


				//getForceData(arm1_current_force, 0, imp_->init);


				//Arm 2
				double arm2_t_vector[9]{ 0 };
				double arm2_f_vector[9]{ 0 };

				double arm2_f_matrix[54]{ 0 };
				double arm2_r_matrix[54]{ 0 };


				double arm2_ee_rm_1[9]{ 0 };
				double arm2_ee_rm_2[9]{ 0 };
				double arm2_ee_rm_3[9]{ 0 };

				double arm2_current_force[6]{ 0 };

				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_1, arm2_ee_rm_1);
				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_2, arm2_ee_rm_2);
				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_3, arm2_ee_rm_3);



				gc.getTorqueVector(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_t_vector);
				gc.getForceVector(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_f_vector);

				gc.getFMatrix(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_f_matrix);
				gc.getRMatrix(arm2_ee_rm_1, arm2_ee_rm_2, arm2_ee_rm_3, arm2_r_matrix);

				gc.getPLMatrix(arm2_f_matrix, arm2_t_vector, imp_->arm2_p_vector);
				gc.getPLMatrix(arm2_r_matrix, arm2_f_vector, imp_->arm2_l_vector);

				double arm2_current_ee_pm[16]{ 0 };
				eeA2.getMpm(arm2_current_ee_pm);

				gc.getCompFT(arm2_current_ee_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, imp_->arm2_comp_f);

				//getForceData(arm2_current_force, 1, imp_->init);

				gc.savePLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);


				mout() << "Current Arm1 Force After Compensation:" << '\n' << arm1_current_force[0] + imp_->arm1_comp_f[0] << '\t' << arm1_current_force[1] + imp_->arm1_comp_f[1] << '\t'
					<< arm1_current_force[2] + imp_->arm1_comp_f[2] << '\t' << arm1_current_force[3] + imp_->arm1_comp_f[3] << '\t'
					<< arm1_current_force[4] + imp_->arm1_comp_f[4] << '\t' << arm1_current_force[5] + imp_->arm1_comp_f[5] << std::endl;

				//mout() << "Current Arm1 End Pos:" << '\n' << arm1_current_ee_pm[0] << '\t' << arm1_current_ee_pm[1] << '\t'
				//	<< arm1_current_ee_pm[2] << '\t' << arm1_current_ee_pm[3] << '\t'
				//	<< arm1_current_ee_pm[4] << '\t' << arm1_current_ee_pm[5] << std::endl;

				mout() << "Current Arm2 Force After Compensation:" << '\n' << arm2_current_force[0] + imp_->arm2_comp_f[0] << '\t' << arm2_current_force[1] + imp_->arm2_comp_f[1] << '\t'
					<< arm2_current_force[2] + imp_->arm2_comp_f[2] << '\t' << arm2_current_force[3] + imp_->arm2_comp_f[3] << '\t'
					<< arm2_current_force[4] + imp_->arm2_comp_f[4] << '\t' << arm2_current_force[5] + imp_->arm2_comp_f[5] << std::endl;

				//mout() << "Current Arm2 End Pos:" << '\n' << arm2_current_ee_pm[0] << '\t' << arm2_current_ee_pm[1] << '\t'
				//	<< arm2_current_ee_pm[2] << '\t' << arm2_current_ee_pm[3] << '\t'
				//	<< arm2_current_ee_pm[4] << '\t' << arm2_current_ee_pm[5] << std::endl;

				

				return 0;

			}

			if (count() == 100000)
			{


				mout() << "Over Time" << std::endl;


			}

			return 100000 - count();
			
		}
	}
	ModelComP::ModelComP(const std::string& name) : imp_(new Imp)
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

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);


	}
	auto ForceAlign::executeRT()->int
	{

		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		imp_->m_ = int32Param("model");

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


		GravComp gc;



		double current_vel[6]{ 0 };
		double current_pos[6]{ 0 };

		double current_force[6]{ 0 };
		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };
		double current_pm[16]{ 0 };

		double comp_force[6]{ 0 };
		double actual_force[6]{ 0 };

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}

			}

			if (m_ == 0)
			{
				//				data_[0] = -data_[0];
				//				data_[1] = -data_[1];

				//				data_[3] = -data_[3];
				//				data_[4] = -data_[4];
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


		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};

		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
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


		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())
			{
				mout() << "forward fail" << std::endl;
			}

			daJointMove(init_angle);
			if (motorsPositionCheck(current_angle, init_angle, 12))
			{
				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);
				mout() << "Back To Init" << std::endl;

				imp_->init = true;
			}
		}
		else
		{
			//Arm1
			if (imp_->m_ == 0)
			{
				//Update Statue
				eeA1.getV(current_vel);
				eeA1.getP(current_pos);
				eeA1.getMpm(current_pm);

				std::copy(current_angle, current_angle + 6, current_sa_angle);

				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}



				//Contact Check

				//test
				if (count() > 28000 && count() <= 29000)
				{
					imp_->actual_force[2] = -0.55;
				}
				else if (count() > 29000 && count() <= 33000)
				{
					imp_->actual_force[2] = -5;
				}
				else if (count() > 33000)
				{
					imp_->actual_force[2] = -7;
				}


				//Get Actual Force
				//getForceData(current_force, imp_->m_, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				if (!imp_->contact_check)
				{
					if (abs(imp_->actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a1, 0);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - imp_->actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a1, 0);

					//if (count() % 100 == 0)
					//{
					//	//mout() << "count(): " << count() << std::endl;
					//	mout() << "force: " << imp_->actual_force[0] << '\t' << imp_->actual_force[1] << '\t' << imp_->actual_force[2] << '\t'
					//		<< imp_->actual_force[3] << '\t' << imp_->actual_force[4] << '\t' << imp_->actual_force[5] << std::endl;

					//}

				}
			}
			//Arm2
			else if (imp_->m_ == 1)
			{
				//Update Statue
				eeA2.getV(current_vel);
				eeA2.getP(current_pos);
				eeA2.getMpm(current_pm);

				std::copy(current_angle + 6, current_angle + 12, current_sa_angle);
				

				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}

				//test
				if (count() > 28000 && count() <= 29000)
				{
					imp_->actual_force[2] = -0.55;
				}
				else if (count() > 29000 && count() <= 33000)
				{
					imp_->actual_force[2] = -5;
				}
				else if (count() > 33000)
				{
					imp_->actual_force[2] = -7;
				}

				//Contact Check

				//Get Actual Force
				//getForceData(current_force, imp_->m_, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				if (!imp_->contact_check)
				{
					if (abs(imp_->actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a2, 1);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - imp_->actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a2, 1);

					if (count() % 1000 == 0)
					{
						//mout() << "count(): " << count() << std::endl;
						mout() << "force: " << imp_->actual_force[0] << '\t' << imp_->actual_force[1] << '\t' << imp_->actual_force[2] << '\t'
							<< imp_->actual_force[3] << '\t' << imp_->actual_force[4] << '\t' << imp_->actual_force[5] << std::endl;

					}

				}
			}
			//Error
			else 
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}
			
		}
		//Over Time Exit
		if (count() == 800000)
		{
			mout() << "Over Time" << std::endl;
		}

		return 800000 - count();

	}
	ForceAlign::ForceAlign(const std::string& name) : imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_fa\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceAlign::~ForceAlign() = default;


	auto ForceKeep::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceKeep::executeRT() -> int
	{
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

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}

			}



		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
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


		for (std::size_t i = 0; i < 6; ++i)
		{
			imp_->actual_force[i] = 0;
		}



		if (count() > 2000 && count() <= 4000)
		{
			//mout() << "fex" << std::endl;
			imp_->actual_force[3] = 0.5;
		}
		//else if (count() > 9000 && count() <= 10000)
		//{
		//	//mout() << "fex" << std::endl;
		//	imp_->actual_force[0] = 5;
		//}


		 //else if (count() > 2000 && count() <= 3000)
		 //{
		 //	imp_->actual_force[2] = -10;
		 //	imp_->actual_force[0] = -15;
		 //}
		 //else if (count() > 3000 && count() <= 4000)
		 //{
		 //	//imp_->actual_force[1] = -5;
		 //	imp_->actual_force[4] = -6;
		 //}




		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init)
		{



			dualArm.setInputPos(init_angle);

			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}

			//Test
			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}

			//daJointMove(init_angle);

			//if (count() % 1000 == 0)
			//{

			//	mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
			//		<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

			//}

			if (motorsPositionCheck(current_angle, init_angle, 12))
			{

				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);

				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);

				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}


		}
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };
				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				//getForceData(current_force, 0, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				//Coordinate Transform Arm1
				transform_force[0] = actual_force[2];
				transform_force[1] = -actual_force[1];
				transform_force[2] = actual_force[0];

				transform_force[3] = actual_force[5];
				transform_force[4] = -actual_force[4];
				transform_force[5] = actual_force[3];


				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->a1_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a1_K[i] * (current_pos[i] - imp_->arm1_x_d[i])) / imp_->a1_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}

				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm1_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
				aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);

				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);



				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->a1_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a1_K[i + 3] * pose_error[i]) / imp_->a1_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a1, 0);




				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					// mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					// 	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

					mout() << "error: " << pose_error[0] << '\t' << pose_error[1] << '\t' << pose_error[2] << std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);

				//getForceData(current_force, 1, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				//Coordinate Transform Arm2
				transform_force[0] = -actual_force[2];
				transform_force[1] = actual_force[1];
				transform_force[2] = actual_force[0];

				transform_force[3] = -actual_force[5];
				transform_force[4] = actual_force[4];
				transform_force[5] = actual_force[3];

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					//acc[i] = (imp_->f_d[i] - imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->M[i];

					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->a2_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a2_K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->a2_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm2_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
				aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);


				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);

			


				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->a2_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a2_K[i + 3] * pose_error[i]) / imp_->a2_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);



				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

		return 30000 - count();
	}
	ForceKeep::ForceKeep(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fk\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceKeep::~ForceKeep() = default;




	auto ForceDrag::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceDrag::executeRT() -> int
	{
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

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}

			}

		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
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


		//for (std::size_t i = 0; i < 6; ++i)
		//{
		//	imp_->actual_force[i] = 0;
		//}



		//if (count() > 5000 && count() <= 7000)
		//{
		//	//mout() << "fex" << std::endl;
		//	imp_->actual_force[0] = -5;
		//}
		//else if (count() > 9000 && count() <= 10000)
		//{
		//	//mout() << "fex" << std::endl;
		//	imp_->actual_force[0] = 5;
		//}


		 //else if (count() > 2000 && count() <= 3000)
		 //{
		 //	imp_->actual_force[2] = -10;
		 //	imp_->actual_force[0] = -15;
		 //}
		 //else if (count() > 3000 && count() <= 4000)
		 //{
		 //	//imp_->actual_force[1] = -5;
		 //	imp_->actual_force[4] = -6;
		 //}




		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init)
		{



			dualArm.setInputPos(init_angle);

			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}

			//Test
			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}

			//daJointMove(init_angle);

			//if (count() % 1000 == 0)
			//{

			//	mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
			//		<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

			//}

			if (motorsPositionCheck(current_angle, init_angle, 12))
			{

				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);

				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);

				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}


		}
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };

				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				//getForceData(current_force, 0, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				//Coordinate Transform Arm1
				transform_force[0] = actual_force[2];
				transform_force[1] = -actual_force[1];
				transform_force[2] = actual_force[0];

				transform_force[3] = actual_force[5];
				transform_force[4] = -actual_force[4];
				transform_force[5] = actual_force[3];

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };




				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a1, 0);


				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);

				//getForceData(current_force, 1, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				//Coordinate Transform Arm2
				transform_force[0] = -actual_force[2];
				transform_force[1] = actual_force[1];
				transform_force[2] = actual_force[0];

				transform_force[3] = -actual_force[5];
				transform_force[4] = actual_force[4];
				transform_force[5] = actual_force[3];

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };




				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				//mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
				//	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				//mout() << dx[0] << '\t' << dx[1] << '\t' << dx[2] << '\t'
				//	<< dx[3] << '\t' << dx[4] << '\t' << dx[5] << std::endl;

				//mout() << acc[0] << '\t' << acc[1] << '\t' << acc[2] << '\t'
				//	<< acc[3] << '\t' << acc[4] << '\t' << acc[5] << std::endl;

				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);



				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

		return 10000 - count();
	}
	ForceDrag::ForceDrag(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fd\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceDrag::~ForceDrag() = default;





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
		aris::core::class_<ForceAlign>("ForceAlign")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ForceKeep>("ForceKeep")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ForceDrag>("ForceDrag")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelTest>("ModelTest")
			.inherit<aris::plan::Plan>();

	}





}





