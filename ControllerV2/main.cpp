#include "cmath"
#pragma comment(lib, "ExternalControllerApi.lib")
#include "ExternalControllerApi.h" // defines the C functions for the external balded controller
#include "risoe_controller_fcns.h" // defines the C functions of the risoe DTU controller
#include "string"
//#include "iostream"
//#include "stdio.h""
using namespace GHTurbineInterface;

double towerfrequencyForPitchFilt = 0; 	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
double filtFreqPowerError_pitch = 0; 	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
int useMW = 20; // choose 20 for 20 MW turbine or else for 10 MW
int use90mpsVersion = 1;			// 1 if 90 m/s 2B version, else 100 m/s version is used
int use2B_upscaled = 1;				// 1 if the upscaled versions of the 2B turbines should be used (they have the same power at 3B rated wind), else 2 etc.
int useSpeedExZone = 0;				// 1 = using exclusion zone; else not
int usePitchTowerFA_Damper = 0;		// 1 = using active tower fore-aft  damper by pitching; else not
int useGenTorqueTowerSS_Damper = 0; // 1 = using active tower side-side damper by generator torque; else not
int useConstPower = 2;			// 15: Generator control switch[1 = constant power, 2 = constant torque] default = 2
int nmbrBlades = 3;
double maxOverspeedFactor = 1.12; // factor to apply to omega rated [default = 1.12]

int useTxtFieldInput = 1;

double tower_FA_velo = 0, tower_SS_velo = 0;
double transition_exZone_Qg_min_partial, check_three = 0, torque_rate_limit_compensator = 0;
int trigger_ex_escape = 0, trigger_ex_entrance = 0;

				// ---- user specific controller parameters ---- //
//int UseInitialValues = 0, UseActivePTC = 0, UseLowPassWindSpeedFilter = 1, UseConstPowerProd = 0; // 1 ^= use; 0^= don't use
//int TimeToStopFeedback = 800;  // use to as start for static wind runs, otherwise put a high value (in seconds) (bigger 100)
//double Delta3_angle = 45;       // delta 3 angle in deg (just needed for active PTC)

double pred_stepno = 0; // predicted step number from last exclusion zone function iteration
double switch_ex_zone = 0; // switch omega error hysteresis in exclusion zone
double ex_zone_sign = 1;
int GridlossDetected = 0; // switch for detecting fault cut out
int StuckedPitchDetected = 0; // switch for detecting fault cut out
double AllPitchLimitSwitchTripped; // used to enable 2 and 3 bladed contoller
double omegaAtGridloss = 0;
//double NormalOrEStopDetected = 0; 
double t_cutout_delay = 30;
 

double GearBoxRatio = 47.6; // be careful, does vary with MW (20MW ~ 48 ; 10MW = 50)
double drive_train_loss = 0.94; // full drive train loss in %
double dummy = 0; // just for checks
double switch1_pitang_lower1, switch1_pitang_upper1, switch1_pitang_lower2, switch1_pitang_upper2;

// Local variables
double time, omega, omegafilt, wsp, WSPfilt, // domega_dt_filt,
omega_err_filt_pitch, omega_err_filt_speed, omega_dtfilt,
ommin1, ommin2, ommax1, ommax2,
meanpitang, meanpitangfilt, theta_min,
aero_gain, x, // unnecessary: dummy,
Qg_min_partial, Qg_max_partial, Qg_min_full, Qg_max_full,
Qgen_ref, theta_col_ref, Pe_ref, Qdamp_ref;

double e_pitch[2];
double y[2];
double pitang[3];
double thetaref[3];
double kgain_torque[3];
double kgain_pitch[3][2];

double nominalWindSpeed;																	// usage of wind input unclear...
double SecondWindInput = 0;  // Inp 6.2														// NO CLUE WHAT THIS VALUE SHOULD BE ???!!!

double tellItOnce = 0;
double Init_cutout_Qgen_ref = 0;
double oldMeanpitang = 0;
double pitchADC = 0; // actuator duty cycle

// Log messages
//int CheckTime = 50, CheckTimeInterv = 5;  // Time and time interval to check control inputs in s (to avoid information overflow) 
//char contmodestring[40], contmodeoldstring[40], LogText[40];
char* ErrorMess;

extern "C"
{
	//////////////////////////////////////////////////////////////////////////
	// Function control1 uses a simple squared law on RPM to determine		//
	// the torque that should be demanded from the turbine.  PI(D) is used	//
	// to control the speed of the turbine in region III while holding		//
	// constant torque.  In between the two regions is a region of			//
	// constant speed controlled by generator torque (region 2 1/2).		//
	// Yaw control is also done here.										//
	//////////////////////////////////////////////////////////////////////////

	int __declspec( dllexport ) __cdecl CONTROLLER (const turbine turbine_id);

	/*! Controller to be run on first timestep. */
	int __declspec( dllexport ) __cdecl CONTROLLER_INIT (const turbine turbine_id)
	{
		if (useSpeedExZone) ReportInfoMessage(turbine_id, "Using untested speed exclusion zone");
		
		char *reportMessage = new char[55];// [3];
		//sprintf(reportMessage, "this is %d\n", 5);

		ReportInfoMessage(turbine_id, "received control parameters:"); 
		const char *userParameters = GetUserParameters(turbine_id); ReportInfoMessage(turbine_id, userParameters); // reading in txt field from bladed external controller
		std::string strParam(userParameters); // reformating const char* in str

		int IndxParamBegin = 0;
		int IndxParamEnd = 0;
		const int nmbrOfInputValues = 15;
		std::string parameterNames[nmbrOfInputValues] = { "pitchPGain",					// 1 // names of parameters that should be swapped from txt field to controller 
														"pitchIGain",					// 2 // (strings have to be identical!!!)
														"pitchPpowerGain",				// 3 // (position in this list has to be identical with parameter index ~15 lines later!!!)
														"pitchIpowerGain",				// 4 
														"nmbrBlades",					// 5 
														"maxOverspeedFactor",			// 6
														"towerfrequencyForPitchFilt",	// 7
														"filtFreqPowerError_pitch",		// 8 // 0 if not to use, else frequency in Hz
														"use90mpsVersion",				// 9
														"use2B_upscaled",				// 10
														"useSpeedExZone",				// 11
														"usePitchTowerFA_Damper",		// 12
														"useGenTorqueTowerSS_Damper",	// 13
														"useConstPower",				// 14
														"useMW" };						// 15 != nmbrOfInputValues
		double iterParameter[nmbrOfInputValues];
		for (int i = 0; i < nmbrOfInputValues; i++) {
			sprintf_s(reportMessage, 50, "searching %s at P%i", parameterNames[i].c_str(), i+1 ); ReportInfoMessage(turbine_id, reportMessage); //just for checking
			IndxParamBegin = strParam.find(parameterNames[i]) +parameterNames[i].length() + 1; // start index for value of search word
			IndxParamEnd = strParam.find(';', IndxParamEnd + 1);								// end  index for value of search word
			//sprintf_s(reportMessage, 50, "value writen from %i to %i", IndxParamBegin, IndxParamEnd); ReportInfoMessage(turbine_id, reportMessage); //just for checking
			iterParameter[i] = std::stod(strParam.substr(IndxParamBegin, IndxParamEnd - IndxParamBegin)); // convert str to double
			sprintf_s(reportMessage, 50, "found %.11f", iterParameter[i]); ReportInfoMessage(turbine_id, reportMessage); //just for checking
		}
		ReportInfoMessage(turbine_id, "Done swapping values\n"); //just for checking
		double pitchPGain	= iterParameter[0];
		double pitchIGain	= iterParameter[1];
		double pitchPpowerGain = iterParameter[2];
		double pitchIpowerGain = iterParameter[3];

		nmbrBlades		= int(iterParameter[4]);
		maxOverspeedFactor =  iterParameter[5];		// factor to apply to omega rated [default = 1.12]
		towerfrequencyForPitchFilt = iterParameter[6];	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
		filtFreqPowerError_pitch = iterParameter[7];	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
		use90mpsVersion = int(iterParameter[8]);		// 1 if 90 m/s 2B version, else 100 m/s version is used
		use2B_upscaled	= int(iterParameter[9]);		// 1 if the upscaled versions of the 2B turbines should be used (they have the same power at 3B rated wind), else 2 etc.
		useSpeedExZone	= int(iterParameter[10]);		// 1 = using exclusion zone; else not
		usePitchTowerFA_Damper = int(iterParameter[11]);	// 1 = using active tower fore-aft  damper by pitching; else not
		useGenTorqueTowerSS_Damper = int(iterParameter[12]); // 1 = using active tower side-side damper by generator torque; else not
		useConstPower	= int(iterParameter[13]);		// 15: Generator control switch[1 = constant power, 0 = constant torque] default = 0
		useMW			= int(iterParameter[14]);		// choose 20 for 20 MW turbine or else for 10 MW

		

		// start subroutine init_regulation
		// DEC$ ATTRIBUTES DLLEXPORT, C, ALIAS:’init_regulation’::init_regulation
		// Local vars
		double minimum_pitch_angle;
		//int i, ifejl;
		//char* text32 = "text32";
		//bool findes;

		// Input array1 must contain:
		double array1[50];
		if (useMW == 20) {
			ReportInfoMessage(turbine_id, ">>> Running DTU 20MW RWT controller. <<<");
			//	 : Overall parameters
			array1[0] = 20000.00;	//  1: Rated power[kW]
			array1[1] = 0.237906864; // 3B: 0.237906864;  // org.: 0.45000;	//  2: Minimum rotor speed[rad / s]
			array1[2] = 0.678034562; // 3B: 0.678034562; // org.: 0.74617;	//  3: Rated rotor speed[rad / s]
			array1[3] = 42426406.87; // org.: 4.4123E+07;	//  4: Maximum allowable generator torque[Nm]                    without gearbox???
			array1[4] = 100;		//  5: Minimum pitch angle, theta_min[deg],
									//	 : if | theta_min | >90, then a table of <wsp, theta_min> is read
									//   : from a file named ’wptable.n’, where n = int(theta_min)							 // AND WHERE IS THAT FILE???!!!
			array1[5] = 90;			//  6: Maximum pitch angle[deg]
			array1[6] = 7.071;		//  7: Maximum pitch velocity operation[deg / s]
			array1[7] = 0.14142;	//  8: Frequency of generator speed filter[Hz]
			array1[8] = 0.7;		//  9: Damping ratio of speed filter[-]
			array1[9] = 0; // 1.31; // org: 0.35000;	// 10: Frequency of free - free DT torsion mode[Hz], if zero no notch filter used
			//	 : Partial load control parameters
			array1[10] = 5.1376E+07;// 11: Optimal Cp tracking K factor[Nm / (rad / s) ^ 2],
									//	 : Qg = K * Omega ^ 2, K = eta * 0.5*rho*A*Cp_opt*R ^ 3 / lambda_opt ^ 3
			array1[11] = 2.73E+08*pow(0.6, 3);// *0.593;	// 12: Proportional gain of torque controller[Nm / (rad / s)]                             // factor of 0.6 to tune prop. and integ. amplitude to same abs size
			ReportInfoMessage(turbine_id, "*0.6^8 on gen PI int. gain and *0.6^3 on prop.");
			array1[12] = 4.34E+07*pow(0.6, 8); //*0.6*0.6*0.6*0.6; // *0.593* 0.593* 0.593;	//;	// 13: Integral gain of torque controller[Nm / rad]
			sprintf_s(reportMessage, 55, "Gen: Prop. gain = %.0f, Int. gain = %.0f", array1[11], array1[12]);	ReportInfoMessage(turbine_id, reportMessage);
			array1[13] = 0;			// 14: Differential gain of torque controller[Nm / (rad / s ^ 2)]
			//	 : Full load control parameters
			array1[14] = useConstPower;			// 15: Generator control switch[1 = constant power, 2 = constant torque]
			array1[15] = 0.5245;	// 16: Proportional gain of pitch controller[rad / (rad / s)]
			array1[16] = 0.0999;	// 17: Integral gain of pitch controller[rad / rad]
			array1[17] = 0;			// 18: Differential gain of pitch controller[rad / (rad / s ^ 2)]
			if (nmbrBlades == 2) {
				array1[15] = array1[15] *0.5;//2B90: * 0.57;	// 16: Proportional gain of pitch controller[rad / (rad / s)]
				array1[16] = array1[16] *0.5;// 2B90: *0.56;	// 17: Integral gain of pitch controller[rad / rad]
				ReportInfoMessage(turbine_id, "*0.5 on pitch PI prop. gain and *0.5 on int.");
			}
			array1[18] = 2.00E-09;	// 19: Proportional power error gain [rad/W]
			array1[19] = 1.41E-09;	// 20: Integral power error gain [rad/(Ws)]
			if (useTxtFieldInput) {
				array1[15] = pitchPGain;
				array1[16] = pitchIGain;
				array1[18] = pitchPpowerGain;
				array1[19] = pitchIpowerGain;
			}
			sprintf_s(reportMessage, 55, "Pitch: Prop. gain = %f, Int. gain = %f", array1[15], array1[16]); ReportInfoMessage(turbine_id, reportMessage);
			array1[20] = 198.329;	// 21: Coefficient of linear term in aerodynamic gain scheduling, KK1 [deg]
			array1[21] = 693.222;	// 22: Coefficient of quadratic term in aerodynamic gain scheduling, KK2 [deg^2]
									//	 : (if zero, KK1 = pitch angle at double gain)
			array1[22] = 1.3;		// 23: constant 23	; Relative speed for double nonlinear gain [-]
			//	 : Cut-in simulation parameters
			array1[23] = 5; // 0.1414;	// 24: Cut-in time [s], if zero no cut-in simulated (SIMULATION TYPE CHECKED BY BLADED)
			array1[24] = 5.6569;	// 25: Time delay for soft start [1/1P]
			//	 : Cut-out simulation parameters
			array1[25] = 20;			// 26: Cut-out time [s], if zero no cut-out simulated (SIMULATION TYPE CHECKED BY BLADED)
			array1[26] = 2; // 0.1414;	// 27: Time constant for 1st order filter lag of torque cut-out [s]
			array1[27] = 1;			// 28: Stop type [1=linear two pitch speed stop, 2=exponential pitch speed stop]
			array1[28] = 1.4142;	// 29: Time delay for pitch stop 1 [s] 
			array1[29] = 14.1421;	// 30: Maximum pitch velocity during stop 1 [deg/s]
			array1[30] = 1.4142;	// 31: Time delay for pitch stop 2 [s]
			array1[31] = 7.0711;	// 32: Maximum pitch velocity during stop 2 [deg/s]
									//	 : Expert parameters (keep default values unless otherwise given)
			array1[32] = 0;		// 33: Lower angle above lowest minimum pitch angle for switch [deg]
			array1[33] = 2;		// 34: Upper angle above lowest minimum pitch angle for switch [deg], if equal then hard switch
			array1[34] = 95; // 95;		// 35: Ratio between filtered and reference speed for fully open torque limits [%] (WILL BE FACTORED TO [-] BELOW!)
			ReportInfoMessage(turbine_id, "*95% rot. speed gen PI zone.");
			array1[35] = 3; // 5;// org: 7.0711;	// 36: Time constant of 1st order filter on wind speed used for minimum pitch [1/1P]
			array1[36] = 3; // org: 7.0711;	// 37: Time constant of 1st order filter on pitch angle for gain scheduling [1/1P]
			array1[37] = 0; // 60000000;		// 10000000 too little effect; // 50000000 too much effect	// 38: Proportional gain of DT damper [Nm/(rad/s)], requires frequency in input 10							  // input 38 NOT GIVEN !!!!!!!

		}
		else {
			ReportInfoMessage(turbine_id, "Running DTU 10MW RWT controller.");
			GearBoxRatio = 50;
			//	 : Overall parameters
			array1[0] = 10000.00;	//  1: Rated power[kW]
			array1[1] = 0.336451114; //org: 0.733;  // org.: 0.45000;	//  2: Minimum rotor speed[rad / s]
			array1[2] = 0.958885674; // org: 1.005; // org.: 0.74617;	//  3: Rated rotor speed[rad / s]
			array1[3] = 15.6E+06; // org.: 4.4123E+07;	//  4: Maximum allowable generator torque[Nm]                    without gearbox???
			array1[4] = 100;		//  5: Minimum pitch angle, theta_min[deg],
									//	 : if | theta_min | >90, then a table of <wsp, theta_min> is read
									//   : from a file named ’wptable.n’, where n = int(theta_min)							 // AND WHERE IS THAT FILE???!!!
			array1[5] = 90;			//  6: Maximum pitch angle[deg]
			array1[6] = 10;		//  7: Maximum pitch velocity operation[deg / s]
			array1[7] = 0.2;	//  8: Frequency of generator speed filter[Hz]
			array1[8] = 0.7;		//  9: Damping ratio of speed filter[-]
			array1[9] = 0.64; // org: 0.35000;	// 10: Frequency of free - free DT torsion mode[Hz], if zero no notch filter used
			//	 : Partial load control parameters
			array1[10] = 9.5E+06;// 11: Optimal Cp tracking K factor[Nm / (rad / s) ^ 2],
									//	 : Qg = K * Omega ^ 2, K = eta * 0.5*rho*A*Cp_opt*R ^ 3 / lambda_opt ^ 3
			array1[11] = 7.33E+07*pow(0.6, 3); //org: 2.73E+08*0.6*0.6;	// 12: Proportional gain of torque controller[Nm / (rad / s)]                             // factor of 0.6 to tune prop. and integ. amplitude to same abs size
			//ReportInfoMessage(turbine_id, "*0.6*0.6*0.6*0.6 on gen PI int. gain and *0.6*0.6 on prop.");
			array1[12] = 1.32E+07*pow(0.6, 8); //org: 4.34E+07*0.6*0.6*0.6*0.6;	// 13: Integral gain of torque controller[Nm / rad]
			ReportInfoMessage(turbine_id, "*0.6^3 on prop and *0.6^10 on int gain");
			ReportInfoMessage(turbine_id, "was good with *0.6^3 on prop and *0.6^8 on int gain");
			array1[13] = 0.0;			// 14: Differential gain of torque controller[Nm / (rad / s ^ 2)]
			//	 : Full load control parameters
			array1[14] = useConstPower;			// 15: Generator control switch[1 = constant power, 2 = constant torque]
			array1[15] = 0.592;		// 16: Proportional gain of pitch controller[rad / (rad / s)]
			array1[16] = 0.133;		// 17: Integral gain of pitch controller[rad / rad]
			if (nmbrBlades == 2) {
				array1[15] = array1[15] * 0.5;// 2B90: * 0.57;	// 16: Proportional gain of pitch controller[rad / (rad / s)]
				array1[16] = array1[16] * 0.5;// 2B90: *0.56;	// 17: Integral gain of pitch controller[rad / rad]
				ReportInfoMessage(turbine_id, "*0.5 on pitch PI prop. gain and *0.5 on int.");
			}
			sprintf_s(reportMessage, 55, "Pitch: Prop. gain = %f, Int. gain = %f", array1[15], array1[16]); ReportInfoMessage(turbine_id, reportMessage);
			array1[17] = 0.0;		// 18: Differential gain of pitch controller[rad / (rad / s ^ 2)]
			array1[18] = 0.40E-08;	// 19: Proportional power error gain [rad/W]
			array1[19] = 0.40E-08;	// 20: Integral power error gain [rad/(Ws)]
			if (useTxtFieldInput) {
				array1[15] = pitchPGain;
				array1[16] = pitchIGain;
				array1[18] = pitchPpowerGain;
				array1[19] = pitchIpowerGain;
			}
			array1[20] = 164.13;	// 21: Coefficient of linear term in aerodynamic gain scheduling, KK1 [deg]
			array1[21] = 702.09;	// 22: Coefficient of quadratic term in aerodynamic gain scheduling, KK2 [deg^2]
									//	 : (if zero, KK1 = pitch angle at double gain)
			array1[22] = 1.3;		// 23: constant 23	; Relative speed for double nonlinear gain [-]
			//	 : Cut-in simulation parameters
			array1[23] = 0;// 0.1;// 0.1414;	// 24: Cut-in time [s], if zero no cut-in simulated (SIMULATION TYPE CHECKED BY BLADED)
			array1[24] = 4.0;		// 25: Time delay for soft start [1/1P]
			//	 : Cut-out simulation parameters
			array1[25] = 30.0;		// 26: Cut-out time [s], if zero no cut-out simulated (SIMULATION TYPE CHECKED BY BLADED)
			array1[26] = 0.5;		// 27: Time constant for 1st order filter lag of torque cut-out [s]
			array1[27] = 1;			// 28: Stop type [1=linear two pitch speed stop, 2=exponential pitch speed stop]
			array1[28] = 1.0;		// 29: Time delay for pitch stop 1 [s]
			array1[29] = 20.0;		// 30: Maximum pitch velocity during stop 1 [deg/s]
			array1[30] = 1.0;		// 31: Time delay for pitch stop 2 [s]
			array1[31] = 10.0;		// 32: Maximum pitch velocity during stop 2 [deg/s]
									//	 : Expert parameters (keep default values unless otherwise given)
			array1[32] = 0.5;		// 33: Lower angle above lowest minimum pitch angle for switch [deg]
			array1[33] = 0.5;		// 34: Upper angle above lowest minimum pitch angle for switch [deg], if equal then hard switch
			array1[34] = 95;		// 35: Ratio between filtered and reference speed for fully open torque limits [%] (WILL BE FACTORED TO [-] BELOW!)
			ReportInfoMessage(turbine_id, "*95% rot. speed gen PI zone.");
			array1[35] = 5.0; // org: 7.0711;	// 36: Time constant of 1st order filter on wind speed used for minimum pitch [1/1P]
			array1[36] = 5.0; // org: 7.0711;	// 37: Time constant of 1st order filter on pitch angle for gain scheduling [1/1P]
			array1[37] = 0;			// 38: Proportional gain of DT damper [Nm/(rad/s)], requires frequency in input 10							  // input 38 NOT GIVEN !!!!!!!
		}

		
		double abs_tip_speed = 100, blade_length_scaling = 1.02064;
		if (use90mpsVersion) {
			abs_tip_speed = 90;
			blade_length_scaling = 1.02133;
		}
		if (!use2B_upscaled) {
			blade_length_scaling = 1;
			ReportInfoMessage(turbine_id, "Using non upscaled version");
		}


		// Overall parameters
		Pe_rated =	array1[0] * 1000 / drive_train_loss; // including power losses of 6% in drivetrain!!
		omega_ref_min =		array1[1];
		if (nmbrBlades == 2) omega_ref_min = array1[1] / blade_length_scaling * abs_tip_speed / 90;
		omega_ref_max =		array1[2];
		if (nmbrBlades == 2) omega_ref_max = array1[2] / blade_length_scaling * abs_tip_speed / 90; // just for 2.06% bigger blades adapted tsr
		max_lss_torque =	array1[3];
		if (nmbrBlades == 2) max_lss_torque = array1[3] * blade_length_scaling / abs_tip_speed * 90;
		minimum_pitch_angle=array1[4] * degrad;
		pitch_stopang =		array1[5] * degrad;
		PID_pit_var.velmax =array1[6] * degrad;
		omega2ordervar.f0 = array1[7];
		omega2ordervar.zeta=array1[8];
		DT_mode_filt.f0 =	array1[9];

		/*Pe_rated = 3524.36 * 1800 / 60 * 2 * pi;	// CART2
		omega_ref_min = 1295/43.165*2*pi/60;		// CART2
		omega_ref_max = 41.7*2*pi/60;				// CART2
		max_lss_torque = 3524.36 * 43.165;			// CART2
		minimum_pitch_angle = 0;					// CART2
		*/

		// Partial load control parameters
		// old: Kopt = array1[10];
		//      if (Kopt* omega_ref_max * omega_ref_max >= Pe_rated / omega_ref_max) Kopt = Pe_rated / pow(omega_ref_max, 3);
		Kopt = fmin(array1[10], Pe_rated / pow(omega_ref_max, 3));																// original Kopt seems to be to low! works better wich calculated one!!!! 
		if (Kopt < Pe_rated / pow(omega_ref_max, 3)) ReportInfoMessage(turbine_id, "Calculated Kopt would have been bigger!");
		Kopt = Pe_rated / pow(omega_ref_max, 3);	 ReportInfoMessage(turbine_id, "Using Calc. Kopt!");						// just for testing the mathematical value !!!!
		/*double rotor_inertia_reduced_by_redesign = 0.5479; // ratio of 2B90 blades mass and 3B blades mass
		ReportInfoMessage(turbine_id, "Reducing Kopt by lower 2B rotor intertia factor of 0.5479!");
		Kopt = Kopt * rotor_inertia_reduced_by_redesign;*/
		PID_gen_var.Kpro = array1[11];
		PID_gen_var.Kint = array1[12];
		PID_gen_var.Kdif = array1[13];

		// Full load control parameters
		const_power = (int(array1[14]) == 1);
		PID_pit_var.Kpro[0] = array1[15];
		PID_pit_var.Kint[0] = array1[16];
		PID_pit_var.Kdif[0] = array1[17];
		PID_pit_var.Kpro[1] = array1[18];
		PID_pit_var.Kint[1] = array1[19];
		PID_pit_var.Kdif[1] = 0.0;
		kk1 =		array1[20] * degrad;
		kk2 =		array1[21] * degrad*degrad;
		rel_limit = array1[22];

		// Cut - in simulation parameters
		if (GetControllerState(turbine_id) == 3) {		// -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP )
			t_cutin = array1[23];
			ReportInfoMessage(turbine_id, "Cut in simulation initiated");
		}
		else t_cutin = 0;
		t_cutin_delay = array1[24] * 2.0 * pi / omega_ref_max;

		// Cut - out simulation parameters
		/*if (GetControllerState(turbine_id) == 4) {		// -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP )
			t_cutout = array1[25];
			ReportInfoMessage(turbine_id, "Cut out simulation initiated");
		}
		else t_cutout =	0;*/
		t_cutout = 0; // t_cutout is triggered by a Bladed control function later in the code
		torquefirstordervar.tau = array1[26];
		pitch_stoptype =int(array1[27]);
		pitch_stopdelay =	array1[28];
		pitch_stopvelmax =	array1[29] * degrad;
		pitch_stopdelay2 =	array1[30];
		pitch_stopvelmax2 = array1[31] * degrad;

		// Expert parameters(keep default values unless otherwise given)
		switch1_pitang_lower =	array1[32] * degrad;
		switch1_pitang_lower1 = switch1_pitang_lower;
		switch1_pitang_upper =	array1[33] * degrad;
		switch1_pitang_upper1 = switch1_pitang_upper;
		rel_sp_open_Qg = array1[34] *0.01; // factor 0.01 because array1[34] is set in %
		wspfirstordervar.tau =	array1[35] * 2.0 * pi / omega_ref_max;
		ReportInfoMessage(turbine_id, "scaling of the wsp filter tau should be checked!");
		ReportInfoMessage(turbine_id, "Why does the filter change with rpm in the first place?");
		pitchfirstordervar.tau =array1[36] * 2.0 * pi / omega_ref_max;

		// Drivetrain damper
		DT_damp_gain = array1[37];
		DT_damper_filt.f0 = DT_mode_filt.f0;
		pwr_DT_mode_filt.f0 = DT_mode_filt.f0;

		// Default and derived parameters
		PID_gen_var.velmax = 0.0; // No limit to generator torque change rate
		Qg_rated = Pe_rated / omega_ref_max;
		switchfirstordervar.tau = 2.0 * pi / omega_ref_max;
		cutinfirstordervar.tau  = 2.0 * pi / omega_ref_max;


		// Wind speed table
		if (abs(minimum_pitch_angle) < 90.0 * degrad)
		{	// minimal pitch angle for all wind speeds, but why???
			OPdatavar.lines = 2;
			OPdatavar.wpdata[0][0] = 0.0;
			OPdatavar.wpdata[1][0] = 99.0;
			OPdatavar.wpdata[0][1] = minimum_pitch_angle;
			OPdatavar.wpdata[1][1] = minimum_pitch_angle;
		} /*else
			{

			// i guess it simply writes the angle data in text32 ??? //- write(text32, ’(i)’) int(minimum_pitch_angle*raddeg)
			sprintf(text32, "%i", int(minimum_pitch_angle*raddeg));
			ReportInfoMessage(turbine_id, text32);
			char* fileName;
			sprintf(fileName, "wpdata. %s", text32);
			// checks if file exists (if it does findes = TRUE)...	// - inquire(file = fileName, exist=findes)
			if (findes) // bool
			{
			open(88, file = fileName);
			read(88, *, iostat = ifejl) OPdatavar.lines;
			if (ifejl = 0)
			{
			do i = 1, OPdatavar.lines
			{
			read(88, *, iostat = ifejl) OPdatavar.wpdata(i, 1), OPdatavar.wpdata(i, 2);
			if (ifejl != 0)
			{
			write(6, *) ’ *** ERROR *** Could not read lines in minimum ’&
			//’pitch table in file wpdata.’//trim(adjustl(text32))
			exit(0);
			}
			OPdatavar.wpdata(i, 2) = OPdatavar.wpdata(i, 2)*degrad;
			}
			}
			else
			{
			write(6, *) ’ *** ERROR *** Could not read number of lines ’&
			//’in minimum pitch table in file wpdata.’//trim(adjustl(text32))
			exit(0)
			}
			close(88)
			}
			else
			{
			write(6, *) ’ *** ERROR *** File ’’wpdata.’//trim(adjustl(text32))&
			//’’’ does not exist in the working directory’
			exit(0);
			}
			} */

		// Just a fast work around (parameter of CART2)
		// Wind speeds in m/s
		OPdatavar.lines = 13;
		OPdatavar.wpdata[0][0] = 0;
		OPdatavar.wpdata[1][0] = 11.4; //  13.6;
		OPdatavar.wpdata[2][0] = 11.5; // 13.8;
		OPdatavar.wpdata[3][0] = 11.7; // 14;
		OPdatavar.wpdata[4][0] = 12.0; // 14.2;
		OPdatavar.wpdata[5][0] = 12.5; // 14.4;
		OPdatavar.wpdata[6][0] = 13.0; // 14.8;
		OPdatavar.wpdata[7][0] = 14.0; // 15.4;
		OPdatavar.wpdata[8][0] = 15.0; // 16.2;
		OPdatavar.wpdata[9][0] = 16.0; // 17.2;
		OPdatavar.wpdata[10][0] = 18.0; // 18.4;
		OPdatavar.wpdata[11][0] = 21.0; // 19.8;
		OPdatavar.wpdata[12][0] = 24.9; // 21.4;
		//OPdatavar.wpdata[13][0] = 23.2;
		//OPdatavar.wpdata[14][0] = 24.8;
		// Pitch angles in rad
		OPdatavar.wpdata[0][1] = 0;
		OPdatavar.wpdata[1][1] = 0;
		OPdatavar.wpdata[2][1] = 0.027001; // 0.034229;
		OPdatavar.wpdata[3][1] = 0.049512; // 0.058098;
		OPdatavar.wpdata[4][1] = 0.07189; // 0.075184;
		OPdatavar.wpdata[5][1] = 0.099601; // 0.089157;
		OPdatavar.wpdata[6][1] = 0.121752; // 0.111914;
		OPdatavar.wpdata[7][1] = 0.158429; // 0.138923;
		OPdatavar.wpdata[8][1] = 0.189587; // 0.168097;
		OPdatavar.wpdata[9][1] = 0.217406; // 0.198883;
		OPdatavar.wpdata[10][1] = 0.267104; // 0.231024;
		OPdatavar.wpdata[11][1] = 0.33237; // 0.264457;
		OPdatavar.wpdata[12][1] = 0.407358; // 0.299239;
		//OPdatavar.wpdata[13][1] = 0.335365;
		//OPdatavar.wpdata[14][1] = 0.36559;

		//Initiate the dynamic variables
		stepno = 0;
		time_old = 0.0;
		deltat = 0.01;

		//No output
		// return; } //- end subroutine init_regulation
		// !**************************************************************************************************! //




		/*
		sprintf_s(LogText,"init Tow Acce:%f",  GetMeasuredTowerTopForeAftAcceleration( turbine_id ) * 0.01);
		ReportInfoMessage(turbine_id, LogText);

		sprintf_s(LogText,"init LSS RPM:%f",  GetMeasuredRotorSpeed(turbine_id )    *30/pi);
		ReportInfoMessage(turbine_id, LogText);

		//---- export initial control values to bladed ---- //
		SetDemandedGeneratorTorque ( turbine_id ,  MAXTORQUE  ) ; // rated torque in Nm
		SetDemandedPitchAngle ( turbine_id ,  0 ,  -1*PI/180  ) ; // bladed pitch at rated in rad
		SetDemandedPitchAngle ( turbine_id ,  1 ,  -1*PI/180  ) ;
		*/
		// ---- logging contoller outputs/data ---- //
		
		AddLogValue  ( turbine_id, "Pe_ref"	, "W");					// 0 : Power reference[W]
		AddLogValue  ( turbine_id, "WSPfilt" , "m/s");				// 1 : Filtered wind speed[m / s]
		AddLogValue  ( turbine_id, "omegafilt", "rad/s");			// 2 : Filtered rotor speed[rad / s]
		AddLogValue  ( turbine_id, "omega_err_filt_speed","rad/s");	// 3 : Filtered rotor speed error for torque[rad / s]
		AddLogValue  ( turbine_id, "omega_dtfilt", "rad/s");		// 4 : Bandpass filtered rotor speed[rad / s]
		AddLogValue  ( turbine_id, "PID_gen_var.outpro", "Nm");		// 5 : Proportional term of torque contr.[Nm]
		AddLogValue  ( turbine_id, "PID_gen_var.outset", "Nm");		// 6 : Integral term of torque controller[Nm]
		AddLogValue  ( turbine_id, "PID_gen_var.outmin", "Nm");		// 7 : Minimum limit of torque[Nm]
		AddLogValue  ( turbine_id, "PID_gen_var.outmax", "Nm");		// 8 : Maximum limit of torque[Nm]
		AddLogValue  ( turbine_id, "switch1", "-");					// 9 : Torque limit switch based on pitch[-]
		AddLogValue  ( turbine_id, "omega_err_filt_pitch", "rad/s");// 10 : Filtered rotor speed error for pitch[rad / s]
		AddLogValue  ( turbine_id, "e_pitch[1]", "W");				// 11 : Power error for pitch[W]
		AddLogValue  ( turbine_id, "PID_pit_var.outpro", "rad");	// 12 : Proportional term of pitch controller[rad]
		AddLogValue  ( turbine_id, "PID_pit_var.outset", "rad");	// 13 : Integral term of pitch controller[rad]
		AddLogValue  ( turbine_id, "PID_pit_var.outmin", "rad");	// 14 : Minimum limit of pitch[rad]
		AddLogValue  ( turbine_id, "PID_pit_var.outmax", "rad");	// 15 : Maximum limit of pitch[rad]
		AddLogValue  ( turbine_id, "pitch_actuator_duty_cycle", "deg");	// 16 : pitch actuator duty cycle
		AddLogValue  ( turbine_id, "final_pitchPGain", "-");		// 17 : pitch gain in controller (after gain scheduling)
		AddLogValue	 ( turbine_id, "final_pitchIGain", "-");		// 18 : pitch gain in controller
		AddLogValue  ( turbine_id, "final_pitchPGain_power", "-");	// 19 : pitch gain in controller
		AddLogValue	 ( turbine_id, "final_pitchIGain_power", "-");	// 20 : pitch gain in controller
		AddLogValue  ( turbine_id, "pitch_gain_schedule", "-");// 21 : pitch gain scheduling in controller
		AddLogValue  ( turbine_id, "pitch_gain_schedule_power", "-");// 22 : pitch gain scheduling in controller
		AddLogValue	 (turbine_id, "meanpitangfilt", "rad");			// 23 : for various tests
		AddLogValue  (turbine_id, "switch1_pitang_upper", "rad");	// 24 : for various tests
		AddLogValue	 (turbine_id, "switch1_pitang_lower", "rad");	// 25 : for various tests
		AddLogValue	 (turbine_id, "Qg_min_partial", "Nm");			// 26 : for various tests
		AddLogValue  (turbine_id, "Qg_max_partial", "Nm");			// 27 : for various tests
		/*AddLogValue  (turbine_id, "ex zone check one", "-");		// 28 : for various tests
		AddLogValue  (turbine_id, "ex zone check two", "-");		// 29 : for various tests
		AddLogValue  (turbine_id, "ex zone sign", "-");				// 30 : for various tests
		AddLogValue  (turbine_id, "exZone_spline", "-");			// 31 : for various tests
		AddLogValue  (turbine_id, "ex zone check three", "-");		// 32 : for various tests
		AddLogValue  (turbine_id, "torque compensator", "Nm");		// 33 : for various tests
		AddLogValue  (turbine_id, "spline1", "-");					// 34 : for various tests
		AddLogValue  (turbine_id, "spline2", "-");					// 35 : for various tests
		AddLogValue  (turbine_id, "Qdamp", "-");					// 36 : for various tests
		AddLogValue(turbine_id, "trigger_ex_escape check", "-");	// 33 : for various tests*/
		
		/*AddLogValue(turbine_id, "Tower SS Accel", "-");				// 34 : for various tests
		AddLogValue(turbine_id, "Tower SS Accel Notch Filt", "-");	// 35 : for various tests
		AddLogValue(turbine_id, "Tower SS Accel Velo", "-");		// 36 : for various tests
		AddLogValue(turbine_id, "generator_cutin", "-");			// 22 : for various tests
		AddLogValue(turbine_id, "check_controller_state", "-");		// 22 : for various tests
		
		AddLogValue(turbine_id, "GetGeneratorContactor", "-");
		AddLogValue(turbine_id, "GetLastErrorCode", "-");
		AddLogValue(turbine_id, "GetMeasuredGeneratorTorque", "-");
		AddLogValue(turbine_id, "GridlossDetected", "-");
		AddLogValue(turbine_id, "IsPitchLimitSwitchTripped", "-");
		AddLogValue(turbine_id, "switch_value_cut_out_pitch", "-");
		AddLogValue(turbine_id, "t_cut_out", "-");
		AddLogValue(turbine_id, "GetShaftBrakeStatusBinaryFlag ", "-");
		AddLogValue(turbine_id, "PID_pit_var.velmax", "rad/s");
		AddLogValue(turbine_id, "PID_pit_var.velmax check", "rad/s");
		AddLogValue(turbine_id, "PID_pit_var.outres1_old", "rad");
		*/
		
		
		//AddLogValue(turbine_id, "the XX", "-");						// 22 : for various tests

		// initial state
		//nominalWindSpeed = GetNominalHubFlowSpeed(turbine_id);
		//PID_pit_var.outset1 = GetOptiPitch(nominalWindSpeed);
		// theta_min = GetOptiPitch(nominalWindSpeed);
// for (int i = 0; i < 2; i++) pitang[i] = SetDemandedPitchAngle(turbine_id, 0, theta_min);


return CONTROLLER(turbine_id);
	}



	/*! Controller to be run on subsequent timesteps. */
	int __declspec(dllexport) __cdecl CONTROLLER(const turbine turbine_id)
	{
		//- subroutine update_regulation(array1, array2) //- implicit none
		// DEC$ ATTRIBUTES DLLEXPORT, C, ALIAS:’update_regulation’::update_regulation

		//	    Input array1 must contain
		//  1 : general time; [s]
		//  2 : constraint bearing1 shaft_rot 1 only 2; [rad / s] Generator LSS speed
		//  3 : constraint bearing2 pitch1 1 only 1; [rad]
		//  4 : constraint bearing2 pitch2 1 only 1; [rad]
		//  5 : constraint bearing2 pitch3 1 only 1; [rad]
		//  6 - 8: wind free_wind 1 0.0 0.0 hub height; [m / s] global coords at hub height             // difference between 6, 7, 8 ???? Windspeed for controller (wsp) seems to be sqrt(array[6]^2 + array[7]^2)
		// 
		//      Output array2 contains:
		//  1 : Generator torque reference[Nm]
		//  2 : Pitch angle reference of blade 1[rad]
		//  3 : Pitch angle reference of blade 2[rad]
		//  4 : Pitch angle reference of blade 3[rad]
		//  5 : Power reference[W]
		//  6 : Filtered wind speed[m / s]
		//  7 : Filtered rotor speed[rad / s]
		//  8 : Filtered rotor speed error for torque[rad / s]
		//  9 : Bandpass filtered rotor speed[rad / s]
		// 10 : Proportional term of torque contr.[Nm]
		// 11 : Integral term of torque controller[Nm]
		// 12 : Minimum limit of torque[Nm]
		// 13 : Maximum limit of torque[Nm]
		// 14 : Torque limit switch based on pitch[-]
		// 15 : Filtered rotor speed error for pitch[rad / s]
		// 16 : Power error for pitch[W]
		// 17 : Proportional term of pitch controller[rad]
		// 18 : Integral term of pitch controller[rad]
		// 19 : Minimum limit of pitch[rad]
		// 20 : Maximum limit of pitch[rad]
		// 21 : Torque reference from DT damper[Nm]		

		//**************************************************************************************************//
		//                Increment time step(may actually not be necessary in type2 DLLs)					//
		//**************************************************************************************************//

		time = GetSimulationCurrentTime(turbine_id);	// Inp 1 // Returns the current time in the simulation in s. 
		if (time > time_old) {
			deltat = time - time_old;
			time_old = time;
			stepno = stepno + 1;
		}

		//**************************************************************************************************//
		//										Inputs and their filtering									//
		//**************************************************************************************************//
		omega = GetMeasuredRotorSpeed(turbine_id);		 // Inp 2 // Returns the measured rotor speed in rad/s. (NOT GENERATOR HSS!!!)
		//Mean pitch angle
		pitang[0] = GetMeasuredPitchAngle(turbine_id, 0); // Inp 3 // Returns the current pitch angle of the blade in rad.
		pitang[1] = GetMeasuredPitchAngle(turbine_id, 1); // Inp 4
		if (nmbrBlades == 3) {
			pitang[2] = GetMeasuredPitchAngle(turbine_id, 2); // Inp 5
			meanpitang = (pitang[0] + pitang[1] + pitang[2]) / 3.0;
			AllPitchLimitSwitchTripped = IsPitchLimitSwitchTripped(turbine_id, 0) + IsPitchLimitSwitchTripped(turbine_id, 1) + IsPitchLimitSwitchTripped(turbine_id, 2);
		}
		else {
			meanpitang = (pitang[0] + pitang[1]) / 2.0;
			AllPitchLimitSwitchTripped = IsPitchLimitSwitchTripped(turbine_id, 0) + IsPitchLimitSwitchTripped(turbine_id, 1);
		}
		pitchADC = pitchADC + abs(meanpitang - oldMeanpitang);
		oldMeanpitang = meanpitang;

		//################# tower FA damping filt ##################################
		//double NacelleForeAftAccel = GetMeasuredNacelleAccelerometerAccelerationX(turbine_id, 0);
		double Tower_FA_Accel = GetMeasuredTowerTopForeAftAcceleration(turbine_id);
		tower_FA_velo = tower_FA_velo + Tower_FA_Accel * deltat;

		Tower_FA_Accel_filt.zeta1 = 0.1 * 12; // *6;
		Tower_FA_Accel_filt.zeta2 = 0.001 * 12; // 6;
		Tower_FA_Accel_filt.f0 = 0.195;
		Tower_FA_Accel_filt = notch2orderfilt(deltat, stepno, Tower_FA_Accel_filt, Tower_FA_Accel);
		double Nacelle_FA_Bandpass_filt = Tower_FA_Accel - Tower_FA_Accel_filt.y1;

		

		//################# tower SS damping filt ##################################
		double Tower_SS_Accel = GetMeasuredTowerTopSideSideAcceleration(turbine_id);

		Tower_SS_Accel_filt.zeta1 = 0.1 * 12; // *6;
		Tower_SS_Accel_filt.zeta2 = 0.001 * 12; // 6;
		Tower_SS_Accel_filt.f0 = 0.195;

		Tower_SS_Accel_filt = notch2orderfilt(deltat, stepno, Tower_SS_Accel_filt, Tower_SS_Accel);


		double Nacelle_SS_Bandpass_filt = Tower_SS_Accel - Tower_SS_Accel_filt.y1;
		tower_SS_velo = tower_SS_velo + Tower_SS_Accel * deltat;

		/*SetLoggingValue(turbine_id, 29, Tower_SS_Accel_filt.y1);
		SetLoggingValue(turbine_id, 33, Tower_SS_Accel);
		SetLoggingValue(turbine_id, 34, Tower_SS_Accel_filt.y1);
		SetLoggingValue(turbine_id, 35, tower_SS_velo);*/


		//################# tower damping filt ##################################


		// Wind speed as horizontal vector sum
		nominalWindSpeed = GetNominalHubFlowSpeed(turbine_id); // Inp 6.1 // Returns the modelled speed of the flow over the hub. 
		 // This is based on free flow at hub position - there is no modelling of actual nacelle anemometer or pitot tube. Measured in m/s.
		wsp = sqrt(nominalWindSpeed * nominalWindSpeed + SecondWindInput * SecondWindInput);
		// Low - pass filtering of the rotor speed
		//-- y = lowpass2orderfilt(deltat, stepno, omega2ordervar, omega);			// array as function output is not possible in c++!!!
		omega2ordervar = lowpass2orderfiltFirstVal(deltat, stepno, omega2ordervar, omega);
		omegafilt = omega2ordervar.y1;
		//y[1] = getLowpass2orderfiltSecondVal(deltat, omega2ordervar, y[0]);						// has to follow lowpass2orderfiltFirstVal!!!
		//y[1] = 0.5 * (y[0] - omega2ordervar.y2_old) / deltat;	// isnt value somewhat useless???	// has to follow lowpass2orderfiltFirstVal!!!
		//domega_dt_filt = y[1];								// isnt value somewhat useless???



		/*// ########################## COPIED FROM EXZONE ###############################
		// notchfilter exclusion zone rotor speed???
		Ex_mode_filt.zeta1 = 0.1 * 12; // *6;
		Ex_mode_filt.zeta2 = 0.001 * 12; // 6;
		Ex_mode_filt.f0 = 0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
								//if (nmbrBlades == 2)	Ex_mode_filt.f0 = 0.124;//  0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
		//if (use90mpsVersion != 1) Ex_mode_filt.f0 = 0.124;
		Ex_mode_filt = notch2orderfilt(deltat, stepno, Ex_mode_filt, omegafilt);
		SetLoggingValue(turbine_id, 29, Ex_mode_filt.y1);

		Ex_mode_filt2.f0 = 0.170; //0.179 // Hz value of filter notch filtered frequency
		Ex_mode_filt2 = notch2orderfilt(deltat, stepno, Ex_mode_filt2, omegafilt);
		SetLoggingValue(turbine_id, 30, Ex_mode_filt2.y1);



		double omega4exZone = Ex_mode_filt.y1; // omegafilt; // omega or omegafilt ???
											   //double omega4exZone = omegafilt;

		omegafilt = Ex_mode_filt.y1;
		// ########################## COPIED FROM EXZONE ###############################

		*/


		//int generator_cutin_check = GetGeneratorContactor(turbine_id);		// Returns the generator contactor: 0=OFF, 1=MAIN, 2=LOW SPEED. 
		int controller_state = GetControllerState(turbine_id);		// -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP 
																	// Cut - out simulation parameters


		if ((GetControllerState(turbine_id) == 4 || GetControllerState(turbine_id) == 5) && t_cutout == 0) { // -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP )
			t_cutout = time;
			ReportInfoMessage(turbine_id, "Cut out simulation initiated");
		}
		else if (GetControllerState(turbine_id) == 0 && GridlossDetected == 0 && GetGeneratorContactor(turbine_id) == 0) { // Returns the generator contactor: 0=OFF, 1=MAIN, 2=LOW SPEED. 
			GridlossDetected = 1;
			t_cutout = time + 5; // +20;
			omega_ref_max = omegafilt * 0.9;
			ReportInfoMessage(turbine_id, "Grit loss is detected and cut out initiated");
			ReportInfoMessage(turbine_id, "pitch by PID for 7s, then smooth pitching out");
			PID_pit_var.Kpro[0] = PID_pit_var.Kpro[0] * 2;
			//PID_pit_var.Kint[0] = PID_pit_var.Kint[0] * 2;
			ReportInfoMessage(turbine_id, "increasing proportional pitch gain by factor 2");
		}
		else if (GetControllerState(turbine_id) == 0 && StuckedPitchDetected == 0 && AllPitchLimitSwitchTripped > 0 ) {
			//IsPitchLimitSwitchTripped(turbine_id, 0) + IsPitchLimitSwitchTripped(turbine_id, 1) + IsPitchLimitSwitchTripped(turbine_id, 2) > 0) {
			StuckedPitchDetected = 1;
			t_cutout = time;
			ReportInfoMessage(turbine_id, "Stucked Pitch is detected and cut out initiated");
		}

		
		if (omegafilt > omega_ref_max*1.08) {
			if (omegafilt > omega_ref_max*maxOverspeedFactor && t_cutout == 0) {
				t_cutout = time - t_cutout_delay / 3;
				//ReportInfoMessage(turbine_id, "Safety Shut Down! omega>n_r*1.12");
				char *reportMessage = new char[55];// [3];
				sprintf_s(reportMessage, 55, "Safety Shut Down! omega>n_r*%1.3f", maxOverspeedFactor);	ReportInfoMessage(turbine_id, reportMessage);
			}
			if (omegafilt > omega_ref_max*1.20) {//&& GetShaftBrakeStatusBinaryFlag(turbine_id) != 31) { // CHECK THE TRIGGER!!!! WHY DAS GetShaftBrakeStatusBinaryFlag != 31 NOT WORK?
				//ReportInfoMessage(turbine_id, "Emergency Shut Down! omega>n_r*1.20");
				SetShaftBrakeStatusBinaryFlag(turbine_id, 31);
			}
		}

		
		int ControllerFalureFlag = GetControllerFailureFlag(turbine_id);

		
		
		
		
		
																	// Mean pitch angle
		// Low - pass filtering of the mean pitch angle for gain scheduling
		pitchfirstordervar = lowpass1orderfilt(deltat, stepno, pitchfirstordervar, meanpitang);
		meanpitangfilt = fmin(pitchfirstordervar.y1, 30.0* degrad);
		// Low - pass filtering of the nacelle wind speed
		wspfirstordervar = lowpass1orderfilt(deltat, stepno, wspfirstordervar, wsp);
		WSPfilt = wspfirstordervar.y1;


		// Minimum pitch angle may vary with filtered wind speed
		theta_min = 0; // GetOptiPitch(WSPfilt);
		if (stepno == 1) {
			ReportInfoMessage(turbine_id, "Manually setting theta_min=0 to avoid pitch limitation!");		//																	theta min always to high????
			ReportInfoMessage(turbine_id, "Ignoring theta_min for pitch switch1!");
		}
		/*switch1_pitang_upper = switch1_pitang_upper + theta_min;
		switch1_pitang_lower = switch1_pitang_lower + theta_min;
		// Alternative:
		switch1_pitang_upper = switch1_pitang_upper1 + theta_min;
		switch1_pitang_lower = switch1_pitang_lower1 + theta_min;*/
		//**************************************************************************************************//
		//  Speed ref.changes max. <->min. for torque controller and remains at rated for pitch controller  //
		//**************************************************************************************************//
		if (omegafilt > 0.5 * (omega_ref_max + omega_ref_min))		// Comm.: In the middle of min and max rotor speed, no speed error is used for the generator PI-controller.
			omega_err_filt_speed = omegafilt - omega_ref_max;		//		  Yet, close to min or max rot. speed, the gen. controller uses a PI function to hold the rotor speed close to these values.
		else omega_err_filt_speed = omegafilt - omega_ref_min;

		//**************************************************************************************************
		//PID regulation of generator torque
		//**************************************************************************************************
		//Limits for full load
		if (const_power)
		{
			Qg_min_full = fmin(Pe_rated / fmax(omega, 1.0E-15), max_lss_torque);
			Qg_max_full = Qg_min_full;
		}
		else
		{
			Qg_min_full = Qg_rated;
			Qg_max_full = Qg_rated;
		}

		// Limits for partial load that opens in both ends
		ommin1 = omega_ref_min;
		ommin2 = omega_ref_min / rel_sp_open_Qg;
		ommax1 = (2.0 * rel_sp_open_Qg - 1.0)*omega_ref_max;
		ommax2 = rel_sp_open_Qg * omega_ref_max;
		x = switch_spline(omegafilt, ommin1, ommin2);
		Qg_min_partial = fmin(Kopt*omegafilt*omegafilt * x, Kopt*ommax1*ommax1);
		x = switch_spline(omegafilt, ommax1, ommax2);
		Qg_max_partial = fmax(Kopt*omegafilt*omegafilt * (1.0 - x) + Qg_max_full * x, Kopt*ommin2*ommin2);


		// ########################################################################################
		// ############################# SPEED EXCLUSION ZONE #####################################
		// ########################################################################################
		
		// notchfilter exclusion zone rotor speed???
		Ex_mode_filt.zeta1 = 0.1 * 12; // *6;
		Ex_mode_filt.zeta2 = 0.001 * 12; // 6;
		Ex_mode_filt.f0 = 0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
								//if (nmbrBlades == 2)	Ex_mode_filt.f0 = 0.124;//  0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
								//if (use90mpsVersion != 1) Ex_mode_filt.f0 = 0.124;
		Ex_mode_filt = notch2orderfilt(deltat, stepno, Ex_mode_filt, omegafilt);
		//SetLoggingValue(turbine_id, 29, Ex_mode_filt.y1);

		Ex_mode_filt2.f0 = 0.170; //0.179 // Hz value of filter notch filtered frequency
		Ex_mode_filt2 = notch2orderfilt(deltat, stepno, Ex_mode_filt2, omegafilt);
		//SetLoggingValue(turbine_id, 30, Ex_mode_filt2.y1);



		double omega4exZone = Ex_mode_filt.y1; // omegafilt; // omega or omegafilt ???
											   //double omega4exZone = omegafilt;

		//omegafilt = Ex_mode_filt.y1;
		

		// ############################################### ENTER THE TOWER EIGENFREQUENCY ##############################################//
		// ????? is Side-Side f = 0.16 Hz and fore-aft f = 0.17 Hz ??????
		//double omega_tower_eigenfrequency_interaction = 0.162 /  nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
		//double omega_tower_eigenfrequency_interaction = 0.17 / nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
		double omega_tower_eigenfrequency_interaction = 0.165 / nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
		if (nmbrBlades == 2) {
			if (use90mpsVersion) {
					omega_tower_eigenfrequency_interaction = 0.170 / nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
			} else	omega_tower_eigenfrequency_interaction = 0.152 / nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
			
		}
		//if (nmbrBlades == 2) omega_tower_eigenfrequency_interaction = 0.172 / nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
		// ############################################### ENTER THE TOWER EIGENFREQUENCY ##############################################//
		double rel_omega_exclusion_zone = 0.15; // 0.15; //0.10; // in % below / above ( note, that fore-aft and side-side are already 3% away from middle value)
		double check_one = 0, check_two = 0;
		double exZoneSwitchWidth = 0.05; // in %
		double exZone_Torque_limit_increase = 0.30; // increases torque limit in %

		double omega_start_lower_exzone = omega_tower_eigenfrequency_interaction * (1 - (rel_omega_exclusion_zone + exZoneSwitchWidth));
		double omega_start_upper_exzone = omega_tower_eigenfrequency_interaction * (1 + (rel_omega_exclusion_zone + exZoneSwitchWidth));

		double omega_saturated_lower_exzone = omega_tower_eigenfrequency_interaction * (1 - rel_omega_exclusion_zone);
		double omega_saturated_upper_exzone = omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone);

		double lower_switch_exZone_torque = Kopt * pow(omega_start_lower_exzone, 2);
		double upper_switch_exZone_torque = Kopt * pow(omega_start_upper_exzone, 2);

		//double lower_exZone_torque		 = Kopt * pow(omega_saturated_lower_exzone, 2);
		double lower_exZone_torque_limit = Kopt * pow(omega_saturated_lower_exzone, 2) * (1 - exZone_Torque_limit_increase);
		//double upper_exZone_torque		 = Kopt * pow(omega_saturated_upper_exzone, 2);
		double upper_exZone_torque_limit = Kopt * pow(omega_saturated_upper_exzone, 2) * (1 + exZone_Torque_limit_increase);
		//double torque_matching_eigenfrequency = Kopt * pow(omega_tower_eigenfrequency_interaction, 2); //needed?

		double max_torque_slope4_transition = 288000; // 6000 * GearBoxRatio; // 6000; // maximum Nm/s gearless generator torque for transition to get out of the exclusion zone
		double Demand_Speed_change_Ex_zone_Exit_rate = 0.2 / 5 * 2 * pi / 60;   // Change rate of rotation speed demanded to exit the exclusion zone; if to low, torque is triggered to be more agressive



		check_three = 0;
		/*if (useSpeedExZone==2) {

			// Checking if rotation speed is inside the outer exclusion zone, remembering entrance direction and manipulating speed error for constant rotation speed
			if (abs(omega4exZone - omega_tower_eigenfrequency_interaction) < omega_tower_eigenfrequency_interaction*(rel_omega_exclusion_zone + exZoneSwitchWidth) * 2) {
				if (stepno != pred_stepno) {
					ReportInfoMessage(turbine_id, "Close to Exclusion zone!");
					//ReportInfoMessage(turbine_id, "using omega unfiltered!");
					ex_zone_sign = copysign(1.0, omega4exZone - omega_tower_eigenfrequency_interaction);
				}
				pred_stepno = stepno + 1;
				check_three = 8;
				//omega_err_filt_speed = omega4exZone - omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign);
			}

			// using three splines with actual omega as trigger between two values. Explanation for comming from beneath exclusion zone (from above is reziprogual) 
			// 1.) increase torque with spline for constant rotation speed, until maximum zone torque is reached
			// 2.) excelerate rotation speed by vastly reducing torque
			// 3.) catch up with actual torque value
			if ((abs(omega4exZone - omega_tower_eigenfrequency_interaction) < omega_tower_eigenfrequency_interaction*(rel_omega_exclusion_zone + exZoneSwitchWidth))) {
				
				if (ex_zone_sign < 0) {
					lower_exZone_torque_limit = Kopt * pow(omega_saturated_upper_exzone, 2);
					//upper_exZone_torque_limit = Kopt * pow(omega_saturated_upper_exzone, 2) * (1 + exZone_Torque_limit_increase);
				} else {
					//lower_exZone_torque_limit = Kopt * pow(omega_saturated_lower_exzone, 2) * (1 - exZone_Torque_limit_increase);
					upper_exZone_torque_limit = Kopt * pow(omega_saturated_lower_exzone, 2);
				}
				//double lower_exZone_torque_limit = Kopt * pow(omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone ), 2) * (1 - exZone_Torque_limit_increase * (1 + ex_zone_sign) / 2); // torque limit part gets ignored depending on ex zone sign
				//double upper_exZone_torque_limit = Kopt * pow(omega_saturated_upper_exzone, 2) * (1 + exZone_Torque_limit_increase * (1 - ex_zone_sign)/2);

				if (omega4exZone <= omega_saturated_lower_exzone) {
					if (ex_zone_sign > 0) upper_exZone_torque_limit = last_Qg_min_partial;
					x = switch_spline(omega4exZone, omega_start_lower_exzone, omega_saturated_lower_exzone);
					Qg_min_partial = lower_switch_exZone_torque * (1 - x) + upper_exZone_torque_limit * x;
					check_three = 1;
					torque_rate_limit_compensator = 0;
				}

				else if (omega4exZone <= omega_saturated_upper_exzone) {
					x = switch_spline(omega4exZone, omega_saturated_lower_exzone, omega_saturated_upper_exzone);
					Qg_min_partial = upper_exZone_torque_limit * (1 - x) + lower_exZone_torque_limit * x - ex_zone_sign * torque_rate_limit_compensator;// - torque_rate_limit_compensator;
					
					if ((abs(last_Qg_min_partial - Qg_min_partial) > max_torque_slope4_transition * deltat) && (check_three == 3 || check_three ==2)) {
						
						torque_rate_limit_compensator = torque_rate_limit_compensator + abs(last_Qg_min_partial - Qg_min_partial) - max_torque_slope4_transition * deltat;
						Qg_min_partial = last_Qg_min_partial + ex_zone_sign * max_torque_slope4_transition * deltat;
						
						check_three = 3;
					} else check_three = 2;
					last_Qg_min_partial = Qg_min_partial;
				}

				else if (omega4exZone <= omega_start_upper_exzone) {
					if (ex_zone_sign < 0) lower_exZone_torque_limit = last_Qg_min_partial;
					x = switch_spline(omega4exZone, omega_saturated_upper_exzone, omega_start_upper_exzone);
					Qg_min_partial = upper_switch_exZone_torque * x + lower_exZone_torque_limit * (1 - x);
					check_three = 4;
					torque_rate_limit_compensator = 0;
				}
				else	ReportInfoMessage(turbine_id, "Something is rotten in the state of denmark!");

				Qg_max_partial = Qg_min_partial;
			}
		}*/




	// Alternate Method:
		if (useSpeedExZone) {

			// Checking if rotation speed is inside the outer exclusion zone, remembering entrance direction and manipulating speed error for constant rotation speed
			if (abs(omega4exZone - omega_tower_eigenfrequency_interaction) < omega_tower_eigenfrequency_interaction*(rel_omega_exclusion_zone + exZoneSwitchWidth * 2)) {
				if (stepno != pred_stepno) {
					//ReportInfoMessage(turbine_id, "using omega unfiltered!");
					ex_zone_sign = copysign(1.0, omega4exZone - omega_tower_eigenfrequency_interaction);
					if (ex_zone_sign>0)	ReportInfoMessage(turbine_id, "Close above the Exclusion zone!");
					else ReportInfoMessage(turbine_id, "Close below the Exclusion zone!");
				}
				pred_stepno = stepno + 1;
				check_one = 1;
				//omega_err_filt_speed = omega4exZone - omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign);
				omega_err_filt_speed = (omega4exZone - omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign)) * 4; // *5;
				// omega_err_filt_speed = (pow(omega4exZone, 2) - pow(omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign), 2)) * 10;
			}

			// using three splines with actual omega as trigger between two values. Explanation for comming from beneath exclusion zone (from above is reziprogual) 
			// 1.) increase torque with spline for constant rotation speed, until maximum zone torque is reached
			// 2.) excelerate rotation speed by vastly reducing torque
			// 3.) catch up with actual torque value
			if (abs(omega4exZone - omega_tower_eigenfrequency_interaction) < omega_tower_eigenfrequency_interaction*(rel_omega_exclusion_zone + exZoneSwitchWidth * (1 + trigger_ex_entrance)) ) {
				trigger_ex_entrance = 1;
				if (ex_zone_sign < 0) {
					lower_exZone_torque_limit = Kopt * pow(omega_saturated_upper_exzone, 2);
					//upper_exZone_torque_limit = Kopt * pow(omega_saturated_upper_exzone, 2) * (1 + exZone_Torque_limit_increase);
				}
				else {
					//lower_exZone_torque_limit = Kopt * pow(omega_saturated_lower_exzone, 2) * (1 - exZone_Torque_limit_increase);
					upper_exZone_torque_limit = Kopt * pow(omega_saturated_lower_exzone, 2);
				}


				// Limits for partial load that gives room for constant speed PI-torque controller in exclusion zone
				if (ex_zone_sign > 0) { // rotor speed entering from above tower eigenfrequency
					//Qg_max_partial =      Kopt * omega_start_upper_exzone*omega_start_upper_exzone;
					/*Qg_max_partial = fmax(Kopt * omega_start_upper_exzone*omega_start_upper_exzone, Kopt * omega4exZone * omega4exZone);
					x = switch_spline(omega4exZone, omega_saturated_upper_exzone, omega_start_upper_exzone);
					Qg_min_partial = Kopt * omega4exZone*omega4exZone * x + lower_exZone_torque_limit * (1.0 - x);*/


					Qg_max_partial = fmax(Kopt * omega_start_upper_exzone*omega_start_upper_exzone, Kopt * omegafilt * omegafilt);

					x = switch_spline(omegafilt, omega_saturated_upper_exzone, omega_start_upper_exzone);
					Qg_min_partial = Kopt * omegafilt*omegafilt * x + lower_exZone_torque_limit * (1.0 - x);

					check_three = 1;

					//if ( (omega4exZone < omega_saturated_upper_exzone) && ( ( PID_gen_var.outres1_old < lower_exZone_torque_limit*1.01) || (trigger_ex_escape = 1) )) {
					if ((trigger_ex_escape == 1) || (PID_gen_var.outres1_old < lower_exZone_torque_limit*1.01)) {
						if (trigger_ex_escape == 0) { ReportInfoMessage(turbine_id, "Exiting Exclusion Zone Downwards"); }
						trigger_ex_escape = 1;
						trigger_ex_entrance = 0;
					}
					else trigger_ex_escape = 0;

				}
				else { // rotor speed entering from below tower eigenfrequency

					/*Qg_min_partial = fmin(Kopt * omega_start_lower_exzone*omega_start_lower_exzone, Kopt * omega4exZone * omega4exZone);
					x = switch_spline(omega4exZone, omega_start_lower_exzone, omega_saturated_lower_exzone);
					Qg_max_partial = Kopt * omega4exZone*omega4exZone * (1.0 - x) + upper_exZone_torque_limit * x;*/

					Qg_min_partial = fmin(Kopt * omega_start_lower_exzone*omega_start_lower_exzone, Kopt * omegafilt * omegafilt);

					x = switch_spline(omegafilt, omega_start_lower_exzone, omega_saturated_lower_exzone);
					Qg_max_partial = Kopt * omegafilt*omegafilt * (1.0 - x) + upper_exZone_torque_limit * x;

					check_three = 5;

					if ((trigger_ex_escape == 1) || (PID_gen_var.outres1_old > upper_exZone_torque_limit/1.01)) {
						if (trigger_ex_escape == 0) { ReportInfoMessage(turbine_id, "Exiting Exclusion Zone Upwards"); }
						trigger_ex_escape = 1;
						trigger_ex_entrance = 0;
					}
					else trigger_ex_escape = 0;


				}

				if (trigger_ex_escape){
					if ((omega4exZone < omega_saturated_lower_exzone)) { // && (ex_zone_sign > 0)) {
						//if (ex_zone_sign > 0) upper_exZone_torque_limit = last_Qg_min_partial;
						x = switch_spline(omega4exZone, omega_start_lower_exzone, omega_saturated_lower_exzone);
						//if (ex_zone_sign < 0) Qg_min_partial = lower_switch_exZone_torque * (1 - x) + upper_exZone_torque_limit * x;
						check_three = 2;
						if (ex_zone_sign > 0) {
							check_three = 6;
							Qg_min_partial = Kopt * omegafilt*omegafilt * (1 - x) + upper_exZone_torque_limit * x;
							Qg_max_partial = Qg_min_partial;
						}
						else trigger_ex_escape = 0;
						
					}
					else if (omega4exZone < omega_saturated_upper_exzone) {
						x = switch_spline(omega4exZone, omega_saturated_lower_exzone, omega_saturated_upper_exzone);
						//Qg_min_partial = upper_exZone_torque_limit * (1 - x) + lower_exZone_torque_limit * x + transition_exZone_Qg_min_partial; // -ex_zone_sign * torque_rate_limit_compensator;// - torque_rate_limit_compensator;
						if (ex_zone_sign > 0) Qg_min_partial = upper_exZone_torque_limit * (1 - x) + (lower_exZone_torque_limit + transition_exZone_Qg_min_partial) * x;
						else				  Qg_min_partial = (upper_exZone_torque_limit + transition_exZone_Qg_min_partial) * (1 - x) + lower_exZone_torque_limit * x;
						
						Qg_max_partial = Qg_min_partial;
						check_three = 3;
					}
					else if ((omega4exZone < omega_start_upper_exzone)) { // && (ex_zone_sign < 0)) {
						x = switch_spline(omega4exZone, omega_saturated_upper_exzone, omega_start_upper_exzone);
						//Qg_min_partial = upper_switch_exZone_torque * x + lower_exZone_torque_limit * (1 - x);
						if (ex_zone_sign < 0) {
							check_three = 7;
							Qg_min_partial = Kopt * omegafilt*omegafilt * x + lower_exZone_torque_limit * (1 - x);
							Qg_max_partial = Qg_min_partial;
						} 
						else trigger_ex_escape = 0;

						check_three = 4;
						//torque_rate_limit_compensator = 0;

					} //else	ReportInfoMessage(turbine_id, "Something is rotten in the state of denmark!");

					//last_Qg_min_partial = Qg_min_partial;
				}
				else {x = switch_spline(omega4exZone, omega_saturated_lower_exzone, omega_saturated_upper_exzone);
					transition_exZone_Qg_min_partial = Qgen_ref - (upper_exZone_torque_limit * (1 - x) + lower_exZone_torque_limit * x);
					// compensating the difference of torque in the transition to the exit phase of the exclusion zone. (cause of need is the divergence of omega to the fixed saturation value of omega for exiting the exclusion zone)
				}
			}
			else {
				trigger_ex_entrance = 0;
			    trigger_ex_escape = 0;
			}
		}

		
		//SetLoggingValue(turbine_id, 32, trigger_ex_escape);
		double exZone_spline = x;


		// ########################################################################################
		// ############################# SPEED EXCLUSION ZONE #####################################
		// ########################################################################################

		/*
		// notchfilter exclusion zone rotor speed???
		Ex_mode_filt.zeta1 = 0.1 * 6;
		Ex_mode_filt.zeta2 = 0.001 * 6;
		Ex_mode_filt.f0 = 0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
								//if (nmbrBlades == 2)	Ex_mode_filt.f0 = 0.124;//  0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
		if (use90mpsVersion != 1) Ex_mode_filt.f0 = 0.124;
		Ex_mode_filt = notch2orderfilt(deltat, stepno, Ex_mode_filt, omegafilt);
		SetLoggingValue(turbine_id, 29, Ex_mode_filt.y1);

		Ex_mode_filt2.f0 = 0.170; //0.179 // Hz value of filter notch filtered frequency
		Ex_mode_filt2 = notch2orderfilt(deltat, stepno, Ex_mode_filt2, omegafilt);
		SetLoggingValue(turbine_id, 30, Ex_mode_filt2.y1);

		double omega4exZone = Ex_mode_filt.y1; // omegafilt; // omega or omegafilt ???
											   //double omega4exZone = omegafilt; */



		// Switch based on pitch
		switch1 = switch_spline(meanpitang, switch1_pitang_lower, switch1_pitang_upper);
		switchfirstordervar = lowpass1orderfilt(deltat, stepno, switchfirstordervar, switch1);
		switch1 = switchfirstordervar.y1;

		if (t_cutin > 0.0) { // During start up pitch angle is quite big but min torque shouldn't be forced to be rated torque
			if (generator_cutin) {
				switch1 = switch1 * switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay);
			}
		}

		// Interpolation between partial and full load torque limits based on switch 1
		PID_gen_var.outmin = (1.0 - switch1)*Qg_min_partial + switch1 * Qg_min_full;
		PID_gen_var.outmax = (1.0 - switch1)*Qg_max_partial + switch1 * Qg_max_full;
		if (PID_gen_var.outmin > PID_gen_var.outmax) PID_gen_var.outmin = PID_gen_var.outmax;

		double K_t_SS_damp = 100000000;
		//if (useGenTorqueTowerSS_Damper) if (time > 15) omega_err_filt_speed = omega_err_filt_speed + K_t_SS_damp * Tower_SS_Accel;

		//Compute PID feedback to generator torque
		kgain_torque[0] = 1.0;
		kgain_torque[1] = 1.0;
		kgain_torque[2] = 1.0;
		PID_gen_var = PID(stepno, deltat, kgain_torque, PID_gen_var, omega_err_filt_speed);
		Qgen_ref = PID_gen_var.outres;

		int start_time_towerSSdamper = 15;
		if (useGenTorqueTowerSS_Damper) if (time > start_time_towerSSdamper) Qgen_ref = Qgen_ref + K_t_SS_damp * Tower_SS_Accel * (pow(time,3) - pow(start_time_towerSSdamper - deltat,3)) / pow(time,3);

		//------------------------------------------------------------------------------------------------//
		//						Control of cut - in regarding generator torque							  //
		//------------------------------------------------------------------------------------------------//
		if (t_cutin > 0.0) // (controller_state == 3) //
		{
			if (generator_cutin) {
				x = switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay);
				// x = switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay/3);
				Qgen_ref = Qgen_ref * x;
			}
		}

		//------------------------------------------------------------------------------------------------//
		//						Control of cut - out regarding generator torque							  //
		//------------------------------------------------------------------------------------------------//
		
		if ( t_cutout > 0.0 && time > t_cutout)  {
			if (Init_cutout_Qgen_ref == 0) Init_cutout_Qgen_ref = Qgen_ref / omegafilt;
			if (time > t_cutout + 5) x = switch_spline(time, t_cutout +5, t_cutout +5 + t_cutout_delay);
			else x = 0;
			Qgen_ref = Init_cutout_Qgen_ref * omegafilt; //* (1 - x);
			// torquefirstordervar = lowpass1orderfilt(deltat, stepno, torquefirstordervar, 0.0);
			// Qgen_ref = torquefirstordervar.y1;
		}
		//else {	torquefirstordervar = lowpass1orderfilt(deltat, stepno, torquefirstordervar, Qgen_ref);}

		//------------------------------------------------------------------------------------------------//
		//									Reference electrical power									  //
		//------------------------------------------------------------------------------------------------//
		Pe_ref = Qgen_ref * omega;

		//------------------------------------------------------------------------------------------------//
		//				Active DT (drive train) damping based on notch filtered of rotor speed			  //
		//------------------------------------------------------------------------------------------------//
		if ((DT_damp_gain > 0.0) && (DT_damper_filt.f0 > 0.0))
		{
			DT_damper_filt = bandpassfilt(deltat, stepno, DT_damper_filt, omega);
			omega_dtfilt = DT_damper_filt.y1;
			if (t_cutin > 0.0) // (controller_state == 3) //
			{
				if ((generator_cutin) && (time > 35))
				{
					x = switch_spline(time, t_generator_cutin + t_cutin_delay, t_generator_cutin + 2.0 * t_cutin_delay);
					Qdamp_ref = DT_damp_gain * omega_dtfilt * x;
					Qgen_ref = fmin(fmax(Qgen_ref + Qdamp_ref, 0.0), max_lss_torque);
				}
			}
			else if (time > 35)
			{
				Qdamp_ref = DT_damp_gain * omega_dtfilt;
				Qgen_ref = fmin(fmax(Qgen_ref + Qdamp_ref, 0.0), max_lss_torque);
			}
		}

		//SetLoggingValue(turbine_id, 31, Qdamp_ref);
		//**************************************************************************************************//
		//							PID regulation of collective pitch angle								//
		//**************************************************************************************************//
		//Reference speed is equal rated speed
		omega_err_filt_pitch = omegafilt - omega_ref_max;


		//Limits
		PID_pit_var.outmin = theta_min;
		PID_pit_var.outmax = pitch_stopang;
		//Aerodynamic gain scheduling
		if (kk2 > 0.0)
			aero_gain = 1.0 + meanpitangfilt / kk1 + meanpitangfilt * meanpitangfilt / kk2;
		else aero_gain = 1.0 + meanpitangfilt / kk1;

		// Nonlinear gain to avoid large rotor speed excursion
		//--kgain_pitch = (omega_err_filt_pitch* omega_err_filt_pitch / pow(omega_ref_max* (rel_limit - 1.0), 2) + 1.0) / aero_gain;
		double help_var;
		help_var = (omega_err_filt_pitch* omega_err_filt_pitch / pow(omega_ref_max* (rel_limit - 1.0), 2) + 1.0) / aero_gain;
		kgain_pitch[0][0] = help_var;
		kgain_pitch[1][0] = help_var;
		kgain_pitch[2][0] = help_var;
		kgain_pitch[0][1] = help_var;
		kgain_pitch[1][1] = help_var;
		kgain_pitch[2][1] = help_var;

		if (GridlossDetected == 1) {
			kgain_pitch[0][1] = 0.0;
			kgain_pitch[1][1] = 0.0;
			kgain_pitch[2][1] = 0.0;
		}
		//------------------------------------------------------------------------------------------------//
		//								Control of cut - in regarding pitch								  //
		//------------------------------------------------------------------------------------------------//
		if (t_cutin > 0.0) //(controller_state == 3) //
		{
			if (time < t_cutin)
			{
				PID_pit_var.outmin = pitch_stopang;
				PID_pit_var.outmax = pitch_stopang;
				//--kgain_pitch = 0.0;
				kgain_pitch[0][0] = 0.0;
				kgain_pitch[1][0] = 0.0;
				kgain_pitch[2][0] = 0.0;
				kgain_pitch[0][1] = 0.0;
				kgain_pitch[1][1] = 0.0;
				kgain_pitch[2][1] = 0.0;

				omega_err_filt_pitch = omegafilt - omega_ref_min;
				cutinfirstordervar = lowpass1orderfilt(deltat, stepno, cutinfirstordervar, omega - omega_ref_min);
				// unnecessary: dummy = cutinfirstordervar.y1;
			}
			else
			{
				double adapt_non_lin_pitch_gain = 0.75; // original = 0.25 (should be between 0 and 1)
				if (!generator_cutin)
				{
					// ########  ANSFAB CUSTOM
					double t_cutin_pitch_delay = pitch_stopang / (3 * degrad); // max start up velocity of 3 deg/s
					x = switch_spline(time, t_cutin, t_cutin + t_cutin_pitch_delay);
					PID_pit_var.outmin = pitch_stopang * (1-x);
					// ########
					kgain_pitch[0][0] = adapt_non_lin_pitch_gain * kgain_pitch[0][0];
					kgain_pitch[1][0] = adapt_non_lin_pitch_gain * kgain_pitch[1][0];
					kgain_pitch[2][0] = adapt_non_lin_pitch_gain * kgain_pitch[2][0];
					kgain_pitch[0][1] = 0.0;
					kgain_pitch[1][1] = 0.0;
					kgain_pitch[2][1] = 0.0;
					omega_err_filt_pitch = omegafilt - omega_ref_min;
					cutinfirstordervar = lowpass1orderfilt(deltat, stepno, cutinfirstordervar, omega - omega_ref_min);
					x = cutinfirstordervar.y1;
					if (abs(x) < omega_ref_min *0.20) // CHANGED: * 0.01)
					{
						ReportInfoMessage(turbine_id, "generator cutting in now (20%before omega_min)");
						generator_cutin = true;
						t_generator_cutin = time;
						Qgen_ref = 0;								// avoiding jump to current Qgen_ref 
						SetGeneratorContactor(turbine_id, 1);		// Sets the generator contactor: 0=OFF, 1=MAIN, 2=LOW SPEED. Returns 0 for success, and -1 for an error. 
					}
				}
				else
				{
					x = switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay);
					omega_err_filt_pitch = omegafilt - (omega_ref_min*(1.0 - x) + omega_ref_max * x);
					kgain_pitch[0][0] = adapt_non_lin_pitch_gain *kgain_pitch[0][0] + kgain_pitch[0][0] * (1 - adapt_non_lin_pitch_gain) * x;
					kgain_pitch[1][0] = adapt_non_lin_pitch_gain *kgain_pitch[1][0] + kgain_pitch[1][0] * (1 - adapt_non_lin_pitch_gain) * x;
					kgain_pitch[2][0] = adapt_non_lin_pitch_gain *kgain_pitch[2][0] + kgain_pitch[2][0] * (1 - adapt_non_lin_pitch_gain) * x;
					kgain_pitch[0][1] = kgain_pitch[0][1] * x;
					kgain_pitch[1][1] = kgain_pitch[1][1] * x;
					kgain_pitch[2][1] = kgain_pitch[2][1] * x;
					dummy = time;
				}
			}
		}

		
		//------------------------------------------------------------------------------------------------//
		//							Control of cut - out regarding pitch								  //
		//------------------------------------------------------------------------------------------------//
		//double x2 =0;
		if (t_cutout > 0.0 && time > t_cutout ) // + pitch_stopdelay) WHY SHOULD THERE BE A PITCH DELAY??? (Delay of actuator is included in Bladed) // || GridlossDetected == 1 )
		{
			switch (pitch_stoptype) {
			case 1: //Normal 2 - step stop situation
				if (tellItOnce == 0) {
					
					tellItOnce = 1;
					
					if (GridlossDetected == 1) {
						pitch_stopvelmax2 = pitch_stopvelmax2 / omegafilt / 4; // / 4;
						// t_cutout_delay = t_cutout_delay *2;
					} else if (StuckedPitchDetected == 1) {
						pitch_stopvelmax2 = pitch_stopvelmax2 / omegafilt / 2;
						// t_cutout_delay = t_cutout_delay *2;
					} else if (GetControllerState(turbine_id) == 5) {
						ReportInfoMessage(turbine_id, "Emergency stop initiated");
						pitch_stopvelmax2 = pitch_stopvelmax / omegafilt;
						t_cutout_delay = t_cutout_delay / 1.5;
						t_cutout = t_cutout - t_cutout_delay / 5;
					} else {
						pitch_stopvelmax2 = pitch_stopvelmax2 / omegafilt;
						ReportInfoMessage(turbine_id, "Normal 2-step stop");
					}
					
				}
				PID_pit_var.outmax = pitch_stopang;
				PID_pit_var.outmin = pitch_stopang;
				
				x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay);
				//if (GridlossDetected == 1) x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay*2);

				/*if (GridlossDetected == 1) {
					x = fmax(0.01,switch_spline(time, t_cutout + t_cutout_delay / 3, t_cutout + t_cutout_delay * (1+1/3)) + (1 - switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 4)) );
					// x = switch_spline(time, t_cutout+ t_cutout_delay/4.1, t_cutout + t_cutout_delay) + (1 - switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 4));
					x2 = 1 - switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 4);
					// x = switch_spline(time, t_cutout - t_cutout_delay, t_cutout + t_cutout_delay * 2) + switch_spline(t_cutout, time, t_cutout + t_cutout_delay/4);

					// x = switch_spline(time, t_cutout - t_cutout_delay, t_cutout + t_cutout_delay * 2) * 2 - switch_spline(time, t_cutout - t_cutout_delay/2, t_cutout + t_cutout_delay);
					// x = switch_spline(time, t_cutout - t_cutout_delay, t_cutout + t_cutout_delay*2);  //x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay/5);
				}*/

				if (omegafilt > omega_ref_min)
					 PID_pit_var.velmax = pitch_stopvelmax2 * x * omegafilt; // multiply omegafilt for pitch sensivity tuning
				else PID_pit_var.velmax = pitch_stopvelmax2 * x * omega_ref_min;
				/* if (time > t_cutout + pitch_stopdelay + pitch_stopdelay2)
					PID_pit_var.velmax = pitch_stopvelmax2;
				else PID_pit_var.velmax = pitch_stopvelmax; */
				break;

			/*case 2: //Exponential decay approach
				if (tellItOnce == 0) {
					ReportInfoMessage(turbine_id, "Exponential decay stop");
					tellItOnce = 1;
				}
				PID_pit_var.outmax = pitch_stopang;
				PID_pit_var.outmin = pitch_stopang;
				if ((time - (t_cutout + pitch_stopdelay)) / pitch_stopdelay2 < 10.0)
					PID_pit_var.velmax = pitch_stopang / pitch_stopdelay2
					* exp(-(time - (t_cutout + pitch_stopdelay)) / pitch_stopdelay2);
				else PID_pit_var.velmax = 0.0;

				if (PID_pit_var.velmax > pitch_stopvelmax)  PID_pit_var.velmax = pitch_stopvelmax;
				if (PID_pit_var.velmax < pitch_stopvelmax2) PID_pit_var.velmax = pitch_stopvelmax2;
				break;*/
			default:
				/*sprintf(ErrorMess, " *** ERROR *** Stop type %i not known", pitch_stoptype);
				ReportInfoMessage(turbine_id, ErrorMess);*/
				ReportInfoMessage(turbine_id, " *** ERROR *** Stop type is not known");
				break;
			}
		}

		

		//------------------------------------------------------------------------------------------------//
		//							Compute PID feedback to generator torque							  //
		//------------------------------------------------------------------------------------------------//
		double K_t_FA_damp = 0.1;
		if (DT_mode_filt.f0 > 0.0)
		{
			DT_mode_filt = notch2orderfilt(deltat, stepno, DT_mode_filt, omega_err_filt_pitch);
			e_pitch[0] = DT_mode_filt.y1;
			pwr_DT_mode_filt = notch2orderfilt(deltat, stepno, pwr_DT_mode_filt, Pe_ref - Pe_rated);
			e_pitch[1] = pwr_DT_mode_filt.y1;
		}
		else
		{
			e_pitch[0] = omega_err_filt_pitch;
			e_pitch[1] = Pe_ref - Pe_rated;
			if (filtFreqPowerError_pitch > 0.0) {		// might be good to filter power error as well
				pitch_power_error_filt.zeta1 = 0.1 * 12; // *6;
				pitch_power_error_filt.zeta2 = 0.001 * 12; // 6;
				pitch_power_error_filt.f0 = filtFreqPowerError_pitch;
				pitch_power_error_filt = notch2orderfilt(deltat, stepno, pitch_power_error_filt, e_pitch[1]);
				e_pitch[1] = pitch_power_error_filt.y1;
			}
			if (usePitchTowerFA_Damper) if (time > 15) e_pitch[0] = omega_err_filt_pitch + K_t_FA_damp * tower_FA_velo; // copysign(pow(tower_FA_velo, 2) * K_t_FA_damp, tower_FA_velo);
		}
		
		if (stepno == 1) PID_pit_var.outset1 = GetOptiPitch(nominalWindSpeed); // initial pitch state in full load operation
		PID_pit_var = PID2(stepno, deltat, kgain_pitch, PID_pit_var, e_pitch);//, GetOptiPitch(nominalWindSpeed));
		theta_col_ref = PID_pit_var.outres;
		//if (time < t_cutin) theta_col_ref = pitch_stopang; // #QUICKNDIRTY
		
		//################# ????FILTERING PITCH SIGNAL?????? #####################
		if (towerfrequencyForPitchFilt != 0) {
			pitch_out_signal_filt.zeta1 = 0.1 * 12; // *6;
			pitch_out_signal_filt.zeta2 = 0.001 * 12; // 6;
			pitch_out_signal_filt.f0 = towerfrequencyForPitchFilt;
			pitch_out_signal_filt = notch2orderfilt(deltat, stepno, pitch_out_signal_filt, theta_col_ref);
			theta_col_ref = pitch_out_signal_filt.y1;
		}
		//################# ????FILTERING PITCH SIGNAL?????? #####################
		
		thetaref[0] = theta_col_ref;
		thetaref[1] = theta_col_ref;
		thetaref[2] = theta_col_ref;
		//************************************************************************************************//
		//												Output											  //
		//************************************************************************************************//
		//SetDemandedGeneratorTorque(turbine_id, Qgen_ref);		// 1 : Generator torque reference[Nm]
		/*if (theta_col_ref > 1 * degrad) //übergang zwischen voll und teillast scheint nicht richtig zu funktionieren
			SetDemandedGeneratorTorque(turbine_id, Qg_rated / GearBoxRatio);
		else
			SetDemandedGeneratorTorque(turbine_id, Qgen_ref / GearBoxRatio);*/
		SetDemandedGeneratorTorque(turbine_id, Qgen_ref/ GearBoxRatio);	// 1 : Generator torque reference[Nm]   // including gear box!!!!
		
		//SetDemandedGeneratorTorque(turbine_id, PID_gen_var.outmax / GearBoxRatio);
		SetDemandedPitchAngle(turbine_id, 0, thetaref[0]);		// 2 : Pitch angle reference of blade 1[rad]
		SetDemandedPitchAngle(turbine_id, 1, thetaref[1]);		// 3 : Pitch angle reference of blade 2[rad]
		if (nmbrBlades == 3) SetDemandedPitchAngle(turbine_id, 2, thetaref[2]);		// 4 : Pitch angle reference of blade 3[rad]
		
		if (GridlossDetected == 1) { // IS IT POSSIBLE TO APPLIE TORQUE WITH SLOPES???
			//SetDrivetrainBrakeStatus(turbine_id, 1, 1); //big effect
			//SetDrivetrainBrakeStatus(turbine_id, 1, 5);
			if (time > t_cutout) {
				//SetDemandedAdditionalBrakeTorque(turbine_id, -10 * deltat);
				SetShaftBrakeStatusBinaryFlag(turbine_id, 0);
			} else SetShaftBrakeStatusBinaryFlag(turbine_id, 31);
			//else SetDemandedAdditionalBrakeTorque(turbine_id, 10 * deltat);
		}


		SetLoggingValue(turbine_id, 0, Pe_ref);					// 5 : Power reference[W]
		SetLoggingValue(turbine_id, 1, WSPfilt);				// 6 : Filtered wind speed[m / s]
		SetLoggingValue(turbine_id, 2, omegafilt);				// 7 : Filtered rotor speed[rad / s]
		SetLoggingValue(turbine_id, 3, omega_err_filt_speed);	// 8 : Filtered rotor speed error for torque[rad / s]
		SetLoggingValue(turbine_id, 4, Qgen_ref); // omega_dtfilt);			// 9 : Bandpass filtered rotor speed[rad / s]
		SetLoggingValue(turbine_id, 5, PID_gen_var.outpro/ GearBoxRatio);		// 10 : Proportional term of torque contr.[Nm]
		SetLoggingValue(turbine_id, 6, PID_gen_var.outset/ GearBoxRatio);		// 11 : Integral term of torque controller[Nm]
		SetLoggingValue(turbine_id, 7, PID_gen_var.outmin/ GearBoxRatio);		// 12 : Minimum limit of torque[Nm]
		SetLoggingValue(turbine_id, 8, PID_gen_var.outmax/ GearBoxRatio);		// 13 : Maximum limit of torque[Nm]
		SetLoggingValue(turbine_id, 9, switch1);				// 14 : Torque limit switch based on pitch[-]
		SetLoggingValue(turbine_id, 10, omega_err_filt_pitch);	// 15 : Filtered rotor speed error for pitch[rad / s]
		SetLoggingValue(turbine_id, 11, e_pitch[1]);			// 16 : Power error for pitch[W]
		SetLoggingValue(turbine_id, 12, PID_pit_var.outpro);	// 17 : Proportional term of pitch controller[rad]
		SetLoggingValue(turbine_id, 13, PID_pit_var.outset);	// 18 : Integral term of pitch controller[rad]
		SetLoggingValue(turbine_id, 14, PID_pit_var.outmin);	// 19 : Minimum limit of pitch[rad]
		SetLoggingValue(turbine_id, 15, PID_pit_var.outmax);	// 20 : Maximum limit of pitch[rad]

		SetLoggingValue(turbine_id, 16, pitchADC*raddeg);	// 16 : pitch actuator duty cycle
		SetLoggingValue(turbine_id, 17, kgain_pitch[0][0] * PID_pit_var.Kpro[0]);		// 17 : pitch gain in controller (after gain scheduling)
		SetLoggingValue(turbine_id, 18, kgain_pitch[1][0] * PID_pit_var.Kint[0]);		// 18 : pitch gain in controller
		SetLoggingValue(turbine_id, 19, kgain_pitch[0][1] * PID_pit_var.Kpro[1]);	// 19 : pitch gain in controller
		SetLoggingValue(turbine_id, 20, kgain_pitch[1][1] * PID_pit_var.Kint[1]);	// 20 : pitch gain in controller
		SetLoggingValue(turbine_id, 21, kgain_pitch[0][0]);// 21 : pitch gain scheduling in controller
		SetLoggingValue(turbine_id, 22, kgain_pitch[0][1]);// 22 : pitch gain scheduling in controller

		SetLoggingValue(turbine_id, 23, meanpitangfilt);				// 21 :
		SetLoggingValue(turbine_id, 24, switch1_pitang_upper);				// 21 :
		SetLoggingValue(turbine_id, 25, switch1_pitang_lower);				// 21 :
		SetLoggingValue(turbine_id, 26, Qg_min_partial);				// 21 :
		SetLoggingValue(turbine_id, 27, Qg_max_partial);				// 21 :


		/*SetLoggingValue(turbine_id, 23, check_one);				// 21 :
		SetLoggingValue(turbine_id, 24, check_two);				// 21 :
		SetLoggingValue(turbine_id, 25, ex_zone_sign);				// 21 :
		SetLoggingValue(turbine_id, 26, exZone_spline);				// 21 :
		SetLoggingValue(turbine_id, 27, check_three);				// 21 :
		SetLoggingValue(turbine_id, 28, torque_rate_limit_compensator);*/
		/*
		SetLoggingValue(turbine_id, 27, generator_cutin);						// 22 : for various tests
		SetLoggingValue(turbine_id, 28, controller_state);						// 22 : for various tests


		SetLoggingValue(turbine_id, 29, GetGeneratorContactor(turbine_id));
		SetLoggingValue(turbine_id, 30, GetLastErrorCode(turbine_id));
		SetLoggingValue(turbine_id, 31, GetMeasuredGeneratorTorque(turbine_id));
		SetLoggingValue(turbine_id, 32, GridlossDetected);
		SetLoggingValue(turbine_id, 33, IsPitchLimitSwitchTripped(turbine_id, 0));						// 22 : for various tests
		SetLoggingValue(turbine_id, 34, x);
		SetLoggingValue(turbine_id, 35, t_cutout);
		SetLoggingValue(turbine_id, 36, GetShaftBrakeStatusBinaryFlag(turbine_id));						// 22 : for various tests
		SetLoggingValue(turbine_id, 37, PID_pit_var.velmax);						// 22 : for various tests
		SetLoggingValue(turbine_id, 38, (abs(PID_pit_var.outres - PID_pit_var.outres1_old) / deltat));
		SetLoggingValue(turbine_id, 39, PID_pit_var.outres1_old);
		*/
		
		
		

		/*SetLoggingValue(turbine_id, 23, switch1_pitang_upper1);				// 21 :
		SetLoggingValue(turbine_id, 24, switch1_pitang_lower1);				// 21 :
		SetLoggingValue(turbine_id, 25, switch1_pitang_upper2);				// 21 :
		SetLoggingValue(turbine_id, 26, switch1_pitang_lower2);				// 21 :*/
		
		
		/* not qiuet sure, what to do with the other output parameter
		array2[4] = Pe_ref;					// 5 : Power reference[W]
		array2[5] = WSPfilt;				// 6 : Filtered wind speed[m / s]
		array2[6] = omegafilt;				// 7 : Filtered rotor speed[rad / s]
		array2[7] = omega_err_filt_speed;	// 8 : Filtered rotor speed error for torque[rad / s]
		array2[8] = omega_dtfilt;			// 9 : Bandpass filtered rotor speed[rad / s]
		array2[9] = PID_gen_var.outpro;	// 10 : Proportional term of torque contr.[Nm]
		array2[10] = PID_gen_var.outset;	// 11 : Integral term of torque controller[Nm]
		array2[11] = PID_gen_var.outmin;	// 12 : Minimum limit of torque[Nm]
		array2[12] = PID_gen_var.outmax;	// 13 : Maximum limit of torque[Nm]
		array2[13] = switch1;				// 14 : Torque limit switch based on pitch[-]
		array2[14] = omega_err_filt_pitch;	// 15 : Filtered rotor speed error for pitch[rad / s]
		array2[15] = e_pitch[2];			// 16 : Power error for pitch[W]
		array2[16] = PID_pit_var.outpro;	// 17 : Proportional term of pitch controller[rad]
		array2[17] = PID_pit_var.outset;	// 18 : Integral term of pitch controller[rad]
		array2[18] = PID_pit_var.outmin;	// 19 : Minimum limit of pitch[rad]
		array2[19] = PID_pit_var.outmax;	// 20 : Maximum limit of pitch[rad]
		array2[20] = Qdamp_ref;				// 21 : Torque reference from DT damper[Nm]
		*/
		
		//	return; } //- end subroutine update_regulation
		//**************************************************************************************************//


		/*
		// Add active pitch teeter coupling
		hdr[TEETERANGLE].value = GetMeasuredTeeterAngle(turbine_id)*180/PI ;    // Returns measured teeter angle in rad
		if ( UseActivePTC == 1 )  DeltaPitch = tan(Delta3_angle*PI/180)*hdr[TEETERANGLE].value;  // pitch teeter coupled aktive pitching */

		return GH_DISCON_SUCCESS ;
	}
}

/*
switch (contmode)
{
case 0:
	sprintf_s(contmodestring, "Waiting for wind"); break;
case 1:
	sprintf_s(contmodestring, "Starting"); break;
case 2:
	sprintf_s(contmodestring, "Operating"); break;
case 3:
	sprintf_s(contmodestring, "High wind cutout"); break;
}

if( strcmp(contmodestring,contmodeoldstring)!=0 )
{
ReportInfoMessage( turbine_id, contmodestring );
strcpy( contmodeoldstring, contmodestring );
}*/




/* #####################################################################################
CHANGE LOG:
14.02.19: Added drive train losses to aim for higher rated torque
until 22.03.19: Added 20MW option; preliminary version of speed exclusion zone
23.03.19: Using start up logic
####################################################################################### */