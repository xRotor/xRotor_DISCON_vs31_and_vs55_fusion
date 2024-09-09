#include "cmath"
#pragma comment(lib, "ExternalControllerApi.lib")
#include "ExternalControllerApi.h" // defines the C functions for the external balded controller
#include "risoe_controller_fcns.h" // defines the C functions of the risoe DTU controller
#include "legacy_controller_fcns.h" // defines the older and not mandatory controller functionalities
#include "string"
//#include "iostream"
//#include "stdio.h""
using namespace GHTurbineInterface;

// SDK-Version 10.0.16299.91

double towerfrequencyForPitchFilt = 0; 	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
double filtFreqPowerError_pitch = 0; 	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
int useMW = 20;							// choose 20 for 20 MW turbine or else for 10 MW
int use90mpsVersion = 1;				// 1 if 90 m/s 2B version, else 100 m/s version is used
int use2B_upscaled = 1;					// 1 if the upscaled versions of the 2B turbines should be used (they have the same power at 3B rated wind), else 2 etc.
int use2B_teetering = 0;				// 1 if a teering version of the 2B turbine should be used, else 0 etc.
int useSpeedExZone = 0;					// 1 = using exclusion zone; else not
int useConstPower = 2;					// 15: Generator control switch[1 = constant power, 2 = constant torque] default = 2
int nmbrBlades = 3;
int useFloatingTurbine = 0;				// 1 if turbine is floating and floating features are required. Else 0.
double maxOverspeedFactor = 1.12;		// factor to apply to omega rated [default = 1.12]
double GearBoxRatio = 47.6;
double factor_GenT_latTowDamper = 0;	// !=0 => using active tower side-side damper by generator torque; else not
double factor_Pitch_FATowDamper = 0;	// !=0 => using active tower fore-aft  damper by pitching; else not
double GenT_latTowDamper_gain = 0;		// StS damper gain
double Pitch_FATowDamper_gain = 0;		// FA  damper gain
double towerFrequencyToAvoid = 0;		// !=0 => try to avoid this tower eigenfrequency with 2P or 3P rotor speed; else to speed exclusion zone
double MaxPitchVelocity = 7.071;		// maximum pitch actuator velocity in deg/s
double fixedPitchAng = -5;		        // will stay in declared pitch angle for 150s and jump to 0.5 deg higher pitch (to evaluate aerodynamic gain); not used if negative
double IgainIPC_1P = 0;
double IgainIPC_2P = 0;

int useTxtFieldInput = 1;
double usePitchRunAwayAt = 0;
double eps_pitchError = 0.01;
double theta_col_FA_damp = 0;

double nacelle_FA_velocity = 0, nacelle_FA_velocity_filtered = 0;
double tower_FA_velo = 0, tower_SS_velo = 0, nacelle_roll_velo = 0;
double transition_exZone_Qg_min_partial;
double time_needed_for_exiting = 25; // seconds to pass first spline of exiting
double time_exit_exzone = -time_needed_for_exiting * 3; // time when exZone should be escaped (to direcly enable the exit of the exclusion Zone)
double rel_omega_exclusion_zone;
double exZone_torque_memory = 0;

double temp_LastMyBlade1 = 0, temp_LastMyBlade2 = 0;
double temp_Blade1XVelo = 0, temp_Blade2XVelo = 0;
double temp_oldYawAngle = 0;

double GA_Parameter1 = 0, GA_Parameter2 = 0, GA_Parameter3 = 0, GA_Parameter4 = 0, GA_Parameter5 = 0, GA_Parameter6 = 0, GA_Parameter7 = 0, GA_Parameter8 = 0;
double GA_Parameter21 = 0, GA_Parameter22 = 0, GA_Parameter23 = 0, GA_Parameter24 = 0, GA_Parameter25 = 0, GA_Parameter26 = 0, GA_Parameter27 = 0, GA_Parameter28 = 0;

// ---- user specific controller parameters ---- //
//int UseInitialValues = 0, UseActivePTC = 0, UseLowPassWindSpeedFilter = 1, UseConstPowerProd = 0; // 1 ^= use; 0^= don't use
//int TimeToStopFeedback = 800;  // use to as start for static wind runs, otherwise put a high value (in seconds) (bigger 100)
//double Delta3_angle = 45;       // delta 3 angle in deg (just needed for active PTC)

double pred_stepno = 0; // predicted step number from last exclusion zone function iteration
double switch_ex_zone = 0; // switch omega error hysteresis in exclusion zone
double ex_zone_sign = 0;
int GridlossDetected = 0; // switch for detecting fault cut out
int StuckPitchDetected = 0; // switch for detecting fault cut out
int apply_teeter_brake = 0;  // switch to apply the teeter brake. Is only used for a two-bladed teetering turbine.
int AllPitchLimitSwitchTripped; // used to enable 2 and 3 bladed contoller
double omegaAtGridloss = 0;
//double NormalOrEStopDetected = 0; 
double t_cutout_delay = 30;


double drive_train_loss = 0.94; // full drive train loss in %
double dummy = 0; // just for checks
double switch1_pitang_lower1, switch1_pitang_upper1, switch1_pitang_lower2, switch1_pitang_upper2;

// Local variables
double time, omega, omegafilt, minimum_pitch_angle, wsp, WSPfilt, // domega_dt_filt,
omega_err_filt_pitch, omega_err_filt_speed, omega_dtfilt,
ommin1, ommin2, ommax1, ommax2,
meanpitang, meanpitangfilt, theta_min,
aero_gain, help_var, x, // unnecessary: dummy,
Qg_min_partial, Qg_max_partial, Qg_min_full, Qg_max_full,
Qgen_ref, theta_col_ref, Pe_ref, Qdamp_ref;

double e_pitch[2];
double y[2];
double pitang[3];
double thetaref[3];
double thetaref_last[3];
double kgain_torque[3];
double kgain_pitch[3][2];
double yaw_stopvelmax = 1.5 * degrad;
double yawRef = 0;
double meanYawMisalignment = 0, time_at_extremeYawMisalignment = 0;
int extremeYawMisalignment = 0;

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
char* reportMessage = new char[55];

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

	int __declspec(dllexport) __cdecl CONTROLLER(const turbine turbine_id);

	/*! Controller to be run on first timestep. */
	int __declspec(dllexport) __cdecl CONTROLLER_INIT(const turbine turbine_id)
	{


		char* reportMessage = new char[55];// [3];

		ReportInfoMessage(turbine_id, "receiving control parameters.");
		const char* userParameters = GetUserParameters(turbine_id); //ReportInfoMessage(turbine_id, userParameters); // reading in txt field from bladed external controller
		std::string strParam(userParameters); // reformating const char* in str

		// transforming the user parameter text field into specific control parameters
		double pitchPGain			= readInUserParameters("pitchPGain", userParameters, 0, turbine_id).parameter_value;
		double pitchIGain			= readInUserParameters("pitchIGain", userParameters, 0, turbine_id).parameter_value;
		double pitchPpowerGain		= readInUserParameters("pitchPpowerGain", userParameters, 0, turbine_id).parameter_value;
		double pitchIpowerGain		= readInUserParameters("pitchIpowerGain", userParameters, 0, turbine_id).parameter_value;

		nmbrBlades					= int(readInUserParameters("nmbrBlades", userParameters, 3, turbine_id).parameter_value);
		maxOverspeedFactor			= readInUserParameters("maxOverspeedFactor", userParameters, 1.8, turbine_id).parameter_value;		// factor to apply to omega rated [default = 1.12]
		towerfrequencyForPitchFilt	= readInUserParameters("towerfrequencyForPitchFilt", userParameters, 0, turbine_id).parameter_value;		// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
		filtFreqPowerError_pitch	= readInUserParameters("filtFreqPowerError_pitch", userParameters, 0, turbine_id).parameter_value;	// 2B90: 0.21; 3B: 0.32  // = 0 if signal should be unfiltered; default = 0
		GearBoxRatio				= readInUserParameters("GearBoxRatio", userParameters, 1, turbine_id).parameter_value;		//default = 47.6; // be careful, does vary with MW (20MW ~ 48 ; 10MW = 50)

		auto result_user_parameter	= readInUserParameters("factor_GenT_latTowDamper", userParameters, 0, turbine_id);
		factor_GenT_latTowDamper = result_user_parameter.parameter_value;	// != 0 => using active tower side-side damper by generator torque; else not
		StS_damper_gains_look_up = result_user_parameter.look_up_table;

		result_user_parameter		= readInUserParameters("factor_Pitch_FATowDamper", userParameters, 0, turbine_id);
		factor_Pitch_FATowDamper = result_user_parameter.parameter_value;	// != 0 => using active tower fore-aft  damper by pitching; else not
		FA_damper_gains_look_up	 = result_user_parameter.look_up_table;

		// FA 2B_ref [2:0.2,5:0.2,7:0.15,9:0.15,11:0.2,13:0.2,15:0.2,17:0.2,19:0.15,21:0.15,23:0.1,25:0.1]
		// StS 2B ref [2:0.1,5:0.1,7:0.1,9:0.1,11:0.1,13:0.1,15:0.1,17:0.05,19:0.05,21:0.05,23:0.05,25:0]
		// StS 2B ref [2:0.1,15:0.1,17:0.05,23:0.05,25:0]
		// FA 3B_ref [2:0.15,5:0.15,7:0.15,9:0.01,11:0,13:0.01,15:0.05,17:0.05,19:0.05,21:0.05,23:0.05,25:0.05]
		// StS 3B_ref [2:0.1,5:0.1,7:0.05,9:0.05,11:0.1,13:0,15:0,17:0.05,19:0.05,21:0.05,23:0.05,25:0.05]
		// FA  2B teeter ref [2:0.2,5:0.2,7:0.2,9:0.2,11:0.25,13:0.25,15:0.25,17:0.25,19:0.25,21:0.2,23:0.2,25:0.15]  //"Using overagressive FA damper gains!"
		// FA  2B teeter ref [2:0.3,5:0.3,7:0.25,9:0.25,11:0.23,13:0.215,15:0.2,17:0.2,19:0.2,21:0.2,23:0.2,25:0.18]  //"Using most overagressive FA damper gains!"
		// StS 2B teeter ref [2:0.3,5:0.3,7:0.3,9:0.2,11:0.1,13:0.1,15:0.2,17:0.2,19:0.2,21:0.2,23:0.2,25:0.2]  // "Using agressive StS damper gains!"
		// FA 2B LIPC [2:0.15,7:0.15,11:0.2,15:0.21,25:0.176]
		// StS 2B LIPC [2:0.65,7:0.65,11:0.2,15:0.1,25:0.044]
		// FA 2B_ref IEA Mono [2:0.3,5:0.3,7:0.2,9:0.1,11:0.25,13:0.07,15:0.07,17:0.7,19:0.16,21:0.1,23:0.08,25:0.06]
		
		towerFrequencyToAvoid		= readInUserParameters("towerFrequencyToAvoid", userParameters, 0, turbine_id).parameter_value;	// frequency for speed exclusion zone; 0 if not used
		MaxPitchVelocity			= readInUserParameters("MaxPitchVelocity", userParameters, 7.071, turbine_id).parameter_value * degrad;	// maximum pitch actuator velocity in deg/s (7.071 deg/s for 20 MW, legacy v31 controller was 15 deg/s)
		IgainIPC_1P					= readInUserParameters("IgainIPC_1P", userParameters, 0, turbine_id).parameter_value;		// I gain for IPC controller; 0 if not used
		IgainIPC_2P					= readInUserParameters("IgainIPC_2P", userParameters, 0, turbine_id).parameter_value;		// I gain for IPC controller; 0 if not used
		use90mpsVersion				= readInUserParameters("use90mpsVersion", userParameters, 0, turbine_id).parameter_value;		// 1 if 90 m/s 2B version, else 101 m/s version is used
		use2B_upscaled				= int(readInUserParameters("use2B_upscaled", userParameters, 0, turbine_id).parameter_value);		// 1 if the upscaled versions of the 2B turbines should be used (they have the same power at 3B rated wind), else 2 etc.
		use2B_teetering				= int(readInUserParameters("use2B_teetering", userParameters, 0, turbine_id).parameter_value);		// 1 if a teering version of the 2B turbine should be used, else 0 etc.
		useConstPower				= int(readInUserParameters("useConstPower", userParameters, 0, turbine_id).parameter_value);			// Generator control switch[1 = constant power, 0 = constant torque] default = 0
		useMW						= int(readInUserParameters("useMW", userParameters, 20, turbine_id).parameter_value);			// choose 20 for 20 MW turbine or else for 10 MW
		useFloatingTurbine			= int(readInUserParameters("useFloatingTurbine", userParameters, 0, turbine_id).parameter_value);		// o if bottom fixed, 1 if floating turbine
		usePitchRunAwayAt			= readInUserParameters("usePitchRunAwayAt", userParameters, 0, turbine_id).parameter_value;		// start simulate failure pitch to feather in s, not used if 0
		fixedPitchAng				= readInUserParameters("fixedPitchAng", userParameters, 0, turbine_id).parameter_value;			// will stay in declared pitch angle for 150s and jump to 0.5 deg higher pitch (to evaluate aerodynamic gain); not used if negative
		time_needed_for_exiting		= readInUserParameters("timeToExitExclusionZone", userParameters, 0, turbine_id).parameter_value;	// will stay in declared pitch angle for 150s and jump to 0.5 deg higher pitch (to evaluate aerodynamic gain); not used if negative
		rel_omega_exclusion_zone	= readInUserParameters("rel_omega_exclusion_zone", userParameters, 0, turbine_id).parameter_value;	// will stay in declared pitch angle for 150s and jump to 0.5 deg higher pitch (to evaluate aerodynamic gain); not used if negative

		GA_Parameter1  = readInUserParameters("GA_Parameter11", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter2  = readInUserParameters("GA_Parameter12", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter3  = readInUserParameters("GA_Parameter13", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter4  = readInUserParameters("GA_Parameter14", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter5  = readInUserParameters("GA_Parameter15", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter6  = readInUserParameters("GA_Parameter16", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter7  = readInUserParameters("GA_Parameter17", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter8  = readInUserParameters("GA_Parameter18", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter21 = readInUserParameters("GA_Parameter21", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter22 = readInUserParameters("GA_Parameter22", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter23 = readInUserParameters("GA_Parameter23", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter24 = readInUserParameters("GA_Parameter24", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter25 = readInUserParameters("GA_Parameter25", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter26 = readInUserParameters("GA_Parameter26", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter27 = readInUserParameters("GA_Parameter27", userParameters, 0, turbine_id).parameter_value;
		GA_Parameter28 = readInUserParameters("GA_Parameter28", userParameters, 0, turbine_id).parameter_value;


		if (towerFrequencyToAvoid > 0) ReportInfoMessage(turbine_id, "Using speed exclusion zone");
		sprintf_s(reportMessage, 50, "FF IPC Para1: %.11f", GA_Parameter1); ReportInfoMessage(turbine_id, reportMessage); //just for checking
		sprintf_s(reportMessage, 50, "FF IPC Para5: %.11f", GA_Parameter5); ReportInfoMessage(turbine_id, reportMessage); //just for checking
		if (usePitchRunAwayAt > 0) ReportInfoMessage(turbine_id, "Simulation with pitch to feather failure!!!");

		if (factor_GenT_latTowDamper > 0) {
			if (factor_GenT_latTowDamper == 1) 
			if (StS_damper_gains_look_up.lines == 0) {
				ReportInfoMessage(turbine_id, "Using predefined flexible gains for StS-damper");
				StS_damper_gains_look_up = loop_up_table_library(nmbrBlades, use2B_teetering, useFloatingTurbine, IgainIPC_1P, IgainIPC_2P, turbine_id).StS_damper_gains_look_up;
			} else ReportInfoMessage(turbine_id, "Using own flexibel gains for generator-driven StS-damper");
			else ReportInfoMessage(turbine_id, "Using own gain for generator-driven StS-damper");
		}
		if (factor_Pitch_FATowDamper > 0) {
			if (factor_Pitch_FATowDamper == 1) 
				if (FA_damper_gains_look_up.lines == 0) {
					ReportInfoMessage(turbine_id, "Using predefined flexible gains for FA-damper");
					FA_damper_gains_look_up = loop_up_table_library(nmbrBlades, use2B_teetering, useFloatingTurbine, IgainIPC_1P, IgainIPC_2P, turbine_id).FA_damper_gains_look_up;
				} else ReportInfoMessage(turbine_id, "Using own flexibel gains for pitch FA-damper");
			else ReportInfoMessage(turbine_id, "Using own gain for pitch FA-damper");
		}

		if (IgainIPC_1P + IgainIPC_2P > 0) LIPC_predef_gains = loop_up_table_library(nmbrBlades, use2B_teetering, useFloatingTurbine, IgainIPC_1P, IgainIPC_2P, turbine_id).LIPC_predef_gains;
		if (use2B_teetering == 4) {
			ReportInfoMessage(turbine_id, "LIPC with 1P, 2P and 3P by bode design at 15 m / s");
			LIPC_predef_gains_1P = loop_up_table_library(nmbrBlades, use2B_teetering, useFloatingTurbine, IgainIPC_1P, IgainIPC_2P, turbine_id).LIPC_predef_gains_1P;
			LIPC_predef_gains_2P = loop_up_table_library(nmbrBlades, use2B_teetering, useFloatingTurbine, IgainIPC_1P, IgainIPC_2P, turbine_id).LIPC_predef_gains_2P;
			LIPC_predef_gains_3P = loop_up_table_library(nmbrBlades, use2B_teetering, useFloatingTurbine, IgainIPC_1P, IgainIPC_2P, turbine_id).LIPC_predef_gains_3P;
		}
		

		



		// Input array1 must contain:
		double array1[50];
		if (useMW == 20) {
			ReportInfoMessage(turbine_id, ">>> Running DTU 20MW RWT controller. <<<");
			//	 : Overall parameters
			array1[0] = 20000.00;	//  1: Rated power[kW]
			array1[1] = 0.237906864; // 3B: 0.237906864;  // org.: 0.45000;	//  2: Minimum rotor speed[rad / s]
			array1[2] = 0.71371927; // "new" 3B: 0.71371927, old 3B: 0.678034562; // org.: 0.74617;	//  3: Rated rotor speed[rad / s]
			array1[3] = 42426406.87; // org.: 4.4123E+07;	//  4: Maximum allowable generator torque[Nm]                    without gearbox???
			array1[4] = 100;		//  5: Minimum pitch angle, theta_min[deg],
			//	 : if | theta_min | >90, then a table of <wsp, theta_min> is read
			//   : from a file named ’wptable.n’, where n = int(theta_min)							 // AND WHERE IS THAT FILE???!!!
			array1[5] = 90;			//  6: Maximum pitch angle[deg]
			array1[6] = 7.071;		//  7: Maximum pitch velocity operation[deg / s]
			array1[7] = 0.14142;	//  8: Frequency of generator speed filter[Hz]
			array1[8] = 0.7;		//  9: Damping ratio of speed filter[-]
			array1[9] = 0; // 1.31; // org: 0.35000;	// 10: Frequency of free - free DT torsion mode[Hz], if zero no notch filter used
			if (useFloatingTurbine == 1) array1[9] = 1.31;
			//	 : Partial load control parameters
			array1[10] = 5.1376E+07;// 11: Optimal Cp tracking K factor[Nm / (rad / s) ^ 2],
			//	 : Qg = K * Omega ^ 2, K = eta * 0.5*rho*A*Cp_opt*R ^ 3 / lambda_opt ^ 3
			array1[11] = 2.73E+08 * pow(0.6, 3); // 4.7 before: 2.73E+08*pow(0.6, 3); // *0.593;	// 12: Proportional gain of torque controller[Nm / (rad / s)]                             // factor of 0.6 to tune prop. and integ. amplitude to same abs size
			ReportInfoMessage(turbine_id, "*0.6^6 on gen PI int. gain and *0.6^3 on prop.");
			array1[12] = 4.34E+07 * pow(0.6, 6); // before: 4.34E+07*pow(0.6, 8); //*0.6*0.6*0.6*0.6; // *0.593* 0.593* 0.593;	//;	// 13: Integral gain of torque controller[Nm / rad]
			sprintf_s(reportMessage, 55, "Gen: Prop. gain = %.0f, Int. gain = %.0f", array1[11], array1[12]);	ReportInfoMessage(turbine_id, reportMessage);
			array1[13] = 0;			// 14: Differential gain of torque controller[Nm / (rad / s ^ 2)]
			//	 : Full load control parameters
			array1[14] = useConstPower;			// 15: Generator control switch[1 = constant power, 2 = constant torque]
			array1[15] = 0.5245;	// 16: Proportional gain of pitch controller[rad / (rad / s)]
			array1[16] = 0.0999;	// 17: Integral gain of pitch controller[rad / rad]
			array1[17] = 0;			// 18: Differential gain of pitch controller[rad / (rad / s ^ 2)]

			array1[18] = 2.00E-09;	// 19: Proportional power error gain [rad/W]
			array1[19] = 1.41E-09;	// 20: Integral power error gain [rad/(Ws)]
			if (useTxtFieldInput) {
				array1[15] = pitchPGain;
				array1[16] = pitchIGain;
				array1[18] = pitchPpowerGain;
				array1[19] = pitchIpowerGain;
			}
			else if (nmbrBlades == 2) {
				array1[15] = array1[15] * 0.5;//2B90: * 0.57;	// 16: Proportional gain of pitch controller[rad / (rad / s)]
				array1[16] = array1[16] * 0.5;// 2B90: *0.56;	// 17: Integral gain of pitch controller[rad / rad]
				ReportInfoMessage(turbine_id, "*0.5 on pitch PI prop. gain and *0.5 on int.");
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
			//GearBoxRatio = 50;
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
			array1[11] = 7.33E+07 * pow(0.6, 3); //org: 2.73E+08*0.6*0.6;	// 12: Proportional gain of torque controller[Nm / (rad / s)]                             // factor of 0.6 to tune prop. and integ. amplitude to same abs size
			//ReportInfoMessage(turbine_id, "*0.6*0.6*0.6*0.6 on gen PI int. gain and *0.6*0.6 on prop.");
			array1[12] = 1.32E+07 * pow(0.6, 8); //org: 4.34E+07*0.6*0.6*0.6*0.6;	// 13: Integral gain of torque controller[Nm / rad]
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


		double abs_tip_speed = 101, blade_length_scaling = 1.0192;
		if (use90mpsVersion) {
			abs_tip_speed = 90;
			blade_length_scaling = 1.02133;
		}
		if (!use2B_upscaled) {
			blade_length_scaling = 1;
			ReportInfoMessage(turbine_id, "Using non upscaled version");
		}


		// Overall parameters
		Pe_rated = array1[0] * 1000 / drive_train_loss; // including power losses of 6% in drivetrain!!

		if (nmbrBlades == 2) {
			omega_ref_min = array1[1] / blade_length_scaling * abs_tip_speed / 90;
			omega_ref_max = array1[2] / blade_length_scaling * abs_tip_speed / 90; // just for 1.0194% bigger blades adapted tsr
			max_lss_torque = array1[3] * blade_length_scaling / abs_tip_speed * 90;
		}
		else {
			omega_ref_min = array1[1];
			omega_ref_max = array1[2];
			max_lss_torque = array1[3];
		}
		minimum_pitch_angle = array1[4] * degrad;
		pitch_stopang = array1[5] * degrad;
		PID_pit_var.velmax = array1[6] * degrad;
		omega2ordervar.f0 = array1[7];
		omega2ordervar.zeta = array1[8];
		DT_mode_filt.f0 = array1[9];

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
		kk1 = array1[20] * degrad;
		kk2 = array1[21] * degrad * degrad;
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
		pitch_stoptype = int(array1[27]);
		pitch_stopdelay = array1[28];
		pitch_stopvelmax = array1[29] * degrad;
		pitch_stopdelay2 = array1[30];
		pitch_stopvelmax2 = array1[31] * degrad;

		// Expert parameters(keep default values unless otherwise given)
		switch1_pitang_lower = array1[32] * degrad;
		switch1_pitang_lower1 = switch1_pitang_lower;
		switch1_pitang_upper = array1[33] * degrad;
		switch1_pitang_upper1 = switch1_pitang_upper;
		rel_sp_open_Qg = array1[34] * 0.01; // factor 0.01 because array1[34] is set in %
		wspfirstordervar.tau = array1[35] * 2.0 * pi / omega_ref_max;
		yawfirstordervar.tau = array1[35] * 2.0 * pi / omega_ref_max / 10;
		pitchfirstordervar.tau = array1[36] * 2.0 * pi / omega_ref_max;

		// Drivetrain damper
		DT_damp_gain = array1[37];
		DT_damper_filt.f0 = DT_mode_filt.f0;
		pwr_DT_mode_filt.f0 = DT_mode_filt.f0;

		// Default and derived parameters
		PID_gen_var.velmax = 0.0; // No limit to generator torque change rate
		Qg_rated = Pe_rated / omega_ref_max;
		switchfirstordervar.tau = 2.0 * pi / omega_ref_max;
		cutinfirstordervar.tau = 2.0 * pi / omega_ref_max;

		// IPC parameter def
		for (int i = 0; i < 4; i++) {
			PID_IPC_tilt_var[i].Kint = IgainIPC_1P / (Qg_rated / nmbrBlades) / (i + 1);
			PID_IPC_tilt_var[i].outmax = 90 * degrad; // 10 * degrad;
			PID_IPC_tilt_var[i].outmin = 0 * degrad; // -10 * degrad;
			PID_IPC_tilt_var[i].velmax = 0; // 0 for unlimited

			PID_IPC_yaw_var[i].Kint = IgainIPC_2P / (Qg_rated / nmbrBlades) / (i + 1);
			PID_IPC_yaw_var[i].outmax = 90 * degrad;
			PID_IPC_yaw_var[i].outmin = 0 * degrad;
			PID_IPC_yaw_var[i].velmax = 0;
		}



		// Wind speed table
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


		

		ReportInfoMessage(turbine_id, "Roll signal of StS damper is notch filtered!!!");

		//Initiate the dynamic variables
		stepno = 0;
		time_old = 0.0;
		deltat = GetCommunicationInterval(turbine_id); // 0.01;
		

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

		AddLogValue(turbine_id, "Pe_ref", "W");					// 0 : Power reference[W]
		AddLogValue(turbine_id, "WSPfilt", "m/s");				// 1 : Filtered wind speed[m / s]
		AddLogValue(turbine_id, "omegafilt", "rad/s");			// 2 : Filtered rotor speed[rad / s]
		AddLogValue(turbine_id, "omega_err_filt_speed", "rad/s");	// 3 : Filtered rotor speed error for torque[rad / s]
		AddLogValue(turbine_id, "omega_dtfilt", "rad/s");		// 4 : Bandpass filtered rotor speed[rad / s]
		AddLogValue(turbine_id, "PID_gen_var.outpro", "Nm");		// 5 : Proportional term of torque contr.[Nm]
		AddLogValue(turbine_id, "PID_gen_var.outset", "Nm");		// 6 : Integral term of torque controller[Nm]
		AddLogValue(turbine_id, "PID_gen_var.outmin", "Nm");		// 7 : Minimum limit of torque[Nm]
		AddLogValue(turbine_id, "PID_gen_var.outmax", "Nm");		// 8 : Maximum limit of torque[Nm]
		AddLogValue(turbine_id, "switch1", "-");					// 9 : Torque limit switch based on pitch[-]
		AddLogValue(turbine_id, "omega_err_filt_pitch", "rad/s");// 10 : Filtered rotor speed error for pitch[rad / s]
		AddLogValue(turbine_id, "e_pitch[1]", "W");				// 11 : Power error for pitch[W]
		AddLogValue(turbine_id, "PID_pit_var.outpro", "rad");	// 12 : Proportional term of pitch controller[rad]
		AddLogValue(turbine_id, "PID_pit_var.outset", "rad");	// 13 : Integral term of pitch controller[rad]
		AddLogValue(turbine_id, "PID_pit_var.outmin", "rad");	// 14 : Minimum limit of pitch[rad]
		AddLogValue(turbine_id, "PID_pit_var.outmax", "rad");	// 15 : Maximum limit of pitch[rad]
		AddLogValue(turbine_id, "pitch_actuator_duty_cycle", "deg");	// 16 : pitch actuator duty cycle
		AddLogValue(turbine_id, "t_cutout", "-");				// 17 : time to cut out
		AddLogValue(turbine_id, "final_pitchPGain", "-");		// 18 : pitch gain in controller (after gain scheduling)
		AddLogValue(turbine_id, "final_pitchIGain", "-");		// 19 : pitch gain in controller
		AddLogValue(turbine_id, "final_pitchPGain_power", "-");	// 20 : pitch gain in controller
		AddLogValue(turbine_id, "final_pitchIGain_power", "-");	// 21 : pitch gain in controller
		AddLogValue(turbine_id, "pitch_gain_schedule", "-");// 22 : pitch gain scheduling in controller
		AddLogValue(turbine_id, "pitch_gain_schedule_power", "-");// 23 : pitch gain scheduling in controller
		AddLogValue(turbine_id, "meanpitangfilt", "rad");			// 24 : for various tests
		AddLogValue(turbine_id, "switch1_pitang_upper", "rad");	// 25 : for various tests
		AddLogValue(turbine_id, "switch1_pitang_lower", "rad");	// 26 : for various tests
		AddLogValue(turbine_id, "Qg_min_partial", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "Qg_max_partial", "Nm");			// 28 : for various tests
		AddLogValue(turbine_id, "Nacelle_Roll_Accel_filt_y1", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "nacelle_roll_velo", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "Nacelle_Roll_Velo_bandfilt_y1", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "Nacelle_Roll_Velo_notch_filt2_y1", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "Nacelle_Roll_Velo_notch_filt3_y1", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "Nacelle_Roll_Velo_notch_filt4_y1", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "Nacelle_Roll_Velo_notch_filt_y1", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "yawErrorFilt", "rad");			// 27 : for various tests
		AddLogValue(turbine_id, "deltaPitchAngles", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "nonlinear_pitch_gain_component", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "aero_gain", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "time_exit_exzone", "s");			// 27 : for various tests
		AddLogValue(turbine_id, "omega_tower_eigenfrequency_interaction", "rad/s");			// 27 : for various tests
		AddLogValue(turbine_id, "check_exZoneState", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "ex_zone_sign", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "Pitch_FATowDamper_gain", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "GenT_latTowDamper_gain", "-");			// 27 : for various tests
		AddLogValue(turbine_id, "GetMeasuredYawError", "rad");			// 27 : for various tests
		AddLogValue(turbine_id, "exZone_torque_memory", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "IPC_theta1", "rad");			// 27 : for various tests
		AddLogValue(turbine_id, "IPC_theta2", "rad");			// 27 : for various tests
		AddLogValue(turbine_id, "IPC_tilt", "rad");			// 27 : for various tests
		AddLogValue(turbine_id, "IPC_yaw", "rad");			// 27 : for various tests
		AddLogValue(turbine_id, "M0", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "M1", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "M_tilt", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "M_yaw", "Nm");			// 27 : for various tests

		AddLogValue(turbine_id, "HPfilt_1P", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "Nfilt_inv_1P", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "Nfilt_3P", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "LPfilt_1P", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "Nfilt_inv_2P", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "BP_blade1_Rossiter", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "BP_blade2_Rossiter", "Nm");			// 27 : for various tests	
		AddLogValue(turbine_id, "blade1_flap_bending_moment", "Nm");			// 27 : for various tests
		AddLogValue(turbine_id, "blade2_flap_bending_moment", "Nm");			// 27 : for various tests		

		AddLogValue(turbine_id, "TeeterCoupledPitchAngle", "rad");			// 27 : for various testss		
		AddLogValue(turbine_id, "Blade1 Velo", "m/s");			// 27 : for various tests






		// initial state
		//nominalWindSpeed = GetNominalHubFlowSpeed(turbine_id);
		//PID_pit_var.outset1 = GetOptiPitch(nominalWindSpeed);
		// theta_min = GetOptiPitch(nominalWindSpeed);
// for (int i = 0; i < 2; i++) pitang[i] = SetDemandedPitchAngle(turbine_id, 0, theta_min);


		SetNamedUserVariable(turbine_id, "ControllerState", GetControllerState(turbine_id));  // is used to communicate with external load DLLs 


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
		meanpitang = 0;
		AllPitchLimitSwitchTripped = 0;
		for (int i = 0; i < nmbrBlades; i++) {
			pitang[i] = GetMeasuredPitchAngle(turbine_id, i); // Inp 3 // Returns the current pitch angle of the blade in rad.
			meanpitang = meanpitang + pitang[i] / nmbrBlades;
			AllPitchLimitSwitchTripped = AllPitchLimitSwitchTripped + IsPitchLimitSwitchTripped(turbine_id, i);

			// IPC - controller: Blade root flapwise moment and azimuth angle [in rad] 
			IPCvar.M_y[i] = GetMeasuredBladeOutOfPlaneBendingMoment(turbine_id, i);
			//bladeFLmoment[i] = GetMeasuredBladeStrainGaugeMy(turbine_id, i, 0);
			IPCvar.azimuth = GetMeasuredRotorAzimuthAngle(turbine_id);
		}
		pitchADC = pitchADC + abs(meanpitang - oldMeanpitang);
		oldMeanpitang = meanpitang;


		//################# tower FA damping filt ##################################
		//double NacelleForeAftAccel = GetMeasuredNacelleAccelerometerAccelerationX(turbine_id, 0);
		double Tower_FA_Accel = GetMeasuredTowerTopForeAftAcceleration(turbine_id);
		tower_FA_velo = tower_FA_velo + Tower_FA_Accel * deltat;

		//Tower_FA_Accel_filt.zeta1 = 0.1 * 12; // *6;
		//Tower_FA_Accel_filt.zeta2 = 0.001 * 12; // 6;
		//Tower_FA_Accel_filt.f0 = 0.195;
		//Tower_FA_Accel_filt = notch2orderfilt(deltat, stepno, Tower_FA_Accel_filt, Tower_FA_Accel);
		//double Nacelle_FA_Bandpass_filt = Tower_FA_Accel - Tower_FA_Accel_filt.y1;

		//################# tower SS damping filt ##################################
		double Tower_SS_Accel = GetMeasuredTowerTopSideSideAcceleration(turbine_id);
		tower_SS_velo = tower_SS_velo + Tower_SS_Accel * deltat;

		Tower_SS_Accel_filt.zeta1 = 0.1 * 12; // *6;
		Tower_SS_Accel_filt.zeta2 = 0.001 * 12; // 6;
		Tower_SS_Accel_filt.f0 = 0.195;

		Tower_SS_Accel_filt = notch2orderfilt(deltat, stepno, Tower_SS_Accel_filt, Tower_SS_Accel);
		double Nacelle_SS_Bandpass_filt = Tower_SS_Accel - Tower_SS_Accel_filt.y1;

		double nacelle_Roll_Accel = GetMeasuredNacelleRollAcceleration(turbine_id);
		nacelle_roll_velo = nacelle_roll_velo + nacelle_Roll_Accel * deltat;

		Nacelle_Roll_Accel_filt.zeta1 = 0.1; //* 12 * 2; // *6;
		Nacelle_Roll_Accel_filt.zeta2 = 0.001; //* 12*2; // 6;
		Nacelle_Roll_Accel_filt.f0 = 3.61; // 1.75;

		Nacelle_Roll_Accel_filt = notch2orderfilt(deltat, stepno, Nacelle_Roll_Accel_filt, nacelle_roll_velo); // nacelle_Roll_Accel);
		double Nacelle_Roll_Accel_Bandpass_filt = nacelle_Roll_Accel - Nacelle_Roll_Accel_filt.y1;

		// Bandpassfilt
		Nacelle_Roll_Velo_bandfilt.f0 = 1.75;
		Nacelle_Roll_Velo_bandfilt.zeta = 0.1;
		Nacelle_Roll_Velo_bandfilt = bandpassfilt(deltat, stepno, Nacelle_Roll_Velo_bandfilt, nacelle_roll_velo);


		// lowpassfilter nacelle roll
		Nacelle_Roll_Velo_LP_filt.f0 = 2; // 2; //frequency in Hz
		Nacelle_Roll_Velo_LP_filt.zeta = 0.7; //damping of filter
		Nacelle_Roll_Velo_LP_filt = lowpass2orderfiltFirstVal(deltat, stepno, Nacelle_Roll_Velo_LP_filt, nacelle_roll_velo);

		Nacelle_Roll_Velo_notch_filt2.f0 = 3.61; // 2; //frequency in Hz
		Nacelle_Roll_Velo_notch_filt2.zeta1 = 0.4; //damping of filter // seams to work well with 0.4 (wider notch if higher value)
		Nacelle_Roll_Velo_notch_filt2.zeta2 = 0.001; //damping of filter
		Nacelle_Roll_Velo_notch_filt2 = notch2orderfilt(deltat, stepno, Nacelle_Roll_Velo_notch_filt2, nacelle_roll_velo);

		Nacelle_Roll_Velo_notch_filt3.f0 = 3.61; // 2; //frequency in Hz
		Nacelle_Roll_Velo_notch_filt3.zeta1 = 0.4; //damping of filter
		Nacelle_Roll_Velo_notch_filt3.zeta2 = 0.001; //damping of filter
		Nacelle_Roll_Velo_notch_filt3 = notch2orderfilt(deltat, stepno, Nacelle_Roll_Velo_notch_filt3, nacelle_roll_velo);

		Nacelle_Roll_Velo_notch_filt4.f0 = 3.61; // 2; //frequency in Hz
		Nacelle_Roll_Velo_notch_filt4.zeta1 = 1.6; //damping of filter
		Nacelle_Roll_Velo_notch_filt4.zeta2 = 0.001; //damping of filter
		Nacelle_Roll_Velo_notch_filt4 = notch2orderfilt(deltat, stepno, Nacelle_Roll_Velo_notch_filt4, nacelle_roll_velo);

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






		// ---------------------------- switching on/off controller states, cut ins and shut downs ------------------------- //

		// calculate the mean yaw error of the last 10 seconds (thus 10s / e.g. 0.01s step length = 1000 samples) by the use of an exponential moving average (EMA)
		double time_period_for_yaw_mean_in_s = 10;
		double smoothing = 2 / (1 + time_period_for_yaw_mean_in_s / deltat);   // free to choose from 0 to 1; (2 in nominator gives the more recent values more weight)
		meanYawMisalignment = -GetMeasuredYawError(turbine_id) * smoothing + meanYawMisalignment * (1 - smoothing);


		//int generator_cutin_check = GetGeneratorContactor(turbine_id);		// Returns the generator contactor: 0=OFF, 1=MAIN, 2=LOW SPEED. 
		SetNamedUserVariable(turbine_id, "ControllerState", GetControllerState(turbine_id));  // -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP 
		if (GridlossDetected || StuckPitchDetected || apply_teeter_brake || t_cutout > 0) {
			SetNamedUserVariable(turbine_id, "ControllerState", 4);  // this will activate the teeter brake in the external load DLL
		}


		int controller_state = GetControllerState(turbine_id);		// -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP 
		//int generator_cutin_check = GetGeneratorContactor(turbine_id);		// Returns the generator contactor: 0=OFF, 1=MAIN, 2=LOW SPEED. 

		// old: if ((GetControllerState(turbine_id) == 4 || GetControllerState(turbine_id) == 5 || abs(GetMeasuredYawError(turbine_id)) > 1000 * pi / 180) && t_cutout == 0 && time > 0) { // -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP )
		if ((controller_state == 4 || controller_state == 5) && t_cutout == 0 && time > 0) { // -1=CONTROLLER_ERROR; 0=POWER_PRODUCTION; 1=PARKED; 2=IDLING; 3=START_UP; 4=NORMAL_STOP; 5=EMERGENCY_STOP )
			t_cutout = time;
			ReportInfoMessage(turbine_id, "Cut out simulation initiated");
			/* old: if (abs(GetMeasuredYawError(turbine_id)) > 1000 * pi / 180) { //low wind speeds tend to have higher than 180 deg of yaw error, thus do first a wind speed clause
				ReportInfoMessage(turbine_id, "Due to extreme yaw error of 1000 deg");
				extremeYawMisalignment = 1;
				t_cutout = time - t_cutout_delay / 3;
			}*/
		}
		else if (controller_state == 0 && GridlossDetected == 0 && GetGeneratorContactor(turbine_id) == 0) { // Returns the generator contactor: 0=OFF, 1=MAIN, 2=LOW SPEED. 
			SetNamedUserVariable(turbine_id, "ControllerState", 4);
			GridlossDetected = 1;
			t_cutout = time + 5; // +20;
			omega_ref_max = omegafilt * 0.9;
			ReportInfoMessage(turbine_id, "Grit loss is detected and cut out initiated");
			ReportInfoMessage(turbine_id, "pitch by PID for 7s, then smooth pitching out");
			PID_pit_var.Kpro[0] = PID_pit_var.Kpro[0] * 2;
			//PID_pit_var.Kint[0] = PID_pit_var.Kint[0] * 2;
			ReportInfoMessage(turbine_id, "increasing proportional pitch gain by factor 2");
		}
		else if (controller_state == 0 && StuckPitchDetected == 0 && AllPitchLimitSwitchTripped != 0) {
			//IsPitchLimitSwitchTripped(turbine_id, 0) + IsPitchLimitSwitchTripped(turbine_id, 1) + IsPitchLimitSwitchTripped(turbine_id, 2) > 0) {
			StuckPitchDetected = 1;
			t_cutout = time; // -t_cutout_delay / 3;
			//ReportInfoMessage(turbine_id, "Stuck Pitch is detected and cut out initiated");
			char* reportMessage = new char[55];// [3];
			sprintf_s(reportMessage, 55, "Stuck Pitch is detected! Cut out initiated at %.0f s", time);
			ReportInfoMessage(turbine_id, reportMessage);
			sprintf_s(reportMessage, 55, "Yaw Control is = %i", GetYawControl(turbine_id)); ReportInfoMessage(turbine_id, reportMessage); //Pitch control: 0 = collective, 1 = individual
			ReportInfoMessage(turbine_id, "0=RATE CONTROL, 1=TORQUE CONTROL");
			ReportInfoMessage(turbine_id, "Should be 0.0000");
			//SetOverrideYawRateWithTorque(turbine_id, 1);
			//sprintf_s(reportMessage, 55, "Yaw Control is = %f", GetYawControl(turbine_id)); ReportInfoMessage(turbine_id, reportMessage); //Pitch control: 0 = collective, 1 = individual
		}
		else if (StuckPitchDetected == 0 && extremeYawMisalignment == 0 && t_cutout == 0 && time > 0) { // Note: This is almost always triggered
			// stuck  pitch causes extreme yaw misalignement and thus has to be excluded from this protection function
			if (abs(meanYawMisalignment) > 42 * pi / 180) {
				apply_teeter_brake = 1;  // extreme yaw misalignment can cause severe teeter excursions.
				if (time - time_at_extremeYawMisalignment > 30) {
					ReportInfoMessage(turbine_id, "Extreme yaw error of >42 deg registered.\nWill yaw to 90 deg misalignment for shut down.");
					extremeYawMisalignment = 1;
				}
			}
			else {
				apply_teeter_brake = 0;
				time_at_extremeYawMisalignment = time;   // yaw to 90 degree misalignment should only be done if the extreme yaw misalignment definitely remains. Thus the time period trigger.
			}
		}


		if (omegafilt > omega_ref_max * maxOverspeedFactor && time - t_cutout_delay / 3 > 0) {
			if (t_cutout == 0) {
				t_cutout = time - t_cutout_delay / 3;
				char* reportMessage = new char[55];// [3];
				sprintf_s(reportMessage, 55, "Safety Shut Down! omega>n_r*%1.3f", maxOverspeedFactor);	ReportInfoMessage(turbine_id, reportMessage);
			}
			if (omegafilt > omega_ref_max * maxOverspeedFactor * 1.05) {//&& GetShaftBrakeStatusBinaryFlag(turbine_id) != 31) { // CHECK THE TRIGGER!!!! WHY DAS GetShaftBrakeStatusBinaryFlag != 31 NOT WORK?
				//ReportInfoMessage(turbine_id, "Emergency Shut Down! omega>n_r*1.20");
				SetShaftBrakeStatusBinaryFlag(turbine_id, 31);
			}
		}


		int ControllerFalureFlag = GetControllerFailureFlag(turbine_id);









		// Mean pitch angle
// Low - pass filtering of the mean pitch angle for gain scheduling
		pitchfirstordervar = lowpass1orderfilt(deltat, stepno, pitchfirstordervar, meanpitang);
		meanpitangfilt = fmin(pitchfirstordervar.y1, 30.0 * degrad);
		// Low - pass filtering of the nacelle wind speed
		wspfirstordervar = lowpass1orderfilt(deltat, stepno, wspfirstordervar, wsp);
		WSPfilt = wspfirstordervar.y1;
		// Low - pass filtering of the nacelle yaw missalignement GetMeasuredYawError(turbine_id)
		yawfirstordervar = lowpass1orderfilt(deltat, stepno, yawfirstordervar, GetMeasuredYawError(turbine_id));
		double yawErrorFilt = yawfirstordervar.y1;
		//double AngleFromNord = GetMeasuredYawError(turbine_id);

		// get current FA damper gain
		if (factor_Pitch_FATowDamper == 1) Pitch_FATowDamper_gain = GetInterpolatedValue(WSPfilt, FA_damper_gains_look_up) * factor_Pitch_FATowDamper; // factor_Pitch_FATowDamper = 1 by default or 0 if no FA damper should be used // use flexible for gain >0.5??
		else Pitch_FATowDamper_gain = factor_Pitch_FATowDamper;
		// get current StS damper gain
		if (factor_GenT_latTowDamper == 1) GenT_latTowDamper_gain = GetInterpolatedValue(WSPfilt, StS_damper_gains_look_up) * factor_GenT_latTowDamper; // factor_GenT_latTowDamper = 1 by default or 0 if no StS damper should be used
		else GenT_latTowDamper_gain = factor_GenT_latTowDamper;
		if (IgainIPC_1P == 1) IgainIPC_2P = GetInterpolatedValue(WSPfilt, LIPC_predef_gains); // factor_GenT_latTowDamper = 1 by default or 0 if no StS damper should be used

		// LIPC 123P param interpolation
		if (use2B_teetering == 4) GA_Parameter1 = GetInterpolatedValue(WSPfilt, LIPC_predef_gains_1P);
		if (use2B_teetering == 4) GA_Parameter2 = GetInterpolatedValue(WSPfilt, LIPC_predef_gains_2P);
		if (use2B_teetering == 4) GA_Parameter3 = GetInterpolatedValue(WSPfilt, LIPC_predef_gains_3P);


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
		ommax1 = (2.0 * rel_sp_open_Qg - 1.0) * omega_ref_max;
		ommax2 = rel_sp_open_Qg * omega_ref_max;
		x = switch_spline(omegafilt, ommin1, ommin2);
		Qg_min_partial = fmin(Kopt * omegafilt * omegafilt * x, Kopt * ommax1 * ommax1);
		x = switch_spline(omegafilt, ommax1, ommax2);
		Qg_max_partial = fmax(Kopt * omegafilt * omegafilt * (1.0 - x) + Qg_max_full * x, Kopt * ommin2 * ommin2);


		// ########################################################################################
		// ######################### START SPEED EXCLUSION ZONE ###################################
		// ########################################################################################

		// notchfilter exclusion zone rotor speed???
		Ex_mode_filt.zeta1 = 0.1 * 12; // *6;
		Ex_mode_filt.zeta2 = 0.001 * 12; // 6;
		Ex_mode_filt.f0 = 0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
		//if (nmbrBlades == 2)	Ex_mode_filt.f0 = 0.124;//  0.17; // 0.162; // Hz value of filter notch filtered frequency (main frequency of rotation oscillation seems to be coupled to tower fore-aft)
		//if (use90mpsVersion != 1) Ex_mode_filt.f0 = 0.124;
		Ex_mode_filt = notch2orderfilt(deltat, stepno, Ex_mode_filt, omegafilt);
		//SetLoggingValue(turbine_id, 29, Ex_mode_filt.y1);

		//Ex_mode_filt2.f0 = 0.170; //0.179 // Hz value of filter notch filtered frequency
		//Ex_mode_filt2 = notch2orderfilt(deltat, stepno, Ex_mode_filt2, omegafilt);
		//SetLoggingValue(turbine_id, 30, Ex_mode_filt2.y1);



		double omega4exZone = Ex_mode_filt.y1; // omegafilt; // omega or omegafilt ???
		//double omega4exZone = omegafilt;


// ############################################### ENTER THE TOWER EIGENFREQUENCY ##############################################//
		double omega_tower_eigenfrequency_interaction = towerFrequencyToAvoid / nmbrBlades * 2 * pi; //0.454209003; // in omega rad/s ( = tower_eigenfrequency * (2*pi) / nmbr_of_blades)
		// ############################################### ENTER THE TOWER EIGENFREQUENCY ##############################################//
		//double rel_omega_exclusion_zone = 0.15; // 0.15; //0.10; // in % below / above ( note, that fore-aft and side-side are already 3% away from middle value)
		double check_exZoneState = 0; // check_two = 0;
		double exZoneSwitchWidth = 0.05; // in %
		double exZone_Torque_limit_increase = 0.1; // 0.30; // increases torque limit in %

		double omega_start_lower_exzone = omega_tower_eigenfrequency_interaction * (1 - (rel_omega_exclusion_zone + exZoneSwitchWidth));
		double omega_start_upper_exzone = omega_tower_eigenfrequency_interaction * (1 + (rel_omega_exclusion_zone + exZoneSwitchWidth));

		double lower_exZone_torque_limit = Kopt * pow(omega_start_lower_exzone, 2);
		double upper_exZone_torque_limit = Kopt * pow(omega_start_upper_exzone, 2);

		if (towerFrequencyToAvoid > 0 && t_cutout == 0 && time > 5) { // Check if exZone should be used in generel

			// using three splines with actual omega as trigger between two values. Explanation for comming from beneath exclusion zone (from above is reziprogual) 
			// 1.) increase torque by PI-controller for constant rotation speed, until maximum zone torque is reached. ((if minimum zone limit is reached, an soft exit takes place))
			// 2.) excelerate rotation speed by vastly reducing torque
			// 3.) catch up with usual K_opt torque value
			if (abs(omega4exZone - omega_tower_eigenfrequency_interaction) < omega_tower_eigenfrequency_interaction * (rel_omega_exclusion_zone + exZoneSwitchWidth * abs(ex_zone_sign))) {

				if (ex_zone_sign == 0) { // first time entering speed exclusion zone
					ex_zone_sign = copysign(1.0, omega4exZone - omega_tower_eigenfrequency_interaction);
					if (ex_zone_sign > 0) { sprintf_s(reportMessage, 55, "Entering speed exclusion zone from above! At %.0f s", time);	ReportInfoMessage(turbine_id, reportMessage); }
					else { sprintf_s(reportMessage, 55, "Entering speed exclusion zone from below! At %.0f s", time);	ReportInfoMessage(turbine_id, reportMessage); }

					// for smooth transition in PI-Torque-control
					time_exit_exzone = time - time_needed_for_exiting;
					exZone_torque_memory = PID_gen_var.outres;
					check_exZoneState = 1;
				}
				// calculating rotation speed error for PI-Torque controller
				//omega_err_filt_speed = omega4exZone - omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign);
				omega_err_filt_speed = (omega4exZone - omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign)) * 4; // *5;  // SHOULDN'T THE MULTIPLYER 4 BE IN THE PI-GAINS OF THE TORQUE CONTROLLER????? (Maybe better like this, because the transition region needs a less aggressive PI-Torque)
				// omega_err_filt_speed = (pow(omega4exZone, 2) - pow(omega_tower_eigenfrequency_interaction * (1 + rel_omega_exclusion_zone * ex_zone_sign), 2)) * 10;

				// Limits for partial load that gives room for constant speed PI-torque controller in exclusion zone
				//Qg_max_partial = upper_exZone_torque_limit;
				//Qg_min_partial = lower_exZone_torque_limit;

				// use spline to enter PI-Torque zone more softly (after time > time_exit_exzone + time_needed_for_exiting*2;   x=1)
				x = switch_spline(time, time_exit_exzone + time_needed_for_exiting, time_exit_exzone + time_needed_for_exiting * 2);
				Qg_max_partial = fmax(Kopt * omegafilt * omegafilt, exZone_torque_memory * (1 - x) + upper_exZone_torque_limit * x);
				Qg_min_partial = fmin(Kopt * omegafilt * omegafilt, exZone_torque_memory * (1 - x) + lower_exZone_torque_limit * x);

				if (time > time_exit_exzone + time_needed_for_exiting) { // takes time_needed_for_exiting to exit by spline wave and should not be interrupted
					if (ex_zone_sign > 0) {
						if (PID_gen_var.outres1 < lower_exZone_torque_limit * 1.01 && omega4exZone > omega_tower_eigenfrequency_interaction) {
							//check if necessary to cross the tower eigenfrequency && only needed if not already crossed (in case of e.g. a ramp or wind gust)
						//if (PID_gen_var.outres1_old < lower_exZone_torque_limit * 1.01) {
							sprintf_s(reportMessage, 55, "Spline-Exiting Exclusion Zone Downwards at %.0fs", time);	ReportInfoMessage(turbine_id, reportMessage);
							time_exit_exzone = time;							   //starts spline exit
							transition_exZone_Qg_min_partial = PID_gen_var.outres1; // for transition/start of spline function
							ex_zone_sign = ex_zone_sign * (-1);					   // change the safty zone side for the transition at the ende of the spline exit
						}
					}
					else {
						if (PID_gen_var.outres1 > upper_exZone_torque_limit * 0.99 && omega4exZone < omega_tower_eigenfrequency_interaction) {
							//check if necessary to cross the tower eigenfrequency && only needed if not already crossed (in case of e.g. a ramp or wind gust)
						//if (PID_gen_var.outres1_old > upper_exZone_torque_limit * 0.99) {//WHY OUTRES1_OLD??? IT IS THE OUTRES FROM 2 TIME STEPS AGO.... AND BY 1.01 or 0.99 ??
							sprintf_s(reportMessage, 55, "Spline-Exiting Exclusion Zone Upwards   at %.0fs", time);	ReportInfoMessage(turbine_id, reportMessage);
							time_exit_exzone = time;							   //starts spline exit
							transition_exZone_Qg_min_partial = PID_gen_var.outres1; // for transition/start of spline function
							ex_zone_sign = ex_zone_sign * (-1);					   // change the safty zone side for the transition at the ende of the spline exit
						}
					}
				}
				else {
					//if (time == time_exit_exzone) {	transition_exZone_Qg_min_partial = PID_gen_var.outres; // for transition/start of spline function
					//	ex_zone_sign = ex_zone_sign * (-1); } // change the safty zone side, when exiting by spline

					x = switch_spline(time, time_exit_exzone, time_exit_exzone + time_needed_for_exiting);
					//Qg_min_partial = transition_exZone_Qg_min_partial * (1 - x) + Kopt * pow(omega_tower_eigenfrequency_interaction, 2) * x;  //Works Good!!! (fixed "spline wave" peak torque)
					Qg_min_partial = transition_exZone_Qg_min_partial * (1 - x) + Kopt * pow(omega4exZone, 2) * x; //Better?-> Wave peak flexible to in/decrease rotation speed when necessary.

					Qg_max_partial = Qg_min_partial;
					exZone_torque_memory = PID_gen_var.outres1;
					check_exZoneState = 3;
				}
			}
			else {
				check_exZoneState = 0;
				if (ex_zone_sign != 0) {
					sprintf_s(reportMessage, 55, "Rotor speed drifted outside Exclusion Zone at %.0fs", time);	ReportInfoMessage(turbine_id, reportMessage);
					check_exZoneState = 4;
				}
				ex_zone_sign = 0;
			}
		}
		// ########################################################################################
		// ########################### END SPEED EXCLUSION ZONE ###################################
		// ########################################################################################





		// Switch based on pitch
		//switch1 = switch_spline(meanpitang, switch1_pitang_lower, switch1_pitang_upper); // ORIGINAL (Not possible with FA damper in partial load conditions)
		switch1 = switch_spline(PID_pit_var.outres, switch1_pitang_lower, switch1_pitang_upper);
		switchfirstordervar = lowpass1orderfilt(deltat, stepno, switchfirstordervar, switch1);
		switch1 = switchfirstordervar.y1;

		if (t_cutin > 0.0) { // During start up pitch angle is quite big but min torque shouldn't be forced to be rated torque
			if (generator_cutin) {
				switch1 = switch1 * switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay);
			}
		}

		// Interpolation between partial and full load torque limits based on switch 1
		PID_gen_var.outmin = (1.0 - switch1) * Qg_min_partial + switch1 * Qg_min_full;
		PID_gen_var.outmax = (1.0 - switch1) * Qg_max_partial + switch1 * Qg_max_full;
		if (PID_gen_var.outmin > PID_gen_var.outmax) PID_gen_var.outmin = PID_gen_var.outmax;

		//double K_t_SS_damp = 100000000;
		//if (useGenTorqueTowerSS_Damper) if (time > 15) omega_err_filt_speed = omega_err_filt_speed + K_t_SS_damp * Tower_SS_Accel;

		//############################################################################ TORQUE-PI-CONTROLLER!!!! #################################################################################################################################
		//Compute PID feedback to generator torque
		kgain_torque[0] = 1.0;
		kgain_torque[1] = 1.0;
		kgain_torque[2] = 1.0;
		PID_gen_var = PID(stepno, deltat, kgain_torque, PID_gen_var, omega_err_filt_speed);
		Qgen_ref = PID_gen_var.outres;
		//############################################################################ TORQUE-PI-CONTROLLER!!!! #################################################################################################################################


		//------------------------------------------------------------------------------------------------//
		//									Side-to-Side damping										  //
		//------------------------------------------------------------------------------------------------//

		//if (useGenTorqueTowerSS_Damper) if (time > start_time_towerSSdamper) Qgen_ref = Qgen_ref + K_t_SS_damp * Tower_SS_Accel * (pow(time, 3) - pow(start_time_towerSSdamper - deltat, 3)) / pow(time, 3);

		double K_t_StS_damp = Qg_rated * 100 * GenT_latTowDamper_gain;// for soft switch: *(pow(time, 3) - pow(start_time_towerSSdamper - deltat, 3)) / pow(time, 3); int start_time_towerSSdamper = 15;

		//if (time > 15) Qgen_ref = Qgen_ref - nacelle_roll_velo * K_t_StS_damp; // default: K_t_StS_damp = Qg_rated * 100
		//if (time > 15) Qgen_ref = Qgen_ref - copysign(pow(nacelle_roll_velo, 2), nacelle_roll_velo) * K_t_StS_damp*2000; // default: K_t_StS_damp = Qg_rated * 200000


		//double Qgen_StS_damp = nacelle_Roll_Accel * K_t_StS_damp/20;// *150;
		//double Qgen_StS_damp = - nacelle_roll_velo * K_t_StS_damp *2; // default: K_t_StS_damp = Qg_rated * 100 and  *200 well for 2B90 //THIS SEEMS TO BE good!
		//double Qgen_StS_damp = -Nacelle_Roll_Velo_LP_filt.y1 * K_t_StS_damp * 2;
		//double Qgen_StS_damp = -Nacelle_Roll_Accel_filt.y1 * K_t_StS_damp * 2;


		double Qgen_StS_damp = -Nacelle_Roll_Velo_notch_filt3.y1 * K_t_StS_damp; //THIS IS BEST!!!!!!
		//double Qgen_StS_damp = -nacelle_roll_velo * K_t_StS_damp; //THIS IS newest
		//double Qgen_StS_damp = tower_SS_velo * K_t_StS_damp / 100; //Was default for last runs!!!



		//double Qgen_StS_damp = Nacelle_Roll_Accel_Bandpass_filt * K_t_StS_damp/40; // for neg: /20

		//double Qgen_StS_damp = Nacelle_Roll_Velo_bandfilt.y1 * K_t_StS_damp; //does not work well, very unstable


		if (Qgen_StS_damp > Qg_rated * 1.15) Qgen_StS_damp = Qg_rated * 0.15;    //0.15% above rated torque is regarded as a maximum
		if (Qgen_StS_damp < -Qg_rated)	 Qgen_StS_damp = 0; // something has to be very wrong for this trigger!

		if (time > 15) { // (time > 15 && t_cutout == 0)
			if (true) {  // default is true
				Qgen_ref = Qgen_ref + Qgen_StS_damp;
			}
			else { 
				// bode designed StS damping compensator 
				GenT_StS_damp_PP.f1 = 0.168; // towerFrequencyToAvoid; // tower StS frequency
				GenT_StS_damp_PP.zeta = 0.1;  // damping -- magnitude is not exactly important
				GenT_StS_damp_PP = bandpassfilter_boris(deltat, stepno, GenT_StS_damp_PP, tower_SS_velo, true); // lowpass
				Qgen_ref = Qgen_ref + GenT_StS_damp_PP.y1 * 8e5 * GenT_latTowDamper_gain;
			}
		}

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

		if (t_cutout > 0.0 && time > t_cutout) {
			if (Init_cutout_Qgen_ref == 0) Init_cutout_Qgen_ref = Qgen_ref / omegafilt;
			//if (time > t_cutout + 5) x = switch_spline(time, t_cutout +5, t_cutout +5 + t_cutout_delay);
			//else x = 0;
			Qgen_ref = Init_cutout_Qgen_ref * omegafilt; //* (1 - x);
			if (StuckPitchDetected) {	// Bump up Qgen if pitch is stuck, to decrease rotation speed faster
				x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 3);
				Qgen_ref = omegafilt * Init_cutout_Qgen_ref * (1 + 0.1 * x);
			}
			//// torquefirstordervar = lowpass1orderfilt(deltat, stepno, torquefirstordervar, 0.0);
			//// Qgen_ref = torquefirstordervar.y1;
		}
		////else {	torquefirstordervar = lowpass1orderfilt(deltat, stepno, torquefirstordervar, Qgen_ref);}

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

		//Aerodynamic gain scheduling!
		if (kk2 > 0.0)
			aero_gain = 1.0 + meanpitangfilt / kk1 + meanpitangfilt * meanpitangfilt / kk2;
		else aero_gain = 1.0 + meanpitangfilt / kk1;

		// Nonlinear gain to avoid large rotor speed excursion
		double nonlinear_pitch_gain_component = (omega_err_filt_pitch * omega_err_filt_pitch / pow(omega_ref_max * (rel_limit - 1.0), 2) + 1.0);
		help_var = (omega_err_filt_pitch * omega_err_filt_pitch / pow(omega_ref_max * (rel_limit - 1.0), 2) + 1.0) / aero_gain;
		//help_var = 1 / aero_gain;

		// It is most important to limit the gain sceduling in the partial load region (otherwise its smaller then 1 anyhow) for floating tubrines to avoid system instabilities cause by too high proportional pitch gains. (Could be tested if it is suitable for bottom-fixed turbines as well)
		if (useFloatingTurbine) help_var = fmin(help_var, 1.5);
		
		kgain_pitch[0][0] = help_var; // will be the gain scheduling for normal power production, if no grid loss, cut in etc. occures
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
					PID_pit_var.outmin = pitch_stopang * (1 - x);
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
					if (abs(x) < omega_ref_min * 0.20) // CHANGED: * 0.01)
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
					omega_err_filt_pitch = omegafilt - (omega_ref_min * (1.0 - x) + omega_ref_max * x);
					kgain_pitch[0][0] = adapt_non_lin_pitch_gain * kgain_pitch[0][0] + kgain_pitch[0][0] * (1 - adapt_non_lin_pitch_gain) * x;
					kgain_pitch[1][0] = adapt_non_lin_pitch_gain * kgain_pitch[1][0] + kgain_pitch[1][0] * (1 - adapt_non_lin_pitch_gain) * x;
					kgain_pitch[2][0] = adapt_non_lin_pitch_gain * kgain_pitch[2][0] + kgain_pitch[2][0] * (1 - adapt_non_lin_pitch_gain) * x;
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
		if (t_cutout > 0.0 && time > t_cutout) // + pitch_stopdelay) WHY SHOULD THERE BE A PITCH DELAY??? (Delay of actuator is included in Bladed) // || GridlossDetected == 1 )
		{
			switch (pitch_stoptype) {
			case 1: //Normal 2 - step stop situation
				if (tellItOnce < 3) {

					tellItOnce = 3;

					if (GridlossDetected == 1) {
						pitch_stopvelmax2 = pitch_stopvelmax2 / omegafilt / 4; // / 4;
						// t_cutout_delay = t_cutout_delay *2;
					//} else if (StuckPitchDetected == 1) {
					//		pitch_stopvelmax2 = pitch_stopvelmax2 / omegafilt / 2;
					}
					else if (GetControllerState(turbine_id) == 5) {
						ReportInfoMessage(turbine_id, "Emergency stop initiated");
						pitch_stopvelmax2 = pitch_stopvelmax / omegafilt;
						t_cutout_delay = t_cutout_delay / 1.5;
						t_cutout = t_cutout - t_cutout_delay / 5;
					}
					else {
						pitch_stopvelmax2 = pitch_stopvelmax2 / omegafilt;
						ReportInfoMessage(turbine_id, "Normal 2-step stop");
					}

				}
				PID_pit_var.outmax = pitch_stopang;
				PID_pit_var.outmin = pitch_stopang;

				x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay);
				PID_pit_var.velmax = pitch_stopvelmax2 * x * fmax(omegafilt, omega_ref_min); // multiply omegafilt for pitch sensivity tuning

				if (StuckPitchDetected == 1) {
					PID_pit_var.velmax = 0; // prohibiting other pitch actuators to move for reduced unbalances
					if (GetYawControl(turbine_id) == 0) {
						if (abs(GetMeasuredNacelleAngleFromNorth(turbine_id) * raddeg) < 90) {
							//yawRef = - yaw_stopvelmax * omegafilt / omega_ref_max/3;
							x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 5);
							yawRef = -fmin(yaw_stopvelmax * omegafilt / omega_ref_max / 3 * x, yaw_stopvelmax);
						}
						else yawRef = 0;
					}
					else if (GetYawControl(turbine_id) == 1) ReportInfoMessage(turbine_id, "Not yet tested! Set yaw control to rate!");
					else ReportInfoMessage(turbine_id, "Something is rotten in the state of Denmark");
					//SetDemandedYawActuatorTorque(turbine_id, yaw_stopvelmax * 100);
					//SetDemandedYawMotorRate(turbine_id, 0);
				}

				//if (GridlossDetected == 1) x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay*2);

				/*if (GridlossDetected == 1) {
					x = fmax(0.01,switch_spline(time, t_cutout + t_cutout_delay / 3, t_cutout + t_cutout_delay * (1+1/3)) + (1 - switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 4)) );
					// x = switch_spline(time, t_cutout+ t_cutout_delay/4.1, t_cutout + t_cutout_delay) + (1 - switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 4));
					x2 = 1 - switch_spline(time, t_cutout, t_cutout + t_cutout_delay / 4);
					// x = switch_spline(time, t_cutout - t_cutout_delay, t_cutout + t_cutout_delay * 2) + switch_spline(t_cutout, time, t_cutout + t_cutout_delay/4);

					// x = switch_spline(time, t_cutout - t_cutout_delay, t_cutout + t_cutout_delay * 2) * 2 - switch_spline(time, t_cutout - t_cutout_delay/2, t_cutout + t_cutout_delay);
					// x = switch_spline(time, t_cutout - t_cutout_delay, t_cutout + t_cutout_delay*2);  //x = switch_spline(time, t_cutout, t_cutout + t_cutout_delay/5);
				}*/




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
				ReportInfoMessage(turbine_id, " *** ERROR *** Stop type is not known");
				break;
			}
		}



		//------------------------------------------------------------------------------------------------//
		//							Compute PID feedback to generator torque							  //
		//------------------------------------------------------------------------------------------------//

		if (DT_mode_filt.f0 > 0.0) {
			DT_mode_filt = notch2orderfilt(deltat, stepno, DT_mode_filt, omega_err_filt_pitch);
			e_pitch[0] = DT_mode_filt.y1;
			pwr_DT_mode_filt = notch2orderfilt(deltat, stepno, pwr_DT_mode_filt, Pe_ref - Pe_rated);
			e_pitch[1] = pwr_DT_mode_filt.y1;
		} 
		else {
			e_pitch[0] = omega_err_filt_pitch;
			e_pitch[1] = Pe_ref - Pe_rated;
			if (filtFreqPowerError_pitch > 0.0) {		// might be good to filter power error as well
				if (tellItOnce == 0) ReportInfoMessage(turbine_id, "Filtering power error for pitch PI controller.");
				pitch_power_error_filt.zeta1 = 0.1 * 12; // *6;
				pitch_power_error_filt.zeta2 = 0.001 * 12; // 6;
				pitch_power_error_filt.f0 = filtFreqPowerError_pitch;
				pitch_power_error_filt = notch2orderfilt(deltat, stepno, pitch_power_error_filt, e_pitch[1]);
				e_pitch[1] = pitch_power_error_filt.y1;
			}
			// if (usePitchTowerFA_Damper) if (time > 15) e_pitch[0] = omega_err_filt_pitch + K_t_FA_damp * tower_FA_velo; // copysign(pow(tower_FA_velo, 2) * K_t_FA_damp, tower_FA_velo); double K_t_FA_damp = 0.1;
		}
		// USE FOR INDIRECT FORE-AFT TOWER DAMPING BY PITCH WITH AN ARTIFICIAL ROTATION SPEED ERROR: (If Pitch is <0, pitch stays 0 and not negative)
		//if (time > 15  && t_cutout == 0) e_pitch[0] = omega_err_filt_pitch + tower_FA_velo * Pitch_FATowDamper_gain; // copysign(pow(tower_FA_velo, 2) * K_t_FA_damp, tower_FA_velo); // works well with factor = 0.1 BEST!!!!!

		double pitch_unfiltered = 0;
		double pitch_filtered = 0;
		if (useFloatingTurbine) { //(time > 15 && t_cutout == 0 && useFloatingTurbine) {
				if (tellItOnce == 0) ReportInfoMessage(turbine_id, "Manipulating rotor reference speed of pitch PI for FA damping of FOWTs.");

				// use a high pass filter to avoid mean zero drifting of the velocity signal
				FOWT_nacelle_FA_velo_DC_blocker.f2 = 0.9997; // should be between 0 and 1
				//FOWT_nacelle_FA_velo_DC_blocker = DC_blocker(stepno, FOWT_nacelle_FA_velo_DC_blocker, GetMeasuredTurbineAccelerometerRotationalAccelerationY(turbine_id, 0));  // high pass
				FOWT_nacelle_FA_velo_DC_blocker = DC_blocker(stepno, FOWT_nacelle_FA_velo_DC_blocker, GetMeasuredTowerTopForeAftAcceleration(turbine_id));  // Use for FA damping!!
				nacelle_FA_velocity += FOWT_nacelle_FA_velo_DC_blocker.y1 * deltat;

				// low pass filter the signals to block unwanted oscillations and adapt a suitable amount of phase shift
				FOWT_Nacelle_FA_velo_2nd_order_Butterworth_LP_filt.f0 = GA_Parameter2; // 0.05; // complex pole ->  Assuming that the waves have most of their energy below 15.0 s, and that platform natural periods will typically be longer than 25.0 s, it would be reasonable in this case to set the cut - off frequency at 2 pi / 20 ^= 0.31 rad / s.Platforms with longer pitch natural periods will benefit from having lower cut - off frequencies, resulting in a more effective filtering action.
				FOWT_Nacelle_FA_velo_2nd_order_Butterworth_LP_filt.zeta = 0.707; // default damping of the complex pole
				FOWT_Nacelle_FA_velo_2nd_order_Butterworth_LP_filt = lowpass2orderfiltFirstVal(deltat, stepno, FOWT_Nacelle_FA_velo_2nd_order_Butterworth_LP_filt, nacelle_FA_velocity); // complex pole
				nacelle_FA_velocity_filtered = FOWT_Nacelle_FA_velo_2nd_order_Butterworth_LP_filt.y1;

				// adding low-pass filtered platform pitch angular velocity and surge velocity to the setpoint rated rotorspeed of the pitch PID rotor speed error to achieve a "feedforward" method damping
				e_pitch[0] = omega_err_filt_pitch + nacelle_FA_velocity_filtered * GA_Parameter1; // +platform_surge_velocity_filtered * GA_Parameter3;
		}

		if (stepno == 1) {
			PID_pit_var.outset1 = GetMeasuredPitchAngle(turbine_id, 0);
			ReportInfoMessage(turbine_id, " CHANGE INITIAL PITCH ANGLE WHEN CONTROLLER DESIGN IS FROZEN");
			PID_pit_var.outset1 = GetOptiPitch(nominalWindSpeed); // initial pitch state in full load operation TEST UF THIS IST NEEDED OR WHETHER THE FULL GetOptiPitch CAN BE ERASED!!!
		}
		
		PID_pit_var = PID2(stepno, deltat, kgain_pitch, PID_pit_var, e_pitch);//, GetOptiPitch(nominalWindSpeed));
		theta_col_ref = PID_pit_var.outres;
		//if (time < t_cutin) theta_col_ref = pitch_stopang; // #QUICKNDIRTY


		// USE FOR DIRECT FORE-AFT TOWER DAMPING BY PITCH WITH AN UNFILTERED OVERLAYER OF THE BASIC PITCH SIGNAL FROM TOWER FORE-AFT VELOCITY
		if (time > 15 && t_cutout == 0 && useFloatingTurbine == false) {
			theta_col_FA_damp = tower_FA_velo * Pitch_FATowDamper_gain;
		}
		theta_col_ref = theta_col_ref + theta_col_FA_damp;  // to guarantee a smooth exit of FA damping during cutout (theta_col_FA_damp then stays constant if t > t_cutout)




		//################# FILTERING PITCH SIGNAL? #####################
		if (towerfrequencyForPitchFilt != 0) {
			pitch_out_signal_filt.zeta1 = 0.1 * 12; // *6;
			pitch_out_signal_filt.zeta2 = 0.001 * 12; // 6;
			pitch_out_signal_filt.f0 = towerfrequencyForPitchFilt;
			pitch_out_signal_filt = notch2orderfilt(deltat, stepno, pitch_out_signal_filt, theta_col_ref);
			theta_col_ref = pitch_out_signal_filt.y1;
		}
		//################# FILTERING PITCH SIGNAL? #####################

		if (StuckPitchDetected == 1) { // set all functioning pitch actuators to stuck pitch value
			theta_col_ref = IsPitchLimitSwitchTripped(turbine_id, 0) * GetMeasuredPitchAngle(turbine_id, 0) + IsPitchLimitSwitchTripped(turbine_id, 1) * GetMeasuredPitchAngle(turbine_id, 1);
			if (nmbrBlades == 3) theta_col_ref = theta_col_ref + IsPitchLimitSwitchTripped(turbine_id, 2) * GetMeasuredPitchAngle(turbine_id, 2);
		}

		// Secure shut down for pitch run away:
		// hier muss das IPC ergänzt werden.! (Pitches gehen auseinander)
		if (1 == 2) { // abs(GetMeasuredPitchAngle(turbine_id, 0) - GetMeasuredPitchAngle(turbine_id, 1)) > eps_pitchError) {
			if (eps_pitchError != 0) ReportInfoMessage(turbine_id, "Second pitch running away with first");
			eps_pitchError = 0;
			theta_col_ref = fmax(GetMeasuredPitchAngle(turbine_id, 0), GetMeasuredPitchAngle(turbine_id, 1));
			if (t_cutout == 0) { t_cutout = time; ReportInfoMessage(turbine_id, "*** Simulated pitch to feather fault, see DLCs 2!"); }
		}

		if (fixedPitchAng >= 0) {
			if (time < 140) fixedPitchAng = theta_col_ref;
			if (time > 140) theta_col_ref = fixedPitchAng; // *degrad;
			if (time > 150) {
				theta_col_ref = fixedPitchAng + 0.5 * degrad;
				Qg_rated = Qg_rated * (1 - omega_err_filt_pitch);
			}
		}

		if (usePitchRunAwayAt > 0 && usePitchRunAwayAt < time) thetaref[0] = fmin(thetaref[0] + 7.071 * degrad * deltat, 90 * degrad);
		else thetaref[0] = theta_col_ref;
		thetaref[1] = theta_col_ref;
		thetaref[2] = theta_col_ref;

		// -------------------------------------------------------------------------------------------------- //
		// ------------------------------- linear individual pitch control (LIPC) --------------------------- //
		// -------------------------------------------------------------------------------------------------- //
		double M0 = 0;
		double M1 = 0;
		double M_tilt = 0;
		double M_yaw = 0;
		double theta_tilt = 0;
		double theta_yaw = 0;
		double Azimuth_angle = GetMeasuredRotorAzimuthAngle(turbine_id);
		// blade root My calculation:
		double Blade1_My = GetMeasuredBladeOutOfPlaneBendingMoment(turbine_id, 0) * cos(GetMeasuredPitchAngle(turbine_id, 0))
			+ GetMeasuredBladeInPlaneBendingMoment(turbine_id, 0) * sin(GetMeasuredPitchAngle(turbine_id, 0));
		double Blade2_My = GetMeasuredBladeOutOfPlaneBendingMoment(turbine_id, 1) * cos(GetMeasuredPitchAngle(turbine_id, 1))
			+ GetMeasuredBladeInPlaneBendingMoment(turbine_id, 1) * sin(GetMeasuredPitchAngle(turbine_id, 1));
		// pure out of plane blade My (might be better due to reducing the influnence of an increasing blade pitch on gravitation driven My loads, which can make the IPC system unstable)

		// ------------------ TUNED BY BODEPLOTS WITH BORIS F. --------------------- //	
		if (use2B_teetering == 4) { // && IgainIPC_1P>0) {
			if (tellItOnce == 0) ReportInfoMessage(turbine_id, "Using 1P,3P LIPC Algorithm. Source: Edwin van Solingen () + BP, Lead filter tuning from Boris");
			if (tellItOnce == 0) ReportInfoMessage(turbine_id, "EXTREME CUSTOM! Param 1-3 are gains for 1-3P and 21-28+8 are LP damping+Lead/Lag frequency for 1-3P");
			M0 = Blade1_My * 1 / 2 + Blade2_My * 1 / 2;   // collective mode:   2P
			M1 = -Blade1_My * 1 / 2 + Blade2_My * 1 / 2; // differential mode: 1P and 3P harmonics

			// Boris tuned LIPC
			double oneP_Hz = omegafilt / 2 / pi; // 0.125; // 1P Hz
			if (true) { //1P best
				double K_p = 3e-11 * GA_Parameter1; // current best with * 14; // GA_Parameter1; // org4: 1e-10 * 4; // org2: 3e-11*4; // org: 1.7e-10*4; // IgainIPC_1P;
				double BP_damp = 0.015;

				LIPC_1P_BP.f1 = oneP_Hz; // 0.125; // 1P Hz
				LIPC_1P_BP.zeta = BP_damp; // 0.06; // org3: 0.15; // org2: 0.02; // org: 0.05;  // damping
				LIPC_1P_BP = bandpassfilter_boris(deltat, stepno, LIPC_1P_BP, M1 * K_p);

				// if the pole is larger than the zero it is a phase lead; otherwise it is a phase lag
				LIPC_1P_lead.f_zero = 0.3 / 2 / pi; // org1&2&4: 0.3 / 2 / pi; // frequency of the zero in Hz
				LIPC_1P_lead.f_pole = 1.0 / 2 / pi; //0.9 / 2 / pi; // org1&2: 1.0 / 2 / pi; // frequency of the pole in Hz
				LIPC_1P_lead = leadlag_filter(deltat, stepno, LIPC_1P_lead, LIPC_1P_BP.y1);
			}
			if (true) { //only 3P (for working in parallel)
				double K_p_3P = 8e-12 * GA_Parameter3; // current best with * 4; // GA_Parameter1; // org4: 1e-10 * 4; // org2: 3e-11*4; // org: 1.7e-10*4; // IgainIPC_1P;
				double BP_damp_3P = 0.015;

				LIPC_3P_PP1.f1 = oneP_Hz * 3; // 0.125; // 1P Hz
				LIPC_3P_PP1.zeta = BP_damp_3P; // 0.06; // org3: 0.15; // org2: 0.02; // org: 0.05;  // damping
				LIPC_3P_PP1 = bandpassfilter_boris(deltat, stepno, LIPC_3P_PP1, M1 * K_p_3P);

				LIPC_3P_integrator = integrator(deltat, stepno, LIPC_3P_integrator, LIPC_3P_PP1.y1, false);

				// if the pole is larger than the zero it is a phase lead; otherwise it is a phase lag
				LIPC_3P_lead.f_zero = 2.0482 / 2 / pi; // org1&2&4: 0.3 / 2 / pi; // frequency of the zero in Hz
				LIPC_3P_lead.f_pole = 2.7105 / 2 / pi; //0.9 / 2 / pi; // org1&2: 1.0 / 2 / pi; // frequency of the pole in Hz
				LIPC_3P_lead = leadlag_filter(deltat, stepno, LIPC_3P_lead, LIPC_3P_integrator.y1);
			}
			if (true) { //only 2P (for working in parallel)
				double K_p_2P = 5e-11 * GA_Parameter2; // current best with * 6; // GA_Parameter1; // org4: 1e-10 * 4; // org2: 3e-11*4; // org: 1.7e-10*4; // IgainIPC_1P;
				double BP_damp_2P = 0.02;

				LIPC_2P_PP.f1 = oneP_Hz * 2; // 0.125; // 1P Hz
				LIPC_2P_PP.zeta = BP_damp_2P; // 0.06; // org3: 0.15; // org2: 0.02; // org: 0.05;  // damping
				LIPC_2P_PP = bandpassfilter_boris(deltat, stepno, LIPC_2P_PP, M0 * K_p_2P);

				LIPC_2P_integrator = integrator(deltat, stepno, LIPC_2P_integrator, LIPC_2P_PP.y1, false);

				// if the pole is larger than the zero it is a phase lead; otherwise it is a phase lag
				LIPC_2P_lag.f_zero = 3.2206 / 2 / pi; // org1&2&4: 0.3 / 2 / pi; // frequency of the zero in Hz
				LIPC_2P_lag.f_pole = 0.76613 / 2 / pi; //0.9 / 2 / pi; // org1&2: 1.0 / 2 / pi; // frequency of the pole in Hz
				LIPC_2P_lag = leadlag_filter(deltat, stepno, LIPC_2P_lag, LIPC_2P_integrator.y1);
			}

			double theta_0 = LIPC_2P_lag.y1; // Nfilt_inv_1P.y1;
			double theta_1 = LIPC_3P_lead.y1 + LIPC_1P_lead.y1; // +LIPC_towerHz_lead.y1; // M1 * IgainIPC_1P;
			thetaref[0] = theta_col_ref + theta_0 - theta_1;
			thetaref[1] = theta_col_ref + theta_0 + theta_1;
		}


		// first working version  of the LIPC. Utilized in older results of the WESC 2021. Code version v27 (DTU_discon_v27_LIPC_new_FA_gains)
		//if (time > 0.01 && t_cutout == 0 && IgainIPC > 0) {
		double IgainIPC = 5e-10;
		bool use_first_LIPC_version_of_DISCON_v27 = false;
		if (time > -1 && t_cutout == 0 && IgainIPC > 0 && use_first_LIPC_version_of_DISCON_v27) {
			// LIPC Algorithm. Source: Edwin van Solingen ()
			M0 = IPCvar.M_y[0] * 1 / 2 + IPCvar.M_y[1] * 1 / 2;   // collective mode:   2P
			M1 = -IPCvar.M_y[0] * 1 / 2 + IPCvar.M_y[1] * 1 / 2; // differential mode: 1P and 3P harmonics

			HPfilt_1P.f1 = 3.65 * 10e-4 * 0.125 / 0.695; // Hz
			HPfilt_1P.f2 = 0.0376; // 0.083; // Hz (Frequency where the filter starts)
			HPfilt_1P.beta = -6.41; // value of zeta to achieve inverse filter
			HPfilt_1P.zeta = 0.7; // 0.48; // value of beta to achieve inverse filter
			HPfilt_1P = filt2orderSolingen(deltat, stepno, HPfilt_1P, M1 * IgainIPC);

			Nfilt_inv_1P.f1 = 0.125; // Hz
			Nfilt_inv_1P.f2 = 0.125; // Hz
			Nfilt_inv_1P.beta = 1.3; // value of zeta to achieve inverse filter
			Nfilt_inv_1P.zeta = 1.5 * 10e-3; // value of beta to achieve inverse filter
			Nfilt_inv_1P = filt2orderSolingen(deltat, stepno, Nfilt_inv_1P, M1 * IgainIPC); //, HPfilt_1P.y1);
			//Nfilt_inv_1P = filt2orderSolingen(deltat, stepno, Nfilt_inv_1P, HPfilt_1P.y1);

			Nfilt_3P.f1 = 0.125 * 3; // Hz
			Nfilt_3P.f2 = 0.125 * 3; // Hz
			Nfilt_3P.beta = 9 * 10e-3; // value of zeta to achieve inverse filter
			Nfilt_3P.zeta = 0.45; // value of beta to achieve inverse filter
			//Nfilt_3P = filt2orderSolingen(deltat, stepno, Nfilt_3P, M1 * IgainIPC); // Nfilt_inv_1P.y1);
			Nfilt_3P = filt2orderSolingen(deltat, stepno, Nfilt_3P, Nfilt_inv_1P.y1);

			Nfilt_inv_3P.f1 = 0.125 * 3; // Hz
			Nfilt_inv_3P.f2 = 0.125 * 3; // Hz
			Nfilt_inv_3P.beta = 0.1; // 45; // value of zeta to achieve inverse filter
			Nfilt_inv_3P.zeta = 9 * 10e-3; // value of beta to achieve inverse filter
			//Nfilt_inv_1P = filt2orderSolingen(deltat, stepno, Nfilt_inv_1P, M1 * IgainIPC); //, HPfilt_1P.y1);
			Nfilt_inv_3P = filt2orderSolingen(deltat, stepno, Nfilt_inv_3P, Nfilt_inv_1P.y1);

			LPfilt_1P.f1 = 2.77 * 0.125 / 0.695; // Hz old 1
			LPfilt_1P.f2 = 4.75 * 0.125 / 0.695; // Hz old 2
			LPfilt_1P.zeta = 0.7; // value of beta to achieve inverse filter, old 0.53
			//LPfilt_1P = LowPassFilt2orderSolingen(deltat, stepno, LPfilt_1P, M1 * IgainIPC); // Nfilt_inv_1P.y1);
			LPfilt_1P = LowPassFilt2orderSolingen(deltat, stepno, LPfilt_1P, 2.9402 * Nfilt_3P.y1);
			// amplifying by 2.9402 results into a filter gain of 0 db at lower frequencies

			Nfilt_inv_2P.f1 = 0.125 * 2; // Hz
			Nfilt_inv_2P.f2 = 0.125 * 2; // Hz
			Nfilt_inv_2P.beta = 0.77; // value of zeta to achieve inverse filter
			Nfilt_inv_2P.zeta = 1.02 * 10e-2; // value of beta to achieve inverse filter
			Nfilt_inv_2P = filt2orderSolingen(deltat, stepno, Nfilt_inv_2P, M0 * IgainIPC); //, HPfilt_1P.y1);
			//Nfilt_inv_1P = filt2orderSolingen(deltat, stepno, Nfilt_inv_1P, HPfilt_1P.y1);

			double theta_0 = Nfilt_inv_2P.y1; // Nfilt_inv_1P.y1;
			double theta_1 = LPfilt_1P.y1; // M1 * IgainIPC;
			thetaref[0] = theta_col_ref + theta_0 - theta_1;
			thetaref[1] = theta_col_ref + theta_0 + theta_1;
		}




		


		// add Yaw controller
		if (use2B_teetering == 3) {
			if (tellItOnce == 0) ReportInfoMessage(turbine_id, "yaw control of rotation speed activated");
			double yawDirection = 1; // GA_Parameter28; // should be 1 or -1;

			PID_pit_var.velmax = 10 * degrad; // rad/s max velo
			PID_pit_var.outmax = 90 * degrad; // max yaw angle in rad
			PID_pit_var.outmin = 0 * degrad; // min yaw angle in rad
			yawRef = yawDirection * (PID_pit_var.outres1 - PID_pit_var.outres1_old) / deltat;

			// to avoid any pitching:
			thetaref[0] = 0;
			thetaref[1] = 0;

			if (abs(PID_pit_var.outres1 - GetMeasuredYawBearingAngularPosition(turbine_id) * -yawDirection) > 0.1 * degrad) {
				PID_pit_var.outres = GetMeasuredYawBearingAngularPosition(turbine_id) * -yawDirection;
				PID_pit_var.outset1 = PID_pit_var.outres - PID_pit_var.outpro - PID_pit_var.outdif;
				PID_pit_var.outres1 = PID_pit_var.outres;
			}
			// if (abs(yawRef) > 10 * degrad) yawRef = copysign(10 * degrad, yawRef);

			/*init_yaw_angle_look_up.lines = 5;
			init_yaw_angle_look_up.wpdata[0][0] = 11;
			init_yaw_angle_look_up.wpdata[1][0] = 12;
			init_yaw_angle_look_up.wpdata[2][0] = 15;
			init_yaw_angle_look_up.wpdata[3][0] = 19;
			init_yaw_angle_look_up.wpdata[4][0] = 23;

			init_yaw_angle_look_up.wpdata[0][1] =  0 * degrad;
			init_yaw_angle_look_up.wpdata[1][1] = 25 * degrad;
			init_yaw_angle_look_up.wpdata[2][1] = 50 * degrad;
			init_yaw_angle_look_up.wpdata[3][1] = 62 * degrad;
			init_yaw_angle_look_up.wpdata[4][1] = 68 * degrad;

			double init_yaw_angle = GetInterpolatedValue(WSPfilt, init_yaw_angle_look_up);
			if (time < -1 && GetMeasuredYawBearingAngularPosition(turbine_id) * -yawDirection < init_yaw_angle) {
				yawRef = yawDirection * 8 * degrad;
			}*/

			if (time < 0.1) { yawRef = 0; } // blocking initial yaw action
		}


		// add teeter motion mimicking by sinusodal yaw oscillation for hixed hub two-bladed turbines.
		if (use2B_teetering == 5) {
			if (tellItOnce == 0) ReportInfoMessage(turbine_id, "Mimicking horizontal teeter motion by yaw oscillation.");
			double yawDirection = 1; // GA_Parameter28; // should be 1 or -1;
			double yawAmplitude = 0.8 * degrad;

			//double YawAngle = abs(sin(GetMeasuredRotorAzimuthAngle(turbine_id))) * yawDirection * yawAmplitude * 2;
			//double YawAngle = fmax(abs(sin(GetMeasuredRotorAzimuthAngle(turbine_id))), 0.2) * yawDirection * yawAmplitude * 1.5;
			//double YawAngle = (1 - cos(GetMeasuredRotorAzimuthAngle(turbine_id)*2 - 0.107*2)) * yawDirection * yawAmplitude/2;
			//double YawAngle = fmax(sqrt(abs(sin(GetMeasuredRotorAzimuthAngle(turbine_id)))), 0.2) * yawDirection * yawAmplitude;
			//double YawAngle = pow(sin(GetMeasuredRotorAzimuthAngle(turbine_id)),4) * yawDirection * yawAmplitude;
			//double YawAngle = (1 - pow(cos(GetMeasuredRotorAzimuthAngle(turbine_id)), 4)) * yawDirection * yawAmplitude;

			double YawAngle = yawDirection * yawAmplitude;

			double CounterPitching = cos(GetMeasuredRotorAzimuthAngle(turbine_id)) * yawDirection * yawAmplitude;
			thetaref[0] = thetaref[0] + CounterPitching;
			thetaref[1] = thetaref[1] - CounterPitching;


			yawRef = (temp_oldYawAngle - YawAngle) / deltat;
			temp_oldYawAngle = YawAngle;

			//if (time < 0.1) { yawRef = 0; } // blocking initial yaw action
		}





		// DLC1.5 (and others) pitch teeter angle and velocity coupling to reduce most extreme teeter excursion and thus closest tip to tower approach.
		double CouplingFreeTeeterAngle = 3 * degrad;   // in rad
		double CouplingFreeTeeterVelocity = 3 * degrad;   // in rad/s
		double TeeterCoupledPitchAngle = 0;
		if (use2B_teetering == 1 && t_cutout == 0) { // if (use2B_teetering && true) {
			if (abs(GetMeasuredTeeterAngle(turbine_id)) > CouplingFreeTeeterAngle) { // "spring": adding a teeter angle and pitch coupling for teeter angles larger 3°
				double PitchTeeterGain = 2;
				TeeterCoupledPitchAngle = copysign(PitchTeeterGain * (abs(GetMeasuredTeeterAngle(turbine_id)) - CouplingFreeTeeterAngle), GetMeasuredTeeterAngle(turbine_id));
			}
			if (abs(GetMeasuredTeeterVelocity(turbine_id)) > CouplingFreeTeeterVelocity) { // "damper": adding a teeter velocity and pitch coupling for teeter velocities larger 3°/s
				double PitchTeeterDampingGain = 1.5;
				TeeterCoupledPitchAngle += copysign(PitchTeeterDampingGain * (abs(GetMeasuredTeeterVelocity(turbine_id)) - CouplingFreeTeeterVelocity), GetMeasuredTeeterVelocity(turbine_id));
			}
			thetaref[0] += -TeeterCoupledPitchAngle;
			thetaref[1] += +TeeterCoupledPitchAngle;
			// keep care of teeter unbalances (unsymmetric pitch teeter coupling), if only one actuator overshooted the minimum pitch angle:
			// NOTE: might be better to prove whether thetaref 0 or 1 are below the min angle and then adapt the coupling value. Otherwise they are not perfectly mirrored, because the collective pitch angle is not static. 
			if (thetaref[0] < -3 * degrad || thetaref[1] < -3 * degrad) { // garantee minimum -3° of pitch 
				double RemoveIt = fmin(thetaref[0], thetaref[1]) + 3 * degrad;
				if (thetaref[0] < thetaref[1]) RemoveIt = RemoveIt * -1;
				thetaref[0] += RemoveIt;
				thetaref[1] -= RemoveIt;
			}
			/*if (thetaref[0] < -9 * degrad || thetaref[1] < -9 * degrad) { // garantee minimum -9° of pitch
				double RemoveIt = fmin(thetaref[0], thetaref[1]) + 9 * degrad;  // calculate which pitch is smaller than -9°
				if (thetaref[0] < thetaref[1]) RemoveIt = RemoveIt * -1;
				thetaref[0] += RemoveIt;
				thetaref[1] -= RemoveIt;
			}*/
		} /* if (use2B_teetering && true) {
			if (tellItOnce == 0) ReportInfoMessage(turbine_id, "Using teeter spring and damper coupling with optimizing params\n Param 11 => PTC spring ratio [deg] \n Param 12 => PTC spring free angle \n Param 13 => PTC damper ratio \n Param 14 => PTC damper free velo [deg/s] \n All Param 15-18 are the same as 11-14 but quadratic coupling ");
			double TeeterAngle = GetMeasuredTeeterAngle(turbine_id);
			double TeeterVelo = GetMeasuredTeeterVelocity(turbine_id);

			double PitchTeeterGain = GA_Parameter1; // default = 2
			CouplingFreeTeeterAngle = GA_Parameter2 * degrad; // default = 3°
			if (abs(TeeterAngle) > CouplingFreeTeeterAngle) { // "spring": adding a teeter angle and pitch coupling for teeter angles larger Param 2
				TeeterCoupledPitchAngle = copysign(PitchTeeterGain * (abs(TeeterAngle) - CouplingFreeTeeterAngle), TeeterAngle);
			}
			double PitchTeeterDampingGain = GA_Parameter3; // default = 1.5
			CouplingFreeTeeterVelocity = GA_Parameter4; // default = 3°/s
			if (abs(TeeterVelo) > CouplingFreeTeeterVelocity) { // "damper": adding a teeter velocity and pitch coupling for teeter velocities larger 3°/s
				TeeterCoupledPitchAngle += copysign(PitchTeeterDampingGain * (abs(TeeterVelo) - CouplingFreeTeeterVelocity), TeeterVelo);
			}
			double QuadraticPitchTeeterGain = GA_Parameter5; // default = none
			double QuadraticCouplingFreeTeeterAngle = GA_Parameter6 * degrad; // default = none
			if (abs(TeeterAngle) > QuadraticCouplingFreeTeeterAngle) { // "quadratic spring": adding a quadratic pitch coupling for teeter angles larger Param 6
				TeeterCoupledPitchAngle += copysign(QuadraticPitchTeeterGain * pow(abs(TeeterAngle) - QuadraticCouplingFreeTeeterAngle, 2), TeeterAngle);
			}
			double QuadraticPitchTeeterDampingGain = GA_Parameter7; // default = 1.5
			double QuadraticCouplingFreeTeeterVelocity = GA_Parameter8; // default = 3°/s
			if (abs(TeeterVelo) > QuadraticCouplingFreeTeeterVelocity) { // "damper": adding a teeter velocity and pitch coupling for teeter velocities larger 3°/s
				TeeterCoupledPitchAngle += copysign(QuadraticPitchTeeterDampingGain * pow(abs(TeeterVelo) - QuadraticCouplingFreeTeeterVelocity, 2), TeeterVelo);
			}
			thetaref[0] += -TeeterCoupledPitchAngle;
			thetaref[1] += +TeeterCoupledPitchAngle;
		}*/


		//********************* Ensure maximum pitch rate and max pitch intervall *********************** //
		double MaxPitchAngle = 90 * degrad, MinPitchAngle = -9 * degrad;

		for (int i = 0; i < nmbrBlades; i++) {
			thetaref[i] = fmax(MinPitchAngle, fmin(MaxPitchAngle, thetaref[i])); //ensures min/max angle
			if (time > 10) {// ensures max pitch velo for time step length "deltat" after first 10 seconds
				if (abs(thetaref[i] - thetaref_last[i]) > MaxPitchVelocity * deltat) {
					thetaref[i] = thetaref_last[i] + copysign(MaxPitchVelocity * deltat, thetaref[i] - thetaref_last[i]);
					if (tellItOnce < 2) {
						tellItOnce = 2; // note, does not always work as planned. Maybe implement a delta difference between limited and unlimited demanded pitch.
						sprintf_s(reportMessage, 55, "WARNING: Limiting the pitch rate at %.2fs !!!", time); ReportWarningMessage(turbine_id, reportMessage);
					}
				}
				else if (tellItOnce < 3) tellItOnce = 1;
			}
			thetaref_last[i] = thetaref[i]; // store for the next step
		}

		//************************************************************************************************//
		//												Output											  //
		//************************************************************************************************//

		SetDemandedGeneratorTorque(turbine_id, Qgen_ref / GearBoxRatio);	// 1 : Generator torque reference[Nm]   // including gear box!!!!

		SetDemandedPitchAngle(turbine_id, 0, thetaref[0]);		// 2 : Pitch angle reference of blade 1[rad]
		SetDemandedPitchAngle(turbine_id, 1, thetaref[1]);		// 3 : Pitch angle reference of blade 2[rad]
		if (nmbrBlades == 3) SetDemandedPitchAngle(turbine_id, 2, thetaref[2]);		// 4 : Pitch angle reference of blade 3[rad]

		if (extremeYawMisalignment) {
			// old legacy code v31 -> // if (abs(GetMeasuredYawError(turbine_id)) > 4 * degrad) yawRef = (-1 * degrad); // copysign(GetMeasuredYawError(turbine_id), yawRate *degrad* deltat);
			// if (abs(meanYawMisalignment) > 0.3 * 10 * degrad) yawRef = 0.7 * degrad * copysign(1, GetMeasuredYawError(turbine_id)); // yawRef = (-1 * degrad); // set constant ja rate rate of 0.3 deg/s (or use yaw_stopvelmax?)
			// if (abs(meanYawMisalignment) < 88 * degrad) yawRef = - 0.7 * degrad * copysign(1, GetMeasuredYawError(turbine_id)); // alternative: yaw to 90 deg for shut down
			if (abs(meanYawMisalignment) < (90 - 0.7 * time_period_for_yaw_mean_in_s) * degrad) yawRef = -0.7 * degrad * copysign(1, GetMeasuredYawError(turbine_id)); // alternative: yaw to 90 deg for shut down
			else {
				extremeYawMisalignment = 0;
				yawRef = 0;
			}

		} // else SetDemandedYawRate(turbine_id, 0); // BE CAREFUL WITH OVERWRITING DEMANDED VALUES SO EASY

		SetDemandedYawRate(turbine_id, yawRef);

		if (GridlossDetected == 1) { // IS IT POSSIBLE TO APPLY TORQUE WITH SLOPES???
			//SetDrivetrainBrakeStatus(turbine_id, 1, 1); //big effect
			//SetDrivetrainBrakeStatus(turbine_id, 1, 5);
			if (time > t_cutout) {
				//SetDemandedAdditionalBrakeTorque(turbine_id, -10 * deltat);
				SetShaftBrakeStatusBinaryFlag(turbine_id, 0);
			}
			else SetShaftBrakeStatusBinaryFlag(turbine_id, 31);
			//else SetDemandedAdditionalBrakeTorque(turbine_id, 10 * deltat);
		}

		if (tellItOnce == 0) tellItOnce = 1;

		SetLoggingValue(turbine_id, 0, Pe_ref);					// 5 : Power reference[W]
		SetLoggingValue(turbine_id, 1, WSPfilt);				// 6 : Filtered wind speed[m / s]
		SetLoggingValue(turbine_id, 2, omegafilt);				// 7 : Filtered rotor speed[rad / s]
		SetLoggingValue(turbine_id, 3, omega_err_filt_speed);	// 8 : Filtered rotor speed error for torque[rad / s]
		SetLoggingValue(turbine_id, 4, Qgen_ref); // omega_dtfilt);			// 9 : Bandpass filtered rotor speed[rad / s]
		SetLoggingValue(turbine_id, 5, PID_gen_var.outpro / GearBoxRatio);		// 10 : Proportional term of torque contr.[Nm]
		SetLoggingValue(turbine_id, 6, PID_gen_var.outset / GearBoxRatio);		// 11 : Integral term of torque controller[Nm]
		SetLoggingValue(turbine_id, 7, PID_gen_var.outmin / GearBoxRatio);		// 12 : Minimum limit of torque[Nm]
		SetLoggingValue(turbine_id, 8, PID_gen_var.outmax / GearBoxRatio);		// 13 : Maximum limit of torque[Nm]
		SetLoggingValue(turbine_id, 9, switch1);				// 14 : Torque limit switch based on pitch[-]
		SetLoggingValue(turbine_id, 10, omega_err_filt_pitch);	// 15 : Filtered rotor speed error for pitch[rad / s]
		SetLoggingValue(turbine_id, 11, e_pitch[1]);			// 16 : Power error for pitch[W]
		SetLoggingValue(turbine_id, 12, PID_pit_var.outpro);	// 17 : Proportional term of pitch controller[rad]
		SetLoggingValue(turbine_id, 13, PID_pit_var.outset);	// 18 : Integral term of pitch controller[rad]
		SetLoggingValue(turbine_id, 14, PID_pit_var.outmin);	// 19 : Minimum limit of pitch[rad]
		SetLoggingValue(turbine_id, 15, PID_pit_var.outmax);	// 20 : Maximum limit of pitch[rad]

		SetLoggingValue(turbine_id, 16, pitchADC * raddeg);		// 16 : pitch actuator duty cycle
		SetLoggingValue(turbine_id, 17, t_cutout);				// 17 : time to cut out
		SetLoggingValue(turbine_id, 18, kgain_pitch[0][0] * PID_pit_var.Kpro[0]);	// 18 : pitch gain in controller (after gain scheduling)
		SetLoggingValue(turbine_id, 19, kgain_pitch[1][0] * PID_pit_var.Kint[0]);	// 19 : pitch gain in controller
		SetLoggingValue(turbine_id, 20, kgain_pitch[0][1] * PID_pit_var.Kpro[1]);	// 20 : pitch gain in controller
		SetLoggingValue(turbine_id, 21, kgain_pitch[1][1] * PID_pit_var.Kint[1]);	// 21 : pitch gain in controller
		SetLoggingValue(turbine_id, 22, kgain_pitch[0][0]);// 22 : pitch gain scheduling in controller
		SetLoggingValue(turbine_id, 23, kgain_pitch[0][1]);// 23 : pitch gain scheduling in controller

		SetLoggingValue(turbine_id, 24, meanpitangfilt);				// 21 :
		SetLoggingValue(turbine_id, 25, switch1_pitang_upper);				// 21 :
		SetLoggingValue(turbine_id, 26, switch1_pitang_lower);				// 21 :
		SetLoggingValue(turbine_id, 27, Qg_min_partial);				// 21 :
		SetLoggingValue(turbine_id, 28, Qg_max_partial);				// 21 :
		SetLoggingValue(turbine_id, 29, nacelle_roll_velo - Nacelle_Roll_Accel_filt.y1);				// 21 :
		SetLoggingValue(turbine_id, 30, nacelle_roll_velo);				// 21 :
		SetLoggingValue(turbine_id, 31, Nacelle_Roll_Velo_bandfilt.y1);				// 21 :
		SetLoggingValue(turbine_id, 32, Nacelle_Roll_Velo_notch_filt2.y1);				// 21 :
		SetLoggingValue(turbine_id, 33, Nacelle_Roll_Velo_notch_filt3.y1);				// 21 :
		SetLoggingValue(turbine_id, 34, Nacelle_Roll_Velo_notch_filt4.y1);				// 21 :
		SetLoggingValue(turbine_id, 35, Nacelle_Roll_Accel_filt.y1);				// 21 :
		SetLoggingValue(turbine_id, 36, yawErrorFilt);
		SetLoggingValue(turbine_id, 37, GetMeasuredPitchAngle(turbine_id, 0) - GetMeasuredPitchAngle(turbine_id, 1));
		SetLoggingValue(turbine_id, 38, nonlinear_pitch_gain_component);
		SetLoggingValue(turbine_id, 39, aero_gain);
		SetLoggingValue(turbine_id, 40, time_exit_exzone);
		SetLoggingValue(turbine_id, 41, omega_tower_eigenfrequency_interaction);
		SetLoggingValue(turbine_id, 42, check_exZoneState);
		SetLoggingValue(turbine_id, 43, ex_zone_sign);
		SetLoggingValue(turbine_id, 44, Pitch_FATowDamper_gain);
		SetLoggingValue(turbine_id, 45, GenT_latTowDamper_gain);
		SetLoggingValue(turbine_id, 46, GetMeasuredYawError(turbine_id));
		SetLoggingValue(turbine_id, 47, exZone_torque_memory);

		SetLoggingValue(turbine_id, 48, IPCvar.theta[1][0]);
		SetLoggingValue(turbine_id, 49, IPCvar.theta[1][1]);
		SetLoggingValue(turbine_id, 50, theta_tilt); // IPCvar.theta_tilt[1]
		SetLoggingValue(turbine_id, 51, theta_yaw);  // IPCvar.theta_yaw[1]
		SetLoggingValue(turbine_id, 52, IPC_LP_Blade1_filt.y1); //M0
		SetLoggingValue(turbine_id, 53, M1);
		SetLoggingValue(turbine_id, 54, M_tilt); // IPCvar.M_tilt[1]
		SetLoggingValue(turbine_id, 55, M_yaw);  // IPCvar.M_yaw[1]

		SetLoggingValue(turbine_id, 56, HPfilt_1P_3P.y1);
		SetLoggingValue(turbine_id, 57, Nfilt_inv_1P.y1);
		SetLoggingValue(turbine_id, 58, Nfilt_3P.y1);
		SetLoggingValue(turbine_id, 59, LPfilt_1P.y1);
		SetLoggingValue(turbine_id, 60, Nfilt_inv_2P.y1);
		SetLoggingValue(turbine_id, 61, BP_blade1_Rossiter.y1);
		SetLoggingValue(turbine_id, 62, BP_blade2_Rossiter.y1);
		SetLoggingValue(turbine_id, 63, Blade1_My);
		SetLoggingValue(turbine_id, 64, Blade2_My);






		SetLoggingValue(turbine_id, 65, TeeterCoupledPitchAngle);
		SetLoggingValue(turbine_id, 66, temp_Blade1XVelo);


		return GH_DISCON_SUCCESS;
	}
}






/* #####################################################################################
CHANGE LOG:
14.02.19: Added drive train losses to aim for higher rated torque
until 22.03.19: Added 20MW option; preliminary version of speed exclusion zone
23.03.19: Using start up logic
####################################################################################### */