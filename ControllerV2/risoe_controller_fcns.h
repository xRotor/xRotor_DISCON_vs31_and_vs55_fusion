#ifndef RISOE_CONTROLLER_FCNS_H
#define RISOE_CONTROLLER_FCNS_H

//#include "cmath" // TRY A WORKAROUND ONCE EVERYTHING IS RUNNING AS PLANNED !!!!

#include "GH_DISCON_Constants.h"
#include "string"
#include "algorithm"


#ifdef __cplusplus    // These statements are only necessary (and indeed valid) when compiled under C++
namespace GHTurbineInterface {

	extern "C"
	{
#endif

		// int SetDemandedLidarBeamFocalPointFocalDistance(const turbine turbine_id, int index_lidar_beam, int index_lidar_beam_focal_point, double focal_distance);






		//************************************************************************************************//
		//************************************************************************************************//
		//************************************************************************************************//
		//									risoe controller fcns.f90									  //			
		//************************************************************************************************//
		//************************************************************************************************//
		//************************************************************************************************//


		//--class risoe_controller_fcns  //- module risoe_controller_fcns
		//--{
		//--public: //neccessary???
		//Constants
		const double pi = 3.14159265358979;
		const double degrad = 0.0174532925;
		const double raddeg = 57.2957795131;
		static const int maxwplines = 100;
		//Structures (Types in f90)
		struct Tfirstordervar { int stepno1 = 0; double tau, x1, x1_old, y1, y1_old; };    // should stepno1 be defined as 1 or 0 and/or is 0 the default value ???
		struct Tlowpass2order { int stepno1 = 0; double zeta, f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old; };
		struct Tnotch2order { int stepno1 = 0; double zeta1 = 0.1, zeta2 = 0.001, f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old; };
		struct Tbandpassfilt { int stepno1 = 0; double zeta = 0.02, tau = 0.0, f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old; };
		struct Tpidvar {
			int stepno1 = 0; double Kpro = 0, Kdif = 0, Kint = 0, outmin = 0, outmax = 0, velmax = 0, error1 = 0, outset1 = 0, outres1 = 0,
				outset = 0, outpro = 0, outdif = 0, error1_old = 0, outset1_old = 0, outres1_old = 0, outres = 0;
		};
		struct Tpid2var {
			int stepno1 = 0; double outmin, outmax, velmax, outset1, outres1, outset, outpro, outdif, outset1_old, outres1_old, outres;
			double Kpro[2]; double Kdif[2]; double Kint[2]; double error1[2]; double error1_old[2];
		};
		struct Twpdata { int lines;   double wpdata[maxwplines][2]; };
		struct IPCstruct { double M_tilt[3], M_yaw[3], M_y[3], theta_tilt[3], theta_yaw[3], theta[3][2], azimuth, azimuthOffset; }; //theta_sum_IPC[2], 
		struct Tfilt2order { int stepno1 = 0; double beta = 0.1, zeta = 0.001, f1, f2, x1, x2, x3, y1, y2, y3; };


		struct Tleadfilt { int stepno1 = 0; double f_zero = 0.3, f_pole = 1, x1, x2, y1, y2; }; // for LIPC
		// Variables
		int    stepno;
		bool   const_power;
		double deltat, time_old;
		double omega_ref_max, omega_ref_min, Pe_rated, Qg_rated, pitch_stopang, max_lss_torque;
		double Kopt, rel_sp_open_Qg;
		double kk1, kk2, rel_limit;
		double switch1_pitang_lower, switch1_pitang_upper, switch1;
		double DT_damp_gain;
		bool   generator_cutin = false;
		double t_cutin, t_generator_cutin = 0, t_cutin_delay;
		int    pitch_stoptype;
		double t_cutout, pitch_stopdelay, pitch_stopdelay2, pitch_stopvelmax, pitch_stopvelmax2;
		Tlowpass2order omega2ordervar;
		Tnotch2order   DT_mode_filt;
		Tnotch2order   pwr_DT_mode_filt;
		//-------------------------------//
		Tnotch2order   Ex_mode_filt;
		Tnotch2order   Ex_mode_filt2;
		Tnotch2order   Ex_mode_filt3;
		Tnotch2order   Tower_FA_Accel_filt;
		Tnotch2order   Tower_SS_Accel_filt;
		Tnotch2order   Nacelle_Roll_Accel_filt;
		Tbandpassfilt  Nacelle_Roll_Velo_bandfilt;
		Tlowpass2order Nacelle_Roll_Velo_LP_filt;
		Tnotch2order Nacelle_Roll_Velo_notch_filt2;
		Tnotch2order Nacelle_Roll_Velo_notch_filt3;
		Tnotch2order Nacelle_Roll_Velo_notch_filt4;
		Tnotch2order   pitch_out_signal_filt;
		Tnotch2order   pitch_power_error_filt;
		//-------------------------------//
		Tbandpassfilt  DT_damper_filt;
		Tpid2var       PID_pit_var;
		Tpidvar        PID_gen_var;
		Tpidvar        PID_IPC_tilt_var[3], PID_IPC_yaw_var[3];
		Twpdata        OPdatavar;
		Twpdata		   FA_damper_gains_look_up, StS_damper_gains_look_up;
		Tfirstordervar wspfirstordervar;
		Tfirstordervar pitchfirstordervar;
		Tfirstordervar yawfirstordervar;
		Tfirstordervar torquefirstordervar;
		Tfirstordervar switchfirstordervar;
		Tfirstordervar cutinfirstordervar;
		IPCstruct	   IPCvar; // , IPCvar_n2, IPCvar_n4;
		Tnotch2order   IPC_notchfilt[3];
		Tfilt2order	   HPfilt_1P, Nfilt_inv_1P, Nfilt_3P, LPfilt_1P;    // only necessary for LIPC 1P
		Tfilt2order	   HPfilt_1P_3P, Nfilt_inv_1P__1P_3P, Nfilt_inv_3P__1P_3P, LPfilt_1P_3P; // only necessary for LIPC 1P 3P
		Tfilt2order    HPfilt_2P, Nfilt_inv_2P, LPfilt_2P;    // only necessary for LIPC 2P
		Tfilt2order    Nfilt_inv_3P; // temp! delete after time
		Tfilt2order    BP_blade1_Rossiter, BP_blade2_Rossiter; // only for Rossiter IPC
		Tfilt2order    BP_blade1_Rossiter_2P, BP_blade2_Rossiter_2P; // only for Rossiter IPC
		Twpdata		   init_yaw_angle_look_up;
		Twpdata		   LIPC_predef_gains;
		Twpdata		   LIPC_predef_gains_1P, LIPC_predef_gains_2P, LIPC_predef_gains_3P;

		Tlowpass2order IPC_LP_filt;
		Tlowpass2order IPC_LP_Blade1_filt, IPC_LP_Blade2_filt;
		Tlowpass2order IPC_LP_Blade1_filt_2P, IPC_LP_Blade2_filt_2P;

		Tleadfilt		LIPC_1P_lead, LIPC_2P_lag, LIPC_2P_integrator, LIPC_3P_lead, LIPC_3P_integrator, LIPC_towerHz_lead, CPC_FA_damp_lead, CPC_FA_damp_lead_2P, CPC_FA_damp_integrator_2P, GenT_StS_damp_integrator; // Boris tuned LIPC
		Tfilt2order     LIPC_1P_BP, LIPC_2P_PP, LIPC_3P_PP2, LIPC_towerHz_PP, LIPC_3P_PP1, LIPC_1P_BP2, LIPC_1P_BP3, CPC_FA_damp_PP, CPC_FA_damp_PP2, CPC_FA_damp_PP_2P, GenT_StS_damp_PP, GenT_StS_damp_PP2, GenT_StS_damp_PP3; // Boris tuned LIPC

		Tfilt2order		omega_2P_notch_filt, omega_1P_notch_filt, DT_freeFree_notch_filt, FOWT_nacelle_FA_velo_DC_blocker, FOWT_surge_velo_DC_blocker;
		Tlowpass2order	FOWT_Nacelle_FA_velo_2nd_order_Butterworth_LP_filt, FOWT_surge_velo_2nd_order_Butterworth_LP_filt;
		//**************************************************************************************************//
		//										"cmath" functions											//
		//**************************************************************************************************//
		/*double powVal(double a, int b) {
			double x = a; // local var
			for (int i = 1; i < b; ++i) double x = x * a;
			return x;
		}
		double absVal(double a) {
			if (a < 0) a = -1 * a;
			return a;
		}
		double fmaxVal(double a, double b){
			double x = a;
			if(b > a) double x = b;
			return x;
		}
		double fminVal(double a, double b){
			double x = a;
			if(b < a) double x = b;
			return x;
		}
		double copysignVal(double a, double b){
			double x = absVal(a);
			if(b < 0) x = -1*x;
			return x;
		}
		double sqrtVal(double a) ????
		*/


		//**************************************************************************************************
		// The statement function in FORTRAN provided a means for a program unit to use a single line of FORTRAN as an internal function //- contains
		//**************************************************************************************************
		// 
		//***************************************** NEW FUNCTIONS BY ANFAB *********************************
		auto readInUserParameters(std::string parameter_name, std::string parameter_search_string, double default_value, const turbine turbine_id) {
			double parameter_value;
			Twpdata look_up_table{};
			int IndxParamBegin = 0;
			int IndxParamEnd = 0;
			std::string param_sub_string = "";
			char* reportMessage = new char[150];// [3];

			IndxParamBegin = parameter_search_string.find(parameter_name) + parameter_name.length() + 1; // start index for value of search word (string start + length of string + 1 (for the "=") )
			IndxParamEnd = parameter_search_string.find(';', IndxParamBegin);								// end  index for value of search word ( the ";" after the found string)
			if (parameter_search_string.find(parameter_name) == std::string::npos) {
				sprintf_s(reportMessage, 150, "Could not find %s! Will use default value %f instead. Please add additional user parameters in external control window!\n", parameter_name.c_str(), default_value); 	ReportWarningMessage(turbine_id, reportMessage);
				parameter_value = default_value;
			}
			else { //new2024

				param_sub_string = parameter_search_string.substr(IndxParamBegin, IndxParamEnd - IndxParamBegin);
				// remove empty space, just in case there are any.
				param_sub_string.erase(std::remove_if(param_sub_string.begin(), param_sub_string.end(), ::isspace), param_sub_string.end());


				sprintf_s(reportMessage, 150, "found %s with value(s): %s", parameter_name.c_str(), param_sub_string.c_str()); ReportInfoMessage(turbine_id, reportMessage);

				if (param_sub_string.find(":") == std::string::npos) { // True if no ':' can be found in string
					parameter_value = std::stod(param_sub_string); // convert str to double
				}
				else {
					// remove brackets, just in case there are any.
					param_sub_string.erase(remove(param_sub_string.begin(), param_sub_string.end(), '\['), param_sub_string.end());
					param_sub_string.erase(remove(param_sub_string.begin(), param_sub_string.end(), '\]'), param_sub_string.end());
					param_sub_string.erase(remove(param_sub_string.begin(), param_sub_string.end(), '\('), param_sub_string.end());
					param_sub_string.erase(remove(param_sub_string.begin(), param_sub_string.end(), '\)'), param_sub_string.end());
					//sprintf_s(reportMessage, 150, "shortened to: %s",param_sub_string.c_str()); ReportInfoMessage(turbine_id, reportMessage);

					// find number of parameters in list
					int param_list_length = 0; // new2024
					for (int i = 0; i < param_sub_string.size(); i++)
						if (param_sub_string[i] == ':')
							param_list_length++;


					sprintf_s(reportMessage, 50, "Found list of parameters for %i wind speeds.", param_list_length);	ReportInfoMessage(turbine_id, reportMessage);
					ReportInfoMessage(turbine_id, "Overwriting predefined flexible default gains.");

					if (param_list_length == 0)
						parameter_value = std::stod(param_sub_string); // convert str to double
					else {
						parameter_value = 1; // to trigger, that the list parameters are used
						look_up_table.lines = param_list_length;
						int list_param_start = 0;
						int ii = 0;
						int iii = 0;
						for (int i = 0; i < param_sub_string.size(); i++) {
							if (param_sub_string[i] == ':') {
								look_up_table.wpdata[ii][0] = std::stod(param_sub_string.substr(list_param_start, i - list_param_start));
								list_param_start = i + 1;
								ii++;
							}
							if (param_sub_string[i] == ',') {
								look_up_table.wpdata[iii][1] = std::stod(param_sub_string.substr(list_param_start, i - list_param_start));
								list_param_start = i + 1;
								iii++;
							}
						}
						// last round does not finish with a seperator, thus there the last number is added manually
						look_up_table.wpdata[iii][1] = std::stod(param_sub_string.substr(list_param_start, param_sub_string.length() - list_param_start));
					}


					param_list_length = 0;
				}
			}
			struct result { double parameter_value; Twpdata look_up_table; };
			return result{parameter_value, look_up_table };
		}
		//***************************************** NEW FUNCTIONS BY ANFAB *********************************
		//***************************************** NEW FUNCTIONS BY ANFAB *********************************
		IPCstruct MBC_transform(IPCstruct IPCvar, int i) {
			int n = i + 1;
			IPCvar.M_tilt[i] = cos(n * IPCvar.azimuth) * IPCvar.M_y[0] + cos(n * (IPCvar.azimuth + pi)) * IPCvar.M_y[1];
			IPCvar.M_yaw[i] = sin(n * IPCvar.azimuth) * IPCvar.M_y[0] + sin(n * (IPCvar.azimuth + pi)) * IPCvar.M_y[1];
			return IPCvar;
		}
		//**************************************************************************************************
		IPCstruct reverse_MBC_transform(IPCstruct IPCvar, int i) {
			int n = i + 1;
			IPCvar.theta[i][0] = cos(n * (IPCvar.azimuth + IPCvar.azimuthOffset)) * IPCvar.theta_tilt[i] + sin(n * (IPCvar.azimuth + IPCvar.azimuthOffset)) * IPCvar.theta_yaw[i];
			IPCvar.theta[i][1] = cos(n * (IPCvar.azimuth + IPCvar.azimuthOffset + pi)) * IPCvar.theta_tilt[i] + sin(n * (IPCvar.azimuth + IPCvar.azimuthOffset + pi)) * IPCvar.theta_yaw[i];
			//IPCvar.theta[i][0] = cos(n*(IPCvar.azimuth + IPCvar.azimuthOffset))      * IPCvar.M_tilt[i] + sin(n*(IPCvar.azimuth + IPCvar.azimuthOffset))      * IPCvar.M_yaw[i];
			//IPCvar.theta[i][1] = cos(n*(IPCvar.azimuth + IPCvar.azimuthOffset + pi)) * IPCvar.M_tilt[i] + sin(n*(IPCvar.azimuth + IPCvar.azimuthOffset + pi)) * IPCvar.M_yaw[i];
			return IPCvar;
		}
		//**************************************************************************************************
		Tfilt2order DC_blocker(int stepno, Tfilt2order filt, double x) {
			// The dc blocker is a small recursive filter specified by the difference equation
			// where R is a parameter that is typically somewhere between 0.9 and 1
			// it is also known as a first order IIR filter
			double R;
			// Step
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.y1 = x;
				filt.y2 = x;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y2 = filt.y1;
				}
				R = filt.f2;
				filt.y1 = R * filt.y2 + filt.x1 - filt.x2;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}
		//**************************************************************************************************
		Tpidvar Icontroller(double dt, double KgainI, Tpidvar PIDvar, double error)
		{ //- implicit none //- integer * 4 stepno //- real * 8 PID, dt, kgain(3), error //- type(Tpidvar) PIDvar
		  // Local vars
			double eps = 1.0E-6;

			// Update the integral term
			//PIDvar.outset = PIDvar.outset1 + 0.5*(error + PIDvar.error1)*KgainI * PIDvar.Kint*dt;
			PIDvar.outset = PIDvar.outset1 + PIDvar.error1 * KgainI * PIDvar.Kint * dt;
			//PIDvar.outset = PIDvar.outset1_old + 0.5*(error + PIDvar.error1)*KgainI * PIDvar.Kint*dt;

			if (PIDvar.outset < PIDvar.outmin)
				PIDvar.outset = PIDvar.outmin;
			else if (PIDvar.outset > PIDvar.outmax)
				PIDvar.outset = PIDvar.outmax;
			// Satisfy max velocity
			if (PIDvar.velmax > eps) {
				if ((abs(PIDvar.outset - PIDvar.outset1) / dt) > PIDvar.velmax)
					PIDvar.outset = PIDvar.outset1 + copysign(PIDvar.velmax * dt, PIDvar.outset - PIDvar.outset1);
			}
			// Anti - windup on integral term and save results
			PIDvar.outset1 = PIDvar.outset;
			PIDvar.error1 = error;
			//PIDvar.stepno1 = stepno;
			//if (stepno == 0) PIDvar.outset = 0;
			return PIDvar; //PID_val
		}

		Tfilt2order filt2orderSolingen(double dt, int stepno, Tfilt2order filt, double x)
		{ // The filter is coded from a notch filter from Edwin van Solingen discretized with Tustin-Method
			// G(s) = (s^2 + 2 beta (2*pi*f1) s + (2*pi*f1)^2) / (s^2 + 2 zeta (2*pi*f2) s + (2*pi*f2)^2) 
		  // However, other filters could be realized as well: for notchfilter f1 = f2 = f0; for inversed notchfilter f1 = f2 = f0 and switch beta and zeta; 
		  // for second order high pass: use f1 and f2 respectively as nominator and denominator frequency or simply f1 = 0

		  // local vars
			double f1, f2, beta, zeta, a1, a2, b0, b1, b2, denom;
			// Step
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.x3 = x;
				filt.y1 = x;
				filt.y2 = x;
				filt.y3 = x;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x3 = filt.x2;
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y3 = filt.y2;
					filt.y2 = filt.y1;
				}
				f1 = filt.f1;
				f2 = filt.f2;
				beta = filt.beta;
				zeta = filt.zeta;
				denom = 4 + 4 * 2 * pi * f2 * zeta * dt + dt * dt * 4 * pi * pi * f2 * f2;

				a1 = (-8 + 2 * dt * dt * 4 * pi * pi * f2 * f2) / denom;
				a2 = (4 - 4 * 2 * pi * f2 * zeta * dt + dt * dt * 4 * pi * pi * f2 * f2) / denom;
				b0 = (4 + 4 * 2 * pi * f1 * beta * dt + dt * dt * 4 * pi * pi * f1 * f1) / denom;
				b1 = (-8 + 2 * dt * dt * 4 * pi * pi * f1 * f1) / denom;
				b2 = (4 - 4 * 2 * pi * f1 * beta * dt + dt * dt * 4 * pi * pi * f1 * f1) / denom;

				filt.y1 = -a1 * filt.y2 - a2 * filt.y3 + b0 * filt.x1 + b1 * filt.x2 + b2 * filt.x3;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}

		Tfilt2order LowPassFilt2orderSolingen(double dt, int stepno, Tfilt2order filt, double x)
		{ // The filter is coded from a notch filter from Edwin van Solingen discretized with Tustin-Method
		  // Its only a lowpass filter

		  // local vars
			double f1, f2, zeta, a1, a2, b0, b1, b2, denom;
			// Step
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.x3 = x;
				filt.y1 = x;
				filt.y2 = x;
				filt.y3 = x;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x3 = filt.x2;
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y3 = filt.y2;
					filt.y2 = filt.y1;
				}
				f1 = filt.f1;
				f2 = filt.f2;
				zeta = filt.zeta;
				denom = 4 + 4 * 2 * pi * f2 * zeta * dt + dt * dt * 4 * pi * pi * f2 * f2;

				a1 = (-8 + 2 * dt * dt * 4 * pi * pi * f2 * f2) / denom;
				a2 = (4 - 4 * 2 * pi * f2 * zeta * dt + dt * dt * 4 * pi * pi * f2 * f2) / denom;
				b0 = (dt * dt * 4 * pi * pi * f1 * f1) / denom;
				b1 = 2 * b0;
				b2 = b0;

				filt.y1 = -a1 * filt.y2 - a2 * filt.y3 + b0 * filt.x1 + b1 * filt.x2 + b2 * filt.x3;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}


		Tfilt2order BandPassFilt_Rossiter(double dt, int stepno, Tfilt2order filt, double x)
		{ // The filter is coded from a bandpass filter Source: "Fundamental performance similarities between IPC strategies for wind turbines" by Wai Hou Lio, Bryn Ll. Jones, Qian Lu & J.A. Rossiter 
		  // G_bp(s) = 2 pi f_h s / (s^2 + 2 pi (f_h+f_l) s + 4 pi^2 f_h f_l) discretized with Tustin-Method

			double om_h, om_l, a1, a2, b0, b2, denom; // local vars
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.x3 = x;
				filt.y1 = 0;
				filt.y2 = 0;
				filt.y3 = 0;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x3 = filt.x2;
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y3 = filt.y2;
					filt.y2 = filt.y1;
				}
				om_h = 2 * pi * filt.f1; // omega_high: this is used as highest frequency before filtering higher ones
				om_l = 2 * pi * filt.f2; // omega_low:  this is used as lowest  frequency before filtering lower  ones

				denom = 4 + 2 * (om_h + om_l) * dt + om_h * om_l * dt * dt;
				a1 = (2 * om_h * om_l * dt * dt - 8) / denom;
				a2 = (4 - 2 * (om_h + om_l) * dt + om_h * om_l * dt * dt) / denom;
				b0 = (2 * om_h * dt) / denom;
				b2 = b0;

				filt.y1 = -a1 * filt.y2 - a2 * filt.y3 + b0 * filt.x1 - b2 * filt.x3;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}


		Tleadfilt leadlag_filter(double dt, int stepno, Tleadfilt filt, double x)
		{ // The filter is coded from Matlabs lead transfer function
		  // G_lead(s) = (1 + s/om_zero) / (1 + s/om_pole) discretized with Tustin-Method
		  // can be used as real pole or real zero with setting the respective frequencies to 0
			double om_zero, om_pole, a1, b0, b1, denom; // local vars
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.y1 = 0;
				filt.y2 = 0;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y2 = filt.y1;
				}
				om_zero = 2 * pi * filt.f_zero; // omega of the zero in rad/s
				om_pole = 2 * pi * filt.f_pole; // omega of the pole in rad/s

				/* from legacy v31 controller code:
				denom = dt + 2 / om_pole;
				a1 = (dt - 2 / om_pole) / denom;
				b0 = (dt + 2 / om_zero) / denom;
				b1 = (dt - 2 / om_zero) / denom;*/

				
				if (om_pole > 0) {
					denom = dt + 2 / om_pole;
					a1 = (dt - 2 / om_pole) / denom;
				} else {
					denom = dt;
					a1 = dt / denom;
				}
				if (om_zero > 0) {
					b0 = (dt + 2 / om_zero) / denom;
					b1 = (dt - 2 / om_zero) / denom;
				} else {
					b0 = dt / denom;
					b1 = dt / denom;
				}

				filt.y1 = -a1 * filt.y2 + b0 * filt.x1 + b1 * filt.x2;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}

		Tfilt2order bandpassfilter_boris(double dt, int stepno, Tfilt2order filt, double x, bool bp = true)
		{ // The filter is coded from Matlabs transfer function with a differentiator and a compex pole pair
		  // if bp=true  => G_bp(s) = s / (1 + 2 *zeta/om * s + 1/om^2 * s^2) discretized with Tustin-Method
		  // if bp=false => G_bp(s) = 1 / (1 + 2 *zeta/om * s + 1/om^2 * s^2) discretized with Tustin-Method

			double om, a0, a1, a2, b0, b1, b2, help; // local vars
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.x3 = x;
				filt.y1 = 0;
				filt.y2 = 0;
				filt.y3 = 0;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x3 = filt.x2;
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y3 = filt.y2;
					filt.y2 = filt.y1;
				}
				om = 2 * pi * filt.f1; // omega: the natural frequency of the complex pole pairs

				a0 = dt * dt + 4 * dt * filt.zeta / om + 4 / om / om;
				a1 = (2 * dt * dt - 8 / om / om);
				a2 = (dt * dt - 4 * dt * filt.zeta / om + 4 / om / om);

				if (bp) { // used as bandpass filter (complex pole pair with differentiator)
					b0 = (2 * dt);
					b1 = 0;
					b2 = -(2 * dt);
				}
				else {
					// used as complex pole pair without differentiator
					b0 = dt * dt;
					b1 = 2 * dt * dt;
					b2 = dt * dt;

					if (om < 0) { // used as complex zero pair without integrator (by swapping a's and b's)
						help = b0;
						b0 = a0;
						a0 = help;
						help = b1;
						b1 = a1;
						a1 = help;
						help = b2;
						b2 = a2;
						a2 = help;
					}
				}

				filt.y1 = (-a1 * filt.y2 - a2 * filt.y3 + b0 * filt.x1 + b1 * filt.x2 + b2 * filt.x3) / a0;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}

		Tleadfilt integrator(double dt, int stepno, Tleadfilt filt, double x, bool integrator = true)
		{ // Simple integrator if true with G(s) = 1/s  and differentiator if false with G(s) = s
		  // discretized with Tustin-Method
			double a1, b0, b1; // local vars
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.y1 = 0;
				filt.y2 = 0;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y2 = filt.y1;
				}

				if (integrator) {
					a1 = -1;
					b0 = dt / 2;
					b1 = dt / 2;
				}
				else {
					a1 = 1;
					b0 = 2 / dt;
					b1 = -2 / dt;
				}

				filt.y1 = -a1 * filt.y2 + b0 * filt.x1 + b1 * filt.x2;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}

		Tfilt2order complexPoleAndZero(double dt, int stepno, Tfilt2order filt, double x)
		{ // might be equal to: filt2orderSolingen with f1 <-> f2 and zeta <-> beta switched 

		  // local vars
			double om1, om2, beta, zeta, a1, a2, b0, b1, b2, denom;
			// Step
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.x3 = x;
				filt.y1 = x;
				filt.y2 = x;
				filt.y3 = x;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x3 = filt.x2;
					filt.x2 = filt.x1;
					filt.x1 = x;
					filt.y3 = filt.y2;
					filt.y2 = filt.y1;
				}
				om1 = filt.f1 * 2 * pi;
				om2 = filt.f2 * 2 * pi;
				beta = filt.beta;
				zeta = filt.zeta;

				if (om2 > 0) { // usual case
					denom = dt * dt + 4 * zeta / om2 * dt + 4 / om2 / om2;
					a1 = (2 * dt * dt - 8 / om2 / om2) / denom;
					a2 = (dt * dt - 4 * zeta / om2 * dt + 4 / om2 / om2) / denom;
				}
				else { // used of only complex zero is desired
					denom = dt * dt;
					a1 = (2 * dt * dt) / denom;
					a2 = (dt * dt) / denom;
				} if (om1 > 0) { // usual case
					b0 = (dt * dt + 4 * beta / om1 * dt + 4 / om1 / om1) / denom;
					b1 = (2 * dt * dt - 8 / om1 / om1) / denom;
					b2 = (dt * dt - 4 * beta / om1 * dt + 4 / om1 / om1) / denom;
				}
				else { // used of only complex pole is desired
					b0 = (dt * dt) / denom;
					b1 = (2 * dt * dt) / denom;
					b2 = (dt * dt) / denom;
				}

				filt.y1 = -a1 * filt.y2 - a2 * filt.y3 + b0 * filt.x1 + b1 * filt.x2 + b2 * filt.x3;
			}
			filt.stepno1 = stepno;
			// Output
			return filt;
		}




		//**************************************************************************************************
		double switch_spline(double x, double x0, double x1)
		{  //A function that goes from 0 at x0 to 1 at x1
		   // -implicit none  // -real * 8 switch_spline, x, x0, x1
			double help_exchange;
			if (x0 > x1) { // usually this function works only if x0 is smaller x1
				help_exchange = x0;
				x0 = x1;
				x1 = help_exchange;
			}
			double switch_spline_val;
			if (x0 >= x1)																				// org: (x0 >= x1)
			{
				if (x < x0) switch_spline_val = 0.0;
				else	     switch_spline_val = 1.0;
			}
			else if (x0 > x1) switch_spline_val = 0.0;                                                  // THIS IS "DEAD CODE" IN THE ORIGINAL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			else {
				if (x < x0) switch_spline_val = 0.0;
				else if (x > x1) switch_spline_val = 1.0;
				else switch_spline_val = 2.0 / pow(-x1 + x0, 3) * x * x * x + (-3.0 * x0 - 3.0 * x1) / pow(-x1 + x0, 3) * x * x
					+ 6.0 * x1 * x0 / pow(-x1 + x0, 3) * x + (x0 - 3.0 * x1) * x0 * x0 / pow(-x1 + x0, 3);
			}
			return switch_spline_val;
		}
		//**************************************************************************************************
		double interpolate(double x, double x0, double x1, double f0, double f1)
		{					//-implicit none //-real * 8 interpolate, x, x0, x1, f0, f1
			double interpolate_val;
			if (x0 == x1) interpolate_val = f0;
			else		  interpolate_val = (x - x1) / (x0 - x1) * f0 + (x - x0) / (x1 - x0) * f1;

			return interpolate_val;
		}
		//**************************************************************************************************
		double GetOptiPitch(double wsp)
		{ // -implicit none // -real * 8 GetOptiPitch, wsp
		  //local vars
			double x, x0, x1, f0, f1, pitch, GetOptiPitch_val;
			int i = -1; // original fortran i = 1; but do while method in fortran checks expression first and then executes statement and c++ arrays start at 0
			// i ist the index of the optimal pitch angle in table OPdatavar

			// finde index for current wind speed
			do { i = i + 1; } while ((OPdatavar.wpdata[i][0] <= wsp) && (i <= OPdatavar.lines - 1)); //better with for???
			// min/max entry of table OPdatavar
			if (i == 1)					    GetOptiPitch_val = OPdatavar.wpdata[1][1];
			else if (i > OPdatavar.lines - 1) GetOptiPitch_val = OPdatavar.wpdata[OPdatavar.lines - 1][1];
			else // interpolation of current optimal pitch value
			{
				x = wsp;
				x0 = OPdatavar.wpdata[i - 1][0];
				x1 = OPdatavar.wpdata[i][0];
				f0 = OPdatavar.wpdata[i - 1][1];
				f1 = OPdatavar.wpdata[i][1];
				pitch = interpolate(x, x0, x1, f0, f1);
				GetOptiPitch_val = pitch;
			}
			return GetOptiPitch_val;
		}
		//**************************************************************************************************
		double GetInterpolatedValue(double wsp, Twpdata InterpolationData) // By ansfab to interpolate the Look-Up Tabels
		{ // -implicit none // -real * 8 GetOptiPitch, wsp
		  //local vars
			double x, x0, x1, f0, f1, pitch, GetOptiPitch_val;
			int i = -1;

			// finde index for current wind speed
			do { i = i + 1; } while ((InterpolationData.wpdata[i][0] <= wsp) && (i <= InterpolationData.lines - 1));
			// min/max entry of table
			if (i == 1)								  GetOptiPitch_val = InterpolationData.wpdata[1][1];
			else if (i > InterpolationData.lines - 1) GetOptiPitch_val = InterpolationData.wpdata[InterpolationData.lines - 1][1];
			else // interpolation of current optimal pitch value
			{
				x = wsp;
				x0 = InterpolationData.wpdata[i - 1][0];
				x1 = InterpolationData.wpdata[i][0];
				f0 = InterpolationData.wpdata[i - 1][1];
				f1 = InterpolationData.wpdata[i][1];
				pitch = interpolate(x, x0, x1, f0, f1);
				GetOptiPitch_val = pitch;
			}
			return GetOptiPitch_val;
		}
		//**************************************************************************************************
		Tfirstordervar lowpass1orderfilt(double dt, int stepno, Tfirstordervar filt, double x)
		{ //- implicit none //- integer * 4 stepno //- real * 8 lowpass1orderfilt, dt, x, y, a1, b1, b0, tau //-type(Tfirstordervar) filt

		  // local vars
			double y, a1, b1, b0, tau;
			//Step
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1_old = x;
				filt.y1_old = x;
				y = x;
			}
			else {
				if (stepno > filt.stepno1)
				{
					filt.x1_old = filt.x1;
					filt.y1_old = filt.y1;
				}
				tau = filt.tau;
				a1 = (2 * tau - dt) / (2 * tau + dt);
				b0 = dt / (2 * tau + dt);
				b1 = b0;
				y = a1 * filt.y1_old + b0 * x + b1 * filt.x1_old;
			}
			// Save previous values //are the values really saved or just local and lost??? maybe with "set" opterator??? strictly speaking in this context you don't need to return (or save) filt since filt is passed by reference so any changes to elements inside 'filt' will be seen outside the function....
			filt.x1 = x;
			filt.y1 = y;
			filt.stepno1 = stepno;
			// Output
			return filt;// y;	//- lowpass1orderfilt = y
		}
		//**************************************************************************************************
		/* original fortran translation. Hence funtions in C++ can not return arrays, a simple workaround is done below
		double(&lowpass2orderfilt(double dt, int stepno, Tlowpass2order filt, double x))[2]
		{ //- implicit none //- real * 8 lowpass2orderfilt(2), dt, x //- integer * 4 stepno //- type(Tlowpass2order) filt
		double lowpass2orderfilt_val[2]; // just used for return value
		// local vars // not local anyway???
		double y, f0, zeta, a1, a2, b0, b1, b2, denom;
		// Step
		if ((stepno == 1) && (stepno > filt.stepno1))
		{
		filt.x1 = x;
		filt.x2 = x;
		filt.x1_old = filt.x1;
		filt.x2_old = filt.x2;
		filt.y1 = x;
		filt.y2 = x;
		filt.y1_old = filt.y1;
		filt.y2_old = filt.y2;
		y = x;
		}
		else
		{
		if (stepno > filt.stepno1)
		{
		filt.x1_old = filt.x1;
		filt.x2_old = filt.x2;
		filt.y1_old = filt.y1;
		filt.y2_old = filt.y2;
		}
		f0 = filt.f0;
		zeta = filt.zeta;
		denom = 3.0 + 6.0*zeta*pi*f0*dt + 4.0*pi*pi * f0*f0 * dt*dt;
		a1 = (6.0 - 4.0*pi*pi * f0*f0 * dt*dt) / denom;
		a2 = (-3.0 + 6.0*zeta*pi*f0*dt - 4.0*pi*pi * f0*f0 * dt*dt) / denom;
		b0 = 4.0*pi*pi * f0*f0 * dt*dt / denom;
		b1 = b0;
		b2 = b0;
		y = a1 * filt.y1_old + a2 * filt.y2_old + b0 * x + b1 * filt.x1_old + b2 * filt.x2_old;
		}
		// Save previous values //are the values really saved or just local and lost???
		filt.x2 = filt.x1;
		filt.x1 = x;
		filt.y2 = filt.y1;
		filt.y1 = y;
		filt.stepno1 = stepno;
		//Output
		lowpass2orderfilt_val[0] = y;
		lowpass2orderfilt_val[1] = 0.5 * (y - filt.y2_old) / dt;
		return lowpass2orderfilt_val;
		} */

		Tlowpass2order lowpass2orderfiltFirstVal(double dt, int stepno, Tlowpass2order filt, double x)
		{ //- implicit none //- real * 8 lowpass2orderfilt(2), dt, x //- integer * 4 stepno //- type(Tlowpass2order) filt

		  // local vars
			double y, f0, zeta, a1, a2, b0, b1, b2, denom;
			// Step
			if ((stepno == 1) && (stepno > filt.stepno1))
			{
				filt.x1 = x;
				filt.x2 = x;
				filt.x1_old = filt.x1;
				filt.x2_old = filt.x2;
				filt.y1 = x;
				filt.y2 = x;
				filt.y1_old = filt.y1;
				filt.y2_old = filt.y2;
				y = x;
			}
			else
			{
				if (stepno > filt.stepno1)
				{
					filt.x1_old = filt.x1;
					filt.x2_old = filt.x2;
					filt.y1_old = filt.y1;
					filt.y2_old = filt.y2;
				}
				f0 = filt.f0;
				zeta = filt.zeta;
				denom = 3.0 + 6.0 * zeta * pi * f0 * dt + 4.0 * pi * pi * f0 * f0 * dt * dt;
				a1 = (6.0 - 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				a2 = (-3.0 + 6.0 * zeta * pi * f0 * dt - 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				b0 = 4.0 * pi * pi * f0 * f0 * dt * dt / denom;
				b1 = b0;
				b2 = b0;
				y = a1 * filt.y1_old + a2 * filt.y2_old + b0 * x + b1 * filt.x1_old + b2 * filt.x2_old;
			}
			// Save previous values //are the values really saved or just local and lost???
			filt.x2 = filt.x1;
			filt.x1 = x;
			filt.y2 = filt.y1;
			filt.y1 = y;
			filt.stepno1 = stepno;
			//Output
			//-- lowpass2orderfilt_val[0] = y;
			//-- lowpass2orderfilt_val[1] = 0.5 * (y - filt.y2_old) / dt;  //can be done in execution code if neccessary
			return filt; // y
		}
		/* easier / faster to do the single line inside execution code
		double getLowpass2orderfiltSecondVal(double dt, Tlowpass2order filt, double y) {
		return (0.5 * (y - filt.y2_old) / dt);
		}*/
		//**************************************************************************************************
		Tnotch2order notch2orderfilt(double dt, int stepno, Tnotch2order filt, double x)
		{ //- implicit none //- real * 8 notch2orderfilt, dt, x //- integer * 4 stepno //- type(Tnotch2order) filt
		  // local vars
			double y, f0, zeta1, zeta2, a1, a2, b0, b1, b2, denom;
			// Step
			if ((stepno == 1) && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				filt.x1_old = filt.x1;
				filt.x2_old = filt.x2;
				filt.y1 = x;
				filt.y2 = x;
				filt.y1_old = filt.y1;
				filt.y2_old = filt.y2;
				y = x;
			}
			else {
				if (stepno > filt.stepno1) {
					filt.x1_old = filt.x1;
					filt.x2_old = filt.x2;
					filt.y1_old = filt.y1;
					filt.y2_old = filt.y2;
				}
				f0 = filt.f0;
				zeta1 = filt.zeta1;
				zeta2 = filt.zeta2;
				denom = 3.0 + 6.0 * zeta1 * pi * f0 * dt + 4.0 * pi * pi * f0 * f0 * dt * dt;
				a1 = (6.0 - 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				a2 = (-3.0 + 6.0 * zeta1 * pi * f0 * dt - 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				b0 = (3.0 + 6.0 * zeta2 * pi * f0 * dt + 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				b1 = (-6.0 + 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				b2 = (3.0 - 6.0 * zeta2 * pi * f0 * dt + 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				y = a1 * filt.y1_old + a2 * filt.y2_old + b0 * x + b1 * filt.x1_old + b2 * filt.x2_old;
			}
			// Save previous values
			filt.x2 = filt.x1;
			filt.x1 = x;
			filt.y2 = filt.y1;
			filt.y1 = y;
			filt.stepno1 = stepno;
			// Output
			return filt; //- notch2orderfilt = y
		}
		//**************************************************************************************************
		Tbandpassfilt bandpassfilt(double dt, int stepno, Tbandpassfilt filt, double x)
		{ //- implicit none //- real * 8 bandpassfilt, dt, x //- integer * 4 stepno //- type(Tbandpassfilt) filt
		  //local vars
			double y, f0, zeta, tau, a1, a2, b0, b1, b2, denom;
			//Step
			if ((stepno == 1)) { // && (stepno > filt.stepno1)) {
				filt.x1 = x;
				filt.x2 = x;
				//filt.x1_old = filt.x1;
				//filt.x2_old = filt.x2;
				filt.y1 = x;
				filt.y2 = x;
				//filt.y1_old = filt.y1;
				//filt.y2_old = filt.y2;
				y = x;
			}
			else {
				//if (stepno > filt.stepno1) {
				//	filt.x1_old = filt.x1;
				//	filt.x2_old = filt.x2;
				//	filt.y1_old = filt.y1;
				//	filt.y2_old = filt.y2;
				//}
				f0 = filt.f0; // omega = 2*pi*f0
				zeta = filt.zeta;
				tau = filt.tau;
				denom = 3.0 + 6.0 * zeta * pi * f0 * dt + 4.0 * pi * pi * f0 * f0 * dt * dt;
				a1 = -(-6.0 + 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				a2 = -(3.0 - 6.0 * zeta * pi * f0 * dt + 4.0 * pi * pi * f0 * f0 * dt * dt) / denom;
				b0 = -(-6.0 * zeta * pi * f0 * dt - 12.0 * zeta * pi * f0 * tau) / denom;
				b1 = -24.0 * zeta * pi * f0 * tau / denom;
				b2 = -(6.0 * zeta * pi * f0 * dt - 12.0 * zeta * pi * f0 * tau) / denom;
				//y = a1 * filt.y1_old + a2 * filt.y2_old + b0 * x + b1 * filt.x1_old + b2 * filt.x2_old;
				y = a1 * filt.y1 + a2 * filt.y2 + b0 * x + b1 * filt.x1 + b2 * filt.x2;
			}
			// Save previous values
			filt.x2 = filt.x1;
			filt.x1 = x;
			filt.y2 = filt.y1;
			filt.y1 = y;
			filt.stepno1 = stepno;
			// Output
			return filt; //- bandpassfilt = y
		}
		//**************************************************************************************************
		Tpidvar PID(int stepno, double dt, double Kgain[3], Tpidvar PIDvar, double error)
		{ //- implicit none //- integer * 4 stepno //- real * 8 PID, dt, kgain(3), error //- type(Tpidvar) PIDvar
		  // Local vars
			double eps = 1.0E-6;
			// Initiate
			if (stepno == 1) {
				PIDvar.outset1 = 0;
				PIDvar.outres1 = 0;
				PIDvar.error1 = 0;
				PIDvar.error1_old = 0.0;
				PIDvar.outset1_old = 0.0;
				PIDvar.outres1_old = 0.0;
			}
			// Save previous values
			if (stepno > PIDvar.stepno1) { // is this ever necessary???
				PIDvar.outset1_old = PIDvar.outset1;
				PIDvar.outres1_old = PIDvar.outres1;
				PIDvar.error1_old = PIDvar.error1;
			}
			// Update the integral term
			PIDvar.outset = PIDvar.outset1_old + 0.5 * (error + PIDvar.error1) * Kgain[1] * PIDvar.Kint * dt;
			// Update proportional term
			PIDvar.outpro = Kgain[0] * PIDvar.Kpro * 0.5 * (error + PIDvar.error1);
			// Update differential term
			PIDvar.outdif = Kgain[2] * PIDvar.Kdif * (error - PIDvar.error1_old) / dt;
			// Sum to up
			PIDvar.outres = PIDvar.outset + PIDvar.outpro + PIDvar.outdif;
			// Satisfy hard limits
			if (PIDvar.outres < PIDvar.outmin)
				PIDvar.outres = PIDvar.outmin;
			else if (PIDvar.outres > PIDvar.outmax)
				PIDvar.outres = PIDvar.outmax;
			// Satisfy max velocity
			if (PIDvar.velmax > eps) {
				if ((abs(PIDvar.outres - PIDvar.outres1_old) / dt) > PIDvar.velmax)
					PIDvar.outres = PIDvar.outres1_old + copysign(PIDvar.velmax * dt, PIDvar.outres - PIDvar.outres1_old);
			}
			// Anti - windup on integral term and save results
			PIDvar.outset1 = PIDvar.outres - PIDvar.outpro - PIDvar.outdif;
			PIDvar.outres1 = PIDvar.outres;
			PIDvar.error1 = error;
			PIDvar.stepno1 = stepno;
			// Set output
			/*double PID_val;
			if (stepno == 0) PID_val = 0;
			else			 PID_val = PIDvar.outres;*/
			if (stepno == 0) PIDvar.outres = 0;
			return PIDvar; //PID_val
		}
		//**************************************************************************************************
		Tpid2var PID2(int stepno, double dt, double Kgain[3][2], Tpid2var PIDvar, double error[2])
		{ //- implicit none //- integer * 4 stepno //- real * 8 PID2, dt, kgain(3, 2), error(2) //- type(Tpid2var) PIDvar
		  // Local vars
			double eps = 1.0E-6;
			// Initiate
			if (stepno == 1)
			{
				//PIDvar.outset1 = 0;
				PIDvar.outres1 = 0;
				PIDvar.error1[0] = 0;
				PIDvar.error1[2] = 0;
				PIDvar.error1_old[0] = 0.0;
				PIDvar.error1_old[1] = 0.0;
				PIDvar.outset1_old = 0.0;
				PIDvar.outres1_old = 0.0;
			}
			// Save previous values
			if (stepno > PIDvar.stepno1)
			{
				PIDvar.outset1_old = PIDvar.outset1;
				PIDvar.outres1_old = PIDvar.outres1;
				PIDvar.error1_old[0] = PIDvar.error1[0];
				PIDvar.error1_old[1] = PIDvar.error1[1];
			}
			// Update the integral term
			PIDvar.outset = PIDvar.outset1_old + 0.50 * dt * (Kgain[1][0] * PIDvar.Kint[0] * (error[0] + PIDvar.error1[0])
				+ Kgain[1][1] * PIDvar.Kint[1] * (error[1] + PIDvar.error1[1]));
			// Update proportional term
			PIDvar.outpro = 0.5 * (Kgain[0][0] * PIDvar.Kpro[0] * (error[0] + PIDvar.error1[0])
				+ Kgain[0][1] * PIDvar.Kpro[1] * (error[1] + PIDvar.error1[1]));
			// Update differential term
			PIDvar.outdif = (Kgain[2][0] * PIDvar.Kdif[0] * (error[0] - PIDvar.error1_old[0])) / dt;
			// Sum to up
			PIDvar.outres = PIDvar.outset + PIDvar.outpro + PIDvar.outdif;
			// Satisfy hard limits
			if (PIDvar.outres < PIDvar.outmin)
				PIDvar.outres = PIDvar.outmin;
			else if (PIDvar.outres > PIDvar.outmax)
				PIDvar.outres = PIDvar.outmax;
			// Satisfy max velocity
			if (stepno > 3) // org. causes some issues: ((PIDvar.velmax > eps) && (stepno > 3)) // avoide issues during simulation beginnings
			{
				if ((abs(PIDvar.outres - PIDvar.outres1_old) / dt) > PIDvar.velmax)
					PIDvar.outres = PIDvar.outres1_old + copysign(PIDvar.velmax * dt, PIDvar.outres - PIDvar.outres1_old);
			}
			// Anti - windup on integral term and save results
			PIDvar.outset1 = PIDvar.outres - PIDvar.outpro - PIDvar.outdif;
			PIDvar.outres1 = PIDvar.outres;
			PIDvar.error1[0] = error[0];
			PIDvar.error1[1] = error[1];
			PIDvar.stepno1 = stepno;
			//if (stepno == 0) PIDvar.outres = 0;
			return PIDvar; // PID2_val;
		}
		//**************************************************************************************************
		//--};	//-	end module risoe_controller_fcns


		//************************************************************************************************//
		//************************************************************************************************//
		//************************************************************************************************//
		//************************************************************************************************//
		//								end risoe controller fcns.f90									  //
		//************************************************************************************************//
		//************************************************************************************************//
		//************************************************************************************************//
		//************************************************************************************************//











#ifdef __cplusplus
	};
}
#endif    // End of __cplusplus

#endif    // End of EXTERNAL_CONTROLLER_ API_H