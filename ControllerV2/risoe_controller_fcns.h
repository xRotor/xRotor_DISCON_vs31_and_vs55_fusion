#ifndef RISOE_CONTROLLER_FCNS_H
#define RISOE_CONTROLLER_FCNS_H

//#include "cmath" // TRY A WORKAROUND ONCE EVERYTHING IS RUNNING AS PLANNED !!!!

#include "GH_DISCON_Constants.h"


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
			int stepno1 = 0; double Kpro, Kdif, Kint, outmin, outmax, velmax, error1, outset1, outres1,
				outset, outpro, outdif, error1_old, outset1_old, outres1_old, outres;
		};
		struct Tpid2var {
			int stepno1 = 0; double outmin, outmax, velmax, outset1, outres1, outset, outpro, outdif, outset1_old, outres1_old, outres;
			double Kpro[2]; double Kdif[2]; double Kint[2]; double error1[2]; double error1_old[2];
		};
		struct Twpdata { int lines;   double wpdata[maxwplines][2]; };
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
		Tnotch2order   pitch_out_signal_filt;
		Tnotch2order   pitch_power_error_filt;
		//-------------------------------//
		Tbandpassfilt  DT_damper_filt;
		Tpid2var       PID_pit_var;
		Tpidvar        PID_gen_var;
		Twpdata        OPdatavar;
		Tfirstordervar wspfirstordervar;
		Tfirstordervar pitchfirstordervar;
		Tfirstordervar torquefirstordervar;
		Tfirstordervar switchfirstordervar;
		Tfirstordervar cutinfirstordervar;
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
				else switch_spline_val = 2.0 / pow(-x1 + x0, 3) * x*x*x + (-3.0*x0 - 3.0*x1) / pow(-x1 + x0, 3) * x*x
					+ 6.0*x1*x0 / pow(-x1 + x0, 3) * x + (x0 - 3.0*x1)*x0*x0 / pow(-x1 + x0, 3);
			}
			return switch_spline_val;
		}
		//**************************************************************************************************
		double interpolate(double x, double x0, double x1, double f0, double f1)
		{					//-implicit none //-real * 8 interpolate, x, x0, x1, f0, f1
			double interpolate_val;
			if (x0 == x1) interpolate_val = f0;
			else		  interpolate_val = (x - x1) / (x0 - x1)*f0 + (x - x0) / (x1 - x0)*f1;

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
			do { i = i + 1; } while ((OPdatavar.wpdata[i][0] <= wsp) && (i <= OPdatavar.lines-1)); //better with for???
																								 // min/max entry of table OPdatavar
			if (i == 1)					    GetOptiPitch_val = OPdatavar.wpdata[1][1];
			else if (i > OPdatavar.lines-1) GetOptiPitch_val = OPdatavar.wpdata[OPdatavar.lines-1][1];
			else // interpolation of current optimal pitch value
			{
				x = wsp;
				x0 = OPdatavar.wpdata[i-1][0];
				x1 = OPdatavar.wpdata[i  ][0];
				f0 = OPdatavar.wpdata[i-1][1];
				f1 = OPdatavar.wpdata[i  ][1];
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
				denom = 3.0 + 6.0*zeta1*pi*f0*dt + 4.0*pi*pi * f0*f0 * dt*dt;
				a1 = (6.0 - 4.0*pi*pi * f0*f0 * dt*dt) / denom;
				a2 = (-3.0 + 6.0*zeta1*pi*f0*dt - 4.0*pi*pi * f0*f0 * dt*dt) / denom;
				b0 = (3.0 + 6.0*zeta2*pi*f0*dt + 4.0*pi*pi * f0*f0 * dt*dt) / denom;
				b1 = (-6.0 + 4.0*pi*pi * f0*f0 * dt*dt) / denom;
				b2 = (3.0 - 6.0*zeta2*pi*f0*dt + 4.0*pi*pi * f0*f0 * dt*dt) / denom;
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
				zeta = filt.zeta;
				tau = filt.tau;
				denom = 3.0 + 6.0*zeta*pi*f0*dt + 4.0*pi*pi * f0*f0 * dt*dt;
				a1 = -(-6.0 + 4.0*pi*pi * f0*f0 * dt*dt) / denom;
				a2 = -(3.0 - 6.0*zeta*pi*f0*dt + 4.0*pi*pi * f0*f0 * dt*dt) / denom;
				b0 = -(-6.0*zeta*pi*f0*dt - 12.0*zeta*pi*f0*tau) / denom;
				b1 = -24.0*zeta*pi*f0*tau / denom;
				b2 = -(6.0*zeta*pi*f0*dt - 12.0*zeta*pi*f0*tau) / denom;
				y = a1 * filt.y1_old + a2 * filt.y2_old + b0 * x + b1 * filt.x1_old + b2 * filt.x2_old;
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
			if (stepno > PIDvar.stepno1) {
				PIDvar.outset1_old = PIDvar.outset1;
				PIDvar.outres1_old = PIDvar.outres1;
				PIDvar.error1_old = PIDvar.error1;
			}
			// Update the integral term
			PIDvar.outset = PIDvar.outset1_old + 0.5*(error + PIDvar.error1)*Kgain[1] * PIDvar.Kint*dt;
			// Update proportional term
			PIDvar.outpro = Kgain[0] * PIDvar.Kpro*0.5*(error + PIDvar.error1);
			// Update differential term
			PIDvar.outdif = Kgain[2] * PIDvar.Kdif*(error - PIDvar.error1_old) / dt;
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
					PIDvar.outres = PIDvar.outres1_old + copysign(PIDvar.velmax*dt, PIDvar.outres - PIDvar.outres1_old);
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
			PIDvar.outset = PIDvar.outset1_old + 0.50*dt*(Kgain[1][0] * PIDvar.Kint[0] * (error[0] + PIDvar.error1[0])
				+ Kgain[1][1] * PIDvar.Kint[1] * (error[1] + PIDvar.error1[1]));
			// Update proportional term
			PIDvar.outpro = 0.5*(Kgain[0][0] * PIDvar.Kpro[0] * (error[0] + PIDvar.error1[0])
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
					PIDvar.outres = PIDvar.outres1_old + copysign(PIDvar.velmax*dt, PIDvar.outres - PIDvar.outres1_old);
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