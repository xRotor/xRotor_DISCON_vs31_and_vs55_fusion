#pragma comment(lib, "ExternalControllerApi.lib")
#include "ExternalControllerApi.h" // defines the C functions for the external balded controller
#include "risoe_controller_fcns.h"
#include "iostream"
#include "cmath"
//#include "stdio.h""
using namespace GHTurbineInterface;

// ---- user specific controller parameters ---- //
//int UseInitialValues = 0, UseActivePTC = 0, UseLowPassWindSpeedFilter = 1, UseConstPowerProd = 0; // 1 ^= use; 0^= don't use
//int TimeToStopFeedback = 800;  // use to as start for static wind runs, otherwise put a high value (in seconds) (bigger 100)
//double Delta3_angle = 45;       // delta 3 angle in deg (just needed for active PTC)


//************************************************************************************************//
//************************************************************************************************//
//************************************************************************************************//
//									risoe controller fcns.f90									  //			
//************************************************************************************************//
//************************************************************************************************//
//************************************************************************************************//

/*
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
struct Tnotch2order	  { int stepno1 = 0; double zeta1 = 0.1, zeta2 = 0.001, f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old; };
struct Tbandpassfilt  { int stepno1 = 0; double zeta = 0.02, tau = 0.0, f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old; };
struct Tpidvar		  { int stepno1 = 0; double Kpro, Kdif, Kint, outmin, outmax, velmax, error1, outset1, outres1,
											outset, outpro, outdif, error1_old, outset1_old, outres1_old, outres; };
struct Tpid2var		  { int stepno1 = 0; double Kpro[2], Kdif[2], Kint[2], outmin, outmax, velmax, error1[2], outset1, outres1,
											outset, outpro, outdif, error1_old[2], outset1_old, outres1_old, outres; };
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
double t_cutin, t_generator_cutin, t_cutin_delay;
int    pitch_stoptype;
double t_cutout, pitch_stopdelay, pitch_stopdelay2, pitch_stopvelmax, pitch_stopvelmax2;
Tlowpass2order omega2ordervar;
Tnotch2order   DT_mode_filt;
Tnotch2order   pwr_DT_mode_filt;
Tbandpassfilt  DT_damper_filt;
Tpid2var       PID_pit_var;
Tpidvar        PID_gen_var;
Twpdata        OPdatavar;
Tfirstordervar wspfirstordervar;
Tfirstordervar pitchfirstordervar;
Tfirstordervar torquefirstordervar;
Tfirstordervar switchfirstordervar;
Tfirstordervar cutinfirstordervar;
//**************************************************************************************************
// The statement function in FORTRAN provided a means for a program unit to use a single line of FORTRAN as an internal function //- contains
//**************************************************************************************************
double switch_spline(double x, double x0, double x1)
{  //A function that goes from 0 at x0 to 1 at x1
	// -implicit none  // -real * 8 switch_spline, x, x0, x1
	float switch_spline_val;
	if (x0 >= x1)
	{
		if (x < x0) switch_spline_val = 0.0;
		else	     switch_spline_val = 1.0;
	}
	else if (x0 > x1) switch_spline_val = 0.0;
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
	int i = 0;  // original fortran i = 1; but do while method in fortran checks expression first and then executes statement
				// i ist the index of the optimal pitch angle in table OPdatavar

	// finde index for current wind speed
	do { i = i + 1; } while ((OPdatavar.wpdata[i][1] <= wsp) && (i <= OPdatavar.lines)); //better with for???
	// min/max entry of table OPdatavar
	if (i == 1)					  GetOptiPitch_val = OPdatavar.wpdata[1][2];
	else if (i > OPdatavar.lines) GetOptiPitch_val = OPdatavar.wpdata[OPdatavar.lines][2];
	else // interpolation of current optimal pitch value
	{
		x = wsp;
		x0 = OPdatavar.wpdata[i - 1][1];
		x1 = OPdatavar.wpdata[i][1];
		f0 = OPdatavar.wpdata[i - 1][2];
		f1 = OPdatavar.wpdata[i][2];
		pitch = interpolate(x, x0, x1, f0, f1);
		GetOptiPitch_val = pitch;
	}
	return GetOptiPitch_val;
}
//**************************************************************************************************
double lowpass1orderfilt(double dt, int stepno, Tfirstordervar filt, double x)
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
	return y;	//- lowpass1orderfilt = y
}
//**************************************************************************************************
*//* original fortran translation. Hence funtions in C++ can not return arrays, a simple workaround is done below
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
	lowpass2orderfilt_val[1] = y;
	lowpass2orderfilt_val[2] = 0.5 * (y - filt.y2_old) / dt;
	return lowpass2orderfilt_val;
} *//*

double lowpass2orderfiltFirstVal(double dt, int stepno, Tlowpass2order filt, double x)
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
	//-- lowpass2orderfilt_val[1] = y;
	//-- lowpass2orderfilt_val[2] = 0.5 * (y - filt.y2_old) / dt;  //can be done in execution code if neccessary
	return y;
}
*//* easier / faster to do the single line inside execution code
double getLowpass2orderfiltSecondVal(double dt, Tlowpass2order filt, double y) {
	return (0.5 * (y - filt.y2_old) / dt);
}*//*
//**************************************************************************************************
double notch2orderfilt(double dt, int stepno, Tnotch2order filt, double x)
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
	return y; //- notch2orderfilt = y
}
//**************************************************************************************************
double bandpassfilt(double dt, int stepno, Tbandpassfilt filt, double x)
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
	return y; //- bandpassfilt = y
}
//**************************************************************************************************
double PID(int stepno, double dt, double Kgain[3], Tpidvar PIDvar, double error)
{ //- implicit none //- integer * 4 stepno //- real * 8 PID, dt, kgain(3), error //- type(Tpidvar) PIDvar
	// Local vars
	float eps = 1.0E-6;
	double PID_val;
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
	PIDvar.outset = PIDvar.outset1_old + 0.5*(error + PIDvar.error1)*Kgain[2] * PIDvar.Kint*dt;
	// Update proportional term
	PIDvar.outpro = Kgain[1] * PIDvar.Kpro*0.5*(error + PIDvar.error1);
	// Update differential term
	PIDvar.outdif = Kgain[3] * PIDvar.Kdif*(error - PIDvar.error1_old) / dt;
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
	if (stepno == 0) PID_val = 0;
	else			 PID_val = PIDvar.outres;
	return PID_val;
}
//**************************************************************************************************
double PID2(int stepno, double dt, double Kgain[3][2], Tpid2var PIDvar, double error[2])
{ //- implicit none //- integer * 4 stepno //- real * 8 PID2, dt, kgain(3, 2), error(2) //- type(Tpid2var) PIDvar
	// Local vars
	float eps = 1.0E-6;
	double PID2_val;
	// Initiate
	if (stepno = 1)
	{
		PIDvar.outset1 = 0;
		PIDvar.outres1 = 0;
		PIDvar.error1[1] = 0;
		PIDvar.error1[2] = 0;
		PIDvar.error1_old[1] = 0.0;
		PIDvar.error1_old[2] = 0.0;
		PIDvar.outset1_old = 0.0;
		PIDvar.outres1_old = 0.0;
	}
	// Save previous values
	if (stepno > PIDvar.stepno1)
	{
		PIDvar.outset1_old = PIDvar.outset1;
		PIDvar.outres1_old = PIDvar.outres1;
		PIDvar.error1_old[1] = PIDvar.error1[1];
		PIDvar.error1_old[2] = PIDvar.error1[2];
	}
	// Update the integral term
	PIDvar.outset = PIDvar.outset1_old + 0.50*dt*(Kgain[2][1] * PIDvar.Kint[1] * (error[1] + PIDvar.error1[1])
		+ Kgain[2][2] * PIDvar.Kint[2] * (error[2] + PIDvar.error1[2]));
	// Update proportional term
	PIDvar.outpro = 0.5*(Kgain[1][1] * PIDvar.Kpro[1] * (error[1] + PIDvar.error1[1])
		+ Kgain[1][2] * PIDvar.Kpro[2] * (error[2] + PIDvar.error1[2]));
	// Update differential term
	PIDvar.outdif = (Kgain[3][1] * PIDvar.Kdif[1] * (error[1] - PIDvar.error1_old[1])) / dt;
	// Sum to up
	PIDvar.outres = PIDvar.outset + PIDvar.outpro + PIDvar.outdif;
	// Satisfy hard limits
	if (PIDvar.outres < PIDvar.outmin)
		PIDvar.outres = PIDvar.outmin;
	else if (PIDvar.outres > PIDvar.outmax)
		PIDvar.outres = PIDvar.outmax;
	// Satisfy max velocity
	if (PIDvar.velmax > eps)
	{
		if ((abs(PIDvar.outres - PIDvar.outres1_old) / dt) > PIDvar.velmax)
			PIDvar.outres = PIDvar.outres1_old + copysign(PIDvar.velmax*dt, PIDvar.outres - PIDvar.outres1_old);
	}
	// Anti - windup on integral term and save results
	PIDvar.outset1 = PIDvar.outres - PIDvar.outpro - PIDvar.outdif;
	PIDvar.outres1 = PIDvar.outres;
	PIDvar.error1[1] = error[1];
	PIDvar.error1[2] = error[2];
	PIDvar.stepno1 = stepno;
	// Set output
	if (stepno == 0) PID2_val = 0;
	else			 PID2_val = PIDvar.outres;
	return PID2_val;
}
//**************************************************************************************************
//--};	//-	end module risoe_controller_fcns

*/
//************************************************************************************************//
//************************************************************************************************//
//************************************************************************************************//
//************************************************************************************************//
//								end risoe controller fcns.f90									  //
//************************************************************************************************//
//************************************************************************************************//
//************************************************************************************************//
//************************************************************************************************//






double y_wsp, a1, b1, b0, tau;



// Local variables
double time, omega, omegafilt, domega_dt_filt, wsp, WSPfilt,
omega_err_filt_pitch, omega_err_filt_speed, omega_dtfilt,
ommin1, ommin2, ommax1, ommax2,
pitang[3], meanpitang, meanpitangfilt, theta_min, e_pitch[2],
kgain_pitch[3][2], kgain_torque[3], aero_gain, x, dummy, y[2],
Qg_min_partial, Qg_max_partial, Qg_min_full, Qg_max_full,
Qgen_ref, theta_col_ref, thetaref[3], Pe_ref, Qdamp_ref;

double nominalWindSpeed;																	// usage of wind input unclear...
double SecondWindInput = 0;  // Inp 6.2														// NO CLUE WHAT THIS VALUE SHOULD BE ???!!!







// Log messages
//int CheckTime = 50, CheckTimeInterv = 5;  // Time and time interval to check control inputs in s (to avoid information overflow) 
char contmodestring[40], contmodeoldstring[40], LogText[40];

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
		ReportInfoMessage(turbine_id, "Running DTU 20MW RWT controller.");
		
		// start subroutine init_regulation
		//-- void init_regulation(double array1[100], double array2[1]) {																							
			 
		// DEC$ ATTRIBUTES DLLEXPORT, C, ALIAS:’init_regulation’::init_regulation
		//- use risoe_controller_fcns
		//--  risoe_controller_fcns;

		// forces all variables to explicite declared (how in in c++?) //- implicit none
		// double array1[100] = { }, array2[1]; // should basically be an input !!!!!
		//- real * 8 array1(1000), array2(1)
		// Local vars
		int i, ifejl;
		char* text32 = "text32";
		double minimum_pitch_angle;
		bool findes;

		// Input array1 must contain:
		//					; Overall parameters
		// 1: constant 1	; Rated power[kW]
		// 2: constant 2	; Minimum rotor speed[rad / s]
		// 3: constant 3	; Rated rotor speed[rad / s]
		// 4: constant 4	; Maximum allowable generator torque[Nm]
		// 5: constant 5	; Minimum pitch angle, theta_min[deg],
		//					; if | theta_min | >90, then a table of <wsp, theta_min> is read
		//					; from a file named ’wptable.n’, where n = int(theta_min)
		// 6: constant 6	; Maximum pitch angle[deg]
		// 7: constant 7	; Maximum pitch velocity operation[deg / s]
		// 8: constant 8	; Frequency of generator speed filter[Hz]
		// 9: constant 9	; Damping ratio of speed filter[-]
		// 10: constant 10	; Frequency of free - free DT torsion mode[Hz], if zero no notch filter used
		//					; Partial load control parameters
		// 11: constant 11	; Optimal Cp tracking K factor[Nm / (rad / s) ^ 2],
		//					; Qg = K * Omega ^ 2, K = eta * 0.5*rho*A*Cp_opt*R ^ 3 / lambda_opt ^ 3
		// 12: constant 12	; Proportional gain of torque controller[Nm / (rad / s)]
		// 13: constant 13	; Integral gain of torque controller[Nm / rad]
		// 14: constant 14	; Differential gain of torque controller[Nm / (rad / s ^ 2)]
		//					; Full load control parameters
		// 15: constant 15	; Generator control switch[1 = constant power, 2 = constant torque]
		// 16: constant 16	; Proportional gain of pitch controller[rad / (rad / s)]
		// 17: constant 17	; Integral gain of pitch controller[rad / rad]
		// 18: constant 18	; Differential gain of pitch controller[rad / (rad / s ^ 2)]
		// 19: constant 19	; Proportional power error gain [rad/W]
		// 20: constant 20	; Integral power error gain [rad/(Ws)]
		// 21: constant 21	; Coefficient of linear term in aerodynamic gain scheduling, KK1 [deg]
		// 22: constant 22	; Coefficient of quadratic term in aerodynamic gain scheduling, KK2 [deg^2]
		//					; (if zero, KK1 = pitch angle at double gain)
		// 23: constant 23	; Relative speed for double nonlinear gain [-]
		//					; Cut-in simulation parameters
		// 24: constant 24	; Cut-in time [s], if zero no cut-in simulated
		// 25: constant 25	; Time delay for soft start [1/1P]
		//					; Cut-out simulation parameters
		// 26: constant 26	; Cut-out time [s], if zero no cut-out simulated
		// 27: constant 27	; Time constant for 1st order filter lag of torque cut-out [s]
		// 28: constant 28	; Stop type [1=linear two pitch speed stop, 2=exponential pitch speed stop]
		// 29: constant 29	; Time delay for pitch stop 1 [s]
		// 30: constant 30	; Maximum pitch velocity during stop 1 [deg/s]
		// 31: constant 31	; Time delay for pitch stop 2 [s]
		// 32: constant 32	; Maximum pitch velocity during stop 2 [deg/s]
		//					; Expert parameters (keep default values unless otherwise given)
		// 33 constant 33	; Lower angle above lowest minimum pitch angle for switch [deg]
		// 34: constant 34	; Upper angle above lowest minimum pitch angle for switch [deg]
		// 35: constant 35	; Ratio between filtered and reference speed for fully open torque limits [%]
		// 36: constant 36	; Time constant of 1st order filter on wind speed used for minimum pitch [1/1P]
		// 37: constant 37	; Time constant of 1st order filter on pitch angle for gain scheduling [1/1P]
		// 38: constant 38	; Proportional gain of DT damper [Nm/(rad/s)], requires frequency in input 10


		// Overall parameters
		Pe_rated =		20000.00 * 1000;	// constant 1	; Rated power[kW]
		omega_ref_min = 0.45000;			// constant 2	; Minimum rotor speed[rad / s]
		omega_ref_max = 0.74617;			// constant 3	; Rated rotor speed[rad / s]
		max_lss_torque = 4.4123E+07;		// constant 4	; Maximum allowable generator torque[Nm]
		minimum_pitch_angle = 100 * degrad; // constant 5	; Minimum pitch angle, theta_min[deg],
											//				; if | theta_min | >90, then a table of <wsp, theta_min> is read
											//				; from a file named ’wptable.n’, where n = int(theta_min)  // AND WHERE IS THAT FILE???!!!
		pitch_stopang = 90 * degrad;		// constant 6	; Maximum pitch angle[deg]
		PID_pit_var.velmax = 7.071 * degrad;// constant 7	; Maximum pitch velocity operation[deg / s]
		omega2ordervar.f0 = 0.14142;		// constant 8	; Frequency of generator speed filter[Hz]
		omega2ordervar.zeta = 0.7;			// constant 9	; Damping ratio of speed filter[-]
		DT_mode_filt.f0 = 0.35000;			// constant 10	; Frequency of free - free DT torsion mode[Hz], if zero no notch filter used


		Pe_rated = 3524.36 * 1800 / 60;		// CART2   // no gearbox in 20 MW RWT
		omega_ref_min = 1295 * 2 * pi / 60; // 1295/43.165*2*pi/60;// CART2
		omega_ref_max = 1800 * 2 * pi / 60;	//41.7*2*pi/60;		// CART2
		max_lss_torque = 3524.36;// *43.165;	// CART2
		minimum_pitch_angle = 0;			// CART2

		// Partial load control parameters
		Kopt = 5.1376E+07;					// constant 11	; Optimal Cp tracking K factor[Nm / (rad / s) ^ 2],
		//					; Qg = K * Omega ^ 2, K = eta * 0.5*rho*A*Cp_opt*R ^ 3 / lambda_opt ^ 3
		if (Kopt* omega_ref_max * omega_ref_max >= Pe_rated / omega_ref_max) Kopt = Pe_rated / pow(omega_ref_max, 3);
		PID_gen_var.Kpro = 2.73E+08;		// constant 12	; Proportional gain of torque controller[Nm / (rad / s)]
		PID_gen_var.Kint = 4.34E+07;		// constant 13	; Integral gain of torque controller[Nm / rad]
		PID_gen_var.Kdif = 0;				// constant 14	; Differential gain of torque controller[Nm / (rad / s ^ 2)]

		// Full load control parameters
		const_power = (int(2) == 1);		// constant 15	; Generator control switch[1 = constant power, 2 = constant torque]
		PID_pit_var.Kpro[1] = 0.5245;		// constant 16	; Proportional gain of pitch controller[rad / (rad / s)]
		PID_pit_var.Kint[1] = 0.0999;		// constant 17	; Integral gain of pitch controller[rad / rad]
		PID_pit_var.Kdif[1] = 0;			// constant 18	; Differential gain of pitch controller[rad / (rad / s ^ 2)]
		PID_pit_var.Kpro[2] = 2.00E-09;		// constant 19	; Proportional power error gain [rad/W]
		PID_pit_var.Kint[2] = 1.41E-09;		// constant 20	; Integral power error gain [rad/(Ws)]
		PID_pit_var.Kdif[2] = 0.0;
		kk1 = 198.329 * degrad;				// constant 21	; Coefficient of linear term in aerodynamic gain scheduling, KK1 [deg]
		kk2 = 693.222 * degrad*degrad;		// constant 22	; Coefficient of quadratic term in aerodynamic gain scheduling, KK2 [deg^2]
											//				; (if zero, KK1 = pitch angle at double gain)
		rel_limit = 1.3;					// constant 23	; Relative speed for double nonlinear gain [-]

		// Cut - in simulation parameters
		t_cutin = 0.1414;					// constant 24	; Cut-in time [s], if zero no cut-in simulated
		t_cutin_delay = 5.6569 * 2.0 * pi / omega_ref_max; // constant 25	; Time delay for soft start [1/1P]

		// Cut - out simulation parameters
		t_cutout = 0;						// constant 26	; Cut-out time [s], if zero no cut-out simulated
		torquefirstordervar.tau = 0.1414;	// constant 27	; Time constant for 1st order filter lag of torque cut-out [s]
		pitch_stoptype = int(1);			// constant 28	; Stop type [1=linear two pitch speed stop, 2=exponential pitch speed stop]
		pitch_stopdelay = 1.4142;			// constant 29	; Time delay for pitch stop 1 [s]
		pitch_stopvelmax = 14.1421 * degrad;// constant 30	; Maximum pitch velocity during stop 1 [deg/s]
		pitch_stopdelay2 = 1.4142;			// constant 31	; Time delay for pitch stop 2 [s]
		pitch_stopvelmax2 = 7.0711 * degrad;// constant 32	; Maximum pitch velocity during stop 2 [deg/s]

		// Expert parameters(keep default values unless otherwise given)
		switch1_pitang_lower = 0.5 * degrad;// constant 33	; Lower angle above lowest minimum pitch angle for switch [deg]
		switch1_pitang_upper = 0.5 * degrad;// constant 34	; Upper angle above lowest minimum pitch angle for switch [deg]
		rel_sp_open_Qg = 95 * 0.01;			// constant 35	; Ratio between filtered and reference speed for fully open torque limits [%]
		wspfirstordervar.tau = 7.0711 * 2.0 * pi / omega_ref_max;	// constant 36	; Time constant of 1st order filter on wind speed used for minimum pitch [1/1P]
		pitchfirstordervar.tau = 7.0711 * 2.0 * pi / omega_ref_max; // constant 37	; Time constant of 1st order filter on pitch angle for gain scheduling [1/1P]

		// Drivetrain damper
		DT_damp_gain = 0;					// constant 38	; Proportional gain of DT damper [Nm/(rad/s)], requires frequency in input 10   // NOT GIVEN !!!!!!!
		DT_damper_filt.f0 = DT_mode_filt.f0;
		pwr_DT_mode_filt.f0 = DT_mode_filt.f0;

		// Default and derived parameters
		PID_gen_var.velmax = 0.0; // No limit to generator torque change rate
		Qg_rated = Pe_rated / omega_ref_max;
		switchfirstordervar.tau = 2.0 * pi / omega_ref_max;
		cutinfirstordervar.tau = 2.0 * pi / omega_ref_max;



		// Wind speed table
		if (abs(minimum_pitch_angle) < 90.0 * degrad)
		{	// minimal pitch angle for all wind speeds, but why???
			OPdatavar.lines = 2;
			OPdatavar.wpdata[1][1] = 0.0;
			OPdatavar.wpdata[2][1] = 99.0;
			OPdatavar.wpdata[1][2] = minimum_pitch_angle;
			OPdatavar.wpdata[2][2] = minimum_pitch_angle;
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

		// Just a fast work around
		OPdatavar.lines = 9;
		OPdatavar.wpdata[1][1] = 0;
		OPdatavar.wpdata[2][1] = 7;
		OPdatavar.wpdata[3][1] = 8;
		OPdatavar.wpdata[4][1] = 9;
		OPdatavar.wpdata[5][1] = 10;
		OPdatavar.wpdata[6][1] = 13;
		OPdatavar.wpdata[7][1] = 18;
		OPdatavar.wpdata[8][1] = 22;
		OPdatavar.wpdata[9][1] = 26;
		OPdatavar.wpdata[1][2] = -1 * degrad;
		OPdatavar.wpdata[2][2] = -1 * degrad;
		OPdatavar.wpdata[3][2] =  0 * degrad;
		OPdatavar.wpdata[4][2] =  1 * degrad;
		OPdatavar.wpdata[5][2] =  3 * degrad;
		OPdatavar.wpdata[6][2] =  6 * degrad;
		OPdatavar.wpdata[7][2] = 11 * degrad;
		OPdatavar.wpdata[8][2] = 18 * degrad;
		OPdatavar.wpdata[9][2] = 24 * degrad;

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
		AddLogValue  ( turbine_id, "e_pitch[2]", "W");				// 11 : Power error for pitch[W]
		AddLogValue  ( turbine_id, "PID_pit_var.outpro", "rad");	// 12 : Proportional term of pitch controller[rad]
		AddLogValue  ( turbine_id, "PID_pit_var.outset", "rad");		// 13 : Integral term of pitch controller[rad]
		AddLogValue  ( turbine_id, "PID_pit_var.outmin", "rad");	// 14 : Minimum limit of pitch[rad]
		AddLogValue  ( turbine_id, "PID_pit_var.outmax", "rad");	// 15 : Maximum limit of pitch[rad]
		AddLogValue  ( turbine_id, "Qdamp_ref", "Nm");				// 16 : Torque reference from DT damper[Nm]

		

  		return CONTROLLER(turbine_id);
	}



	/*! Controller to be run on subsequent timesteps. */
	int __declspec( dllexport ) __cdecl CONTROLLER (const turbine turbine_id)
	{

		/*// --- import values from bladed --- //
		hdr[HSSTORQUE].fastval =  GetMeasuredGeneratorTorque(turbine_id);       // Returns the measured generator torque in Nm. )
		hdr[HSSRPM].value	     =  GetMeasuredGeneratorSpeed(turbine_id )*30/PI; // Returns the measured generator speed. Measured in rad/s.
		hdr[HSSRPMDOT].fastval =  (hdr[HSSRPM].value - hdr[HSSRPM].lastval);// / 0.01;  // discretized deviation of HSSRPM for a simulation with 0.01s time step. */ 	  


		// void update_regulation(double array1[1000], double array2[100]) {							//- subroutine update_regulation(array1, array2)
		
		//- implicit none
		//--risoe_controller_fcns;
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
		omega = GetMeasuredRotorSpeed(turbine_id);		 // Inp 2 // Returns the measured rotor speed in rad/s. (NOT GENERATOR LSS!!!)
		omega = GetMeasuredGeneratorSpeed(turbine_id);   // CART2 //20MW RWT has no gear box
		//Mean pitch angle
		pitang[1] = GetMeasuredPitchAngle(turbine_id,0); // Inp 3 // Returns the current pitch angle of the blade in rad.
		pitang[2] = GetMeasuredPitchAngle(turbine_id,1); // Inp 4
		pitang[3] = GetMeasuredPitchAngle(turbine_id,1); // Inp 5
		meanpitang = (pitang[1] + pitang[2] + pitang[3]) / 3.0;
		// Wind speed as horizontal vector sum
		nominalWindSpeed = GetNominalHubFlowSpeed(turbine_id); // Inp 6.1 // Returns the modelled speed of the flow over the hub. 
		 // This is based on free flow at hub position - there is no modelling of actual nacelle anemometer or pitot tube. Measured in m/s.
		wsp = sqrt(nominalWindSpeed * nominalWindSpeed + SecondWindInput * SecondWindInput);
		// Low - pass filtering of the rotor speed
		//-- y = lowpass2orderfilt(deltat, stepno, omega2ordervar, omega);			// array as function output is not possible in c++!!!
		y[1] = lowpass2orderfiltFirstVal(deltat, stepno, omega2ordervar, omega);
		omegafilt = y[1];
		//y[2] = getLowpass2orderfiltSecondVal(deltat, omega2ordervar, y[1]);				// has to follow lowpass2orderfiltFirstVal!!!
		y[2] = 0.5 * (y[1] - omega2ordervar.y2_old) / deltat;	// isnt value somewhat???	// has to follow lowpass2orderfiltFirstVal!!!
		domega_dt_filt = y[2];									// isnt value somewhat???
		
		
		// Mean pitch angle
		// Low - pass filtering of the mean pitch angle for gain scheduling
		meanpitangfilt = fmin(lowpass1orderfilt(deltat, stepno, pitchfirstordervar, meanpitang), 30.0* degrad);
		// Low - pass filtering of the nacelle wind speed
		//WSPfilt = lowpass1orderfilt(deltat, stepno, wspfirstordervar, wsp);
		WSPfilt = lowpass1orderfilt(deltat, stepno, wspfirstordervar, wsp);

		// Minimum pitch angle may vary with filtered wind speed
		theta_min = GetOptiPitch(WSPfilt);
		switch1_pitang_lower = switch1_pitang_lower + theta_min;
		switch1_pitang_upper = switch1_pitang_upper + theta_min;
		//**************************************************************************************************//
		//  Speed ref.changes max. <->min. for torque controller and remains at rated for pitch controller  //
		//**************************************************************************************************//
		if (omegafilt > 0.5 * (omega_ref_max + omega_ref_min))
			omega_err_filt_speed = omegafilt - omega_ref_max;
		else omega_err_filt_speed = omegafilt - omega_ref_min;

		//**************************************************************************************************
		//PID regulation of generator torque
		//**************************************************************************************************
		//Limits for full load
		if (const_power)
		{
			Qg_min_full = fmin(Pe_rated / fmax(omega, 1.0E15), max_lss_torque);
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

		// Switch based on pitch
		double switch1 = switch_spline(meanpitang, switch1_pitang_lower, switch1_pitang_upper);
		switch1 = lowpass1orderfilt(deltat, stepno, switchfirstordervar, switch1);

		// Interpolation between partial and full load torque limits based on switch 1
		PID_gen_var.outmin = (1.0 - switch1)*Qg_min_partial + switch1 * Qg_min_full;
		PID_gen_var.outmax = (1.0 - switch1)*Qg_max_partial + switch1 * Qg_max_full;
		if (PID_gen_var.outmin > PID_gen_var.outmax) PID_gen_var.outmin = PID_gen_var.outmax;

		//Compute PID feedback to generator torque
		kgain_torque[1] = 1.0;
		kgain_torque[2] = 1.0;
		kgain_torque[3] = 1.0;
		Qgen_ref = PID(stepno, deltat, kgain_torque, PID_gen_var, omega_err_filt_speed);

		//------------------------------------------------------------------------------------------------//
		//						Control of cut - in regarding generator torque							  //
		//------------------------------------------------------------------------------------------------//
		if (t_cutin > 0.0) {
			if (generator_cutin) {
				x = switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay);
				Qgen_ref = Qgen_ref * x;
			}
		}

		//------------------------------------------------------------------------------------------------//
		//						Control of cut - out regarding generator torque							  //
		//------------------------------------------------------------------------------------------------//
		if ((t_cutout > 0.0) && (time > t_cutout))
			Qgen_ref = lowpass1orderfilt(deltat, stepno, torquefirstordervar, 0.0);
		else dummy = lowpass1orderfilt(deltat, stepno, torquefirstordervar, Qgen_ref);

		//------------------------------------------------------------------------------------------------//
		//									Reference electrical power									  //
		//------------------------------------------------------------------------------------------------//
		Pe_ref = Qgen_ref * omega;

		//------------------------------------------------------------------------------------------------//
		//				Active DT (drive train) damping based on notch filtered of rotor speed			  //
		//------------------------------------------------------------------------------------------------//
		if ((DT_damp_gain > 0.0) && (DT_damper_filt.f0 > 0.0))
		{
			omega_dtfilt = bandpassfilt(deltat, stepno, DT_damper_filt, omega);																			// i guess this fct can be found in risoe_controller_fcns
			if (t_cutin > 0.0)
			{
				if (generator_cutin)
				{
					x = switch_spline(time, t_generator_cutin + t_cutin_delay, t_generator_cutin + 2.0 * t_cutin_delay);
					Qdamp_ref = DT_damp_gain * omega_dtfilt * x;
					Qgen_ref = fmin(fmax(Qgen_ref + Qdamp_ref, 0.0), max_lss_torque);
				}
			}
			else
			{
				Qdamp_ref = DT_damp_gain * omega_dtfilt;
				Qgen_ref = fmin(fmax(Qgen_ref + Qdamp_ref, 0.0), max_lss_torque);
			}
		}
		
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
		kgain_pitch[1][1] = help_var;
		kgain_pitch[2][1] = help_var;
		kgain_pitch[3][1] = help_var;
		kgain_pitch[1][2] = help_var;
		kgain_pitch[2][2] = help_var;
		kgain_pitch[3][2] = help_var;
		//------------------------------------------------------------------------------------------------//
		//								Control of cut - in regarding pitch								  //
		//------------------------------------------------------------------------------------------------//
		if (t_cutin > 0.0)
		{
			if (time < t_cutin)
			{
				PID_pit_var.outmin = pitch_stopang;
				PID_pit_var.outmax = pitch_stopang;
				//--kgain_pitch = 0.0;
				kgain_pitch[1][1] = 0.0;
				kgain_pitch[2][1] = 0.0;
				kgain_pitch[3][1] = 0.0;
				kgain_pitch[1][2] = 0.0;
				kgain_pitch[2][2] = 0.0;
				kgain_pitch[3][2] = 0.0;
				dummy = lowpass1orderfilt(deltat, stepno, cutinfirstordervar, omega - omega_ref_min);
			}
			else
			{
				if (!generator_cutin)
				{
					kgain_pitch[1][1] = 0.25 * kgain_pitch[1][1];
					kgain_pitch[2][1] = 0.25 * kgain_pitch[2][1];
					kgain_pitch[3][1] = 0.25 * kgain_pitch[3][1];
					kgain_pitch[1][2] = 0.0;
					kgain_pitch[2][2] = 0.0;
					kgain_pitch[3][2] = 0.0;
					omega_err_filt_pitch = omegafilt - omega_ref_min;
					x = lowpass1orderfilt(deltat, stepno, cutinfirstordervar, omega - omega_ref_min);
					if (abs(x) < omega_ref_min * 0.01)
					{
						generator_cutin = true;
						t_generator_cutin = time;
					}
				}
				else
				{
					x = switch_spline(time, t_generator_cutin, t_generator_cutin + t_cutin_delay);
					omega_err_filt_pitch = omegafilt - (omega_ref_min*(1.0 - x) + omega_ref_max * x);
					kgain_pitch[1][1] = 0.25*kgain_pitch[1][1] + kgain_pitch[1][1] * 0.75 * x;
					kgain_pitch[2][1] = 0.25*kgain_pitch[2][1] + kgain_pitch[2][1] * 0.75 * x;
					kgain_pitch[3][1] = 0.25*kgain_pitch[3][1] + kgain_pitch[3][1] * 0.75 * x;
					kgain_pitch[1][2] = kgain_pitch[1][2] * x;
					kgain_pitch[2][2] = kgain_pitch[2][2] * x;
					kgain_pitch[3][2] = kgain_pitch[3][2] * x;
				}
			}
		}

		//------------------------------------------------------------------------------------------------//
		//							Control of cut - out regarding pitch								  //
		//------------------------------------------------------------------------------------------------//
		if ((t_cutout > 0.0) && (time > t_cutout + pitch_stopdelay))
		{
			switch (pitch_stoptype) {
			case 1: //Normal 2 - step stop situation
				PID_pit_var.outmax = pitch_stopang;
				PID_pit_var.outmin = pitch_stopang;
				if (time > t_cutout + pitch_stopdelay + pitch_stopdelay2)
					PID_pit_var.velmax = pitch_stopvelmax2;
				else PID_pit_var.velmax = pitch_stopvelmax;

			case 2: //Exponential decay approach
				PID_pit_var.outmax = pitch_stopang;
				PID_pit_var.outmin = pitch_stopang;
				if ((time - (t_cutout + pitch_stopdelay)) / pitch_stopdelay2 < 10.0)
					PID_pit_var.velmax = pitch_stopang / pitch_stopdelay2
					* exp(-(time - (t_cutout + pitch_stopdelay)) / pitch_stopdelay2);
				else PID_pit_var.velmax = 0.0;

				if (PID_pit_var.velmax > pitch_stopvelmax)  PID_pit_var.velmax = pitch_stopvelmax;
				if (PID_pit_var.velmax < pitch_stopvelmax2) PID_pit_var.velmax = pitch_stopvelmax2;
			default:
				char* ErrorMess;
				sprintf(ErrorMess, " *** ERROR *** Stop type %i not known", pitch_stoptype);
				ReportInfoMessage(turbine_id, ErrorMess);
				//- write(6, ’(a, i2, a)’) ’ *** ERROR *** Stop type ’, pitch_stoptype, ’ not known’
				exit(0);
			}
		}

		//------------------------------------------------------------------------------------------------//
		//							Compute PID feedback to generator torque							  //
		//------------------------------------------------------------------------------------------------//
		if (DT_mode_filt.f0 > 0.0)
		{
			e_pitch[1] = notch2orderfilt(deltat, stepno, DT_mode_filt, omega_err_filt_pitch);
			e_pitch[2] = notch2orderfilt(deltat, stepno, pwr_DT_mode_filt, Pe_ref - Pe_rated);
		}
		else
		{
			e_pitch[1] = omega_err_filt_pitch;
			e_pitch[2] = Pe_ref - Pe_rated;
		}
		theta_col_ref = PID2(stepno, deltat, kgain_pitch, PID_pit_var, e_pitch);
		thetaref[1] = theta_col_ref;
		thetaref[2] = theta_col_ref;
		thetaref[3] = theta_col_ref;
		//************************************************************************************************//
		//												Output											  //
		//************************************************************************************************//
		SetDemandedGeneratorTorque(turbine_id, Qgen_ref);		// 1 : Generator torque reference[Nm]
		SetDemandedPitchAngle(turbine_id, 0, thetaref[1]);		// 2 : Pitch angle reference of blade 1[rad]
		SetDemandedPitchAngle(turbine_id, 1, thetaref[2]);		// 3 : Pitch angle reference of blade 2[rad]
		//SetDemandedPitchAngle(turbine_id, 2, thetaref[3]);	// 4 : Pitch angle reference of blade 3[rad]


		
		SetLoggingValue(turbine_id, 0, Pe_ref);					// 5 : Power reference[W]
		SetLoggingValue(turbine_id, 1, WSPfilt);				// 6 : Filtered wind speed[m / s]
		SetLoggingValue(turbine_id, 2, wsp); // omegafilt);				// 7 : Filtered rotor speed[rad / s]
		SetLoggingValue(turbine_id, 3, wspfirstordervar.x1_old); // omega_err_filt_speed);	// 8 : Filtered rotor speed error for torque[rad / s]
		SetLoggingValue(turbine_id, 4, wspfirstordervar.x1); //  omega_dtfilt);			// 9 : Bandpass filtered rotor speed[rad / s]
		SetLoggingValue(turbine_id, 5, wspfirstordervar.y1_old); // PID_gen_var.outpro);		// 10 : Proportional term of torque contr.[Nm]
		SetLoggingValue(turbine_id, 6, wspfirstordervar.y1);//  PID_gen_var.outset);		// 11 : Integral term of torque controller[Nm]
		SetLoggingValue(turbine_id, 7, PID_gen_var.outmin);		// 12 : Minimum limit of torque[Nm]
		SetLoggingValue(turbine_id, 8, PID_gen_var.outmax);		// 13 : Maximum limit of torque[Nm]
		SetLoggingValue(turbine_id, 9, switch1);				// 14 : Torque limit switch based on pitch[-]
		SetLoggingValue(turbine_id, 10, omega_err_filt_pitch);	// 15 : Filtered rotor speed error for pitch[rad / s]
		SetLoggingValue(turbine_id, 11, e_pitch[2]);			// 16 : Power error for pitch[W]
		SetLoggingValue(turbine_id, 12, PID_pit_var.outpro);	// 17 : Proportional term of pitch controller[rad]
		SetLoggingValue(turbine_id, 13, PID_pit_var.outset);	// 18 : Integral term of pitch controller[rad]
		SetLoggingValue(turbine_id, 14, PID_pit_var.outmin);	// 19 : Minimum limit of pitch[rad]
		SetLoggingValue(turbine_id, 15, PID_pit_var.outmax);	// 20 : Maximum limit of pitch[rad]
		SetLoggingValue(turbine_id, 16, Qdamp_ref);				// 21 : Torque reference from DT damper[Nm]
		




		/* not qiuet sure, what to do with the other output parameter
		array2[5] = Pe_ref;					// 5 : Power reference[W]
		array2[6] = WSPfilt;				// 6 : Filtered wind speed[m / s]
		array2[7] = omegafilt;				// 7 : Filtered rotor speed[rad / s]
		array2[8] = omega_err_filt_speed;	// 8 : Filtered rotor speed error for torque[rad / s]
		array2[9] = omega_dtfilt;			// 9 : Bandpass filtered rotor speed[rad / s]
		array2[10] = PID_gen_var.outpro;	// 10 : Proportional term of torque contr.[Nm]
		array2[11] = PID_gen_var.outset;	// 11 : Integral term of torque controller[Nm]
		array2[12] = PID_gen_var.outmin;	// 12 : Minimum limit of torque[Nm]
		array2[13] = PID_gen_var.outmax;	// 13 : Maximum limit of torque[Nm]
		array2[14] = switch1;				// 14 : Torque limit switch based on pitch[-]
		array2[15] = omega_err_filt_pitch;	// 15 : Filtered rotor speed error for pitch[rad / s]
		array2[16] = e_pitch[2];			// 16 : Power error for pitch[W]
		array2[17] = PID_pit_var.outpro;	// 17 : Proportional term of pitch controller[rad]
		array2[18] = PID_pit_var.outset;	// 18 : Integral term of pitch controller[rad]
		array2[19] = PID_pit_var.outmin;	// 19 : Minimum limit of pitch[rad]
		array2[20] = PID_pit_var.outmax;	// 20 : Maximum limit of pitch[rad]
		array2[21] = Qdamp_ref;				// 21 : Torque reference from DT damper[Nm]
		*/
		
		//	return; } //- end subroutine update_regulation
		//**************************************************************************************************//


		/*
		// Add active pitch teeter coupling
		hdr[TEETERANGLE].value = GetMeasuredTeeterAngle(turbine_id)*180/PI ;    // Returns measured teeter angle in rad
		if ( UseActivePTC == 1 )  DeltaPitch = tan(Delta3_angle*PI/180)*hdr[TEETERANGLE].value;  // pitch teeter coupled aktive pitching


		// export to file
		SetLoggingValue(turbine_id, 0, rpmtarget);
		SetLoggingValue(turbine_id, 1, DeltaPitch);
		SetLoggingValue(turbine_id, 2, MeanPitch);
		*/

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
