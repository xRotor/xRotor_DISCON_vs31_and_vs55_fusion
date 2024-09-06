# Bladed controller code for the two- and three-bladed 20MW turbines of the projects X-Rotor -- two-bladed (/floating) wind turbines

This is the code of the discontinal Bladed controllers for the two- and three-bladed 20MW turbines of the projects "X-Rotor -- two-bladed wind turbines" and "X-Rotor -- two-bladed floating wind turbines" of the Hamburg University of Applied Sciences. It is generally based on the "Basic DTU Wind Energy controller DTU reference controller" from  Morten Hartvig Hansen and Lars Christian Henriksen documented in (Hansen, M. H., & Henriksen, L. C. (2013). Basic DTU Wind Energy controller. DTU Wind Energy. DTU Wind Energy E No. 0028, https://backend.orbit.dtu.dk/ws/portalfiles/portal/56263924/DTU_Wind_Energy_E_0028.pdf). It still possesses multiple added features and filters and algorithms, such as:
- speed exclusion zone to avoid a longer crossing of the tower eigenfrequency and the rotor's twice-per-revolution (two-bladed) or thrice-per-revolution excitation frequency (three-bladed) 
- pitch-driven tower fore-aft dampers which need a user defined proportional gain to add the nacelle fore-aft velocity signal to the pitch signal
- generator-driven tower side-side dampers which need a user defined proportional gain to add the nacelle roll velocity signal to the generator signal
- pitch-teeter and pitch-teeter-velocity coupling in extreme sitations
- linear individual pitch control (LIPC) including discrete time transfer functions, inspired py Edvin von Solingen. LIPC is a unique method of IPC for two-bladed turbines due to a singularity in the coordinate transformation matrix
- 

This version is a fusion of the project's bottom-fixed turbines' legacy controller 'vs31' and the floating turbines' legacy controller 'vs51' with less outdated functions and more input options. Yet, the control code needs to be adapted if the usage for other turbines is desired, specifically the hard-coded parameter input in the upper sections for '20MW' and '10MW'.
