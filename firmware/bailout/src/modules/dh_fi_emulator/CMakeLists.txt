############################################################################
# SITL TEST ONLY: emulates the kit's skybeta flight_interface, which streams
# /fmu/in/trajectory_setpoint at 10 Hz in every PX4 mode. Enabled only in
# boards/px4/sitl/default.px4board; build.sh checks it is not in the .px4.
############################################################################

px4_add_module(
	MODULE modules__dh_fi_emulator
	MAIN dh_fi_emulator
	SRCS
		DhFiEmulator.cpp
	DEPENDS
		px4_work_queue
	)
