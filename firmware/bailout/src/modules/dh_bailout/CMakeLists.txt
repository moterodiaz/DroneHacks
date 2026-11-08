############################################################################
# DroneHacks "DH Bailout" flight mode (firmware/bailout in the DroneHacks repo)
############################################################################

px4_add_module(
	MODULE modules__dh_bailout
	MAIN dh_bailout
	SRCS
		DhBailout.cpp
		DhBailout.hpp
		bailout_logic.hpp
		brake_profile.hpp
	DEPENDS
		px4_work_queue
	)
