############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE systemcmds__can_loopback_test
	MAIN can_loopback_test
	COMPILE_FLAGS
	SRCS
		can_loopback_test.c
	DEPENDS
		px4_platform
	)
