
ccflags-y += -DFSC_PLATFORM_LINUX
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_TRIGGERED_INTERRUPT) += -DFSC_INTERRUPT_TRIGGERED
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_DEBUG) += -DFSC_DEBUG

ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_SRC) += -DFSC_HAVE_SRC
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_SNK) += -DFSC_HAVE_SNK
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_DRP) += -DFSC_HAVE_DRP
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_ACCMODE) += -DFSC_HAVE_ACCMODE
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_VDM) += -DFSC_HAVE_VDM
ccflags-$(CONFIG_POWER_SUPPLY_FUSB30X_DP) += -DFSC_HAVE_DP


obj-$(CONFIG_POWER_SUPPLY_FUSB30X) := fusb30x_whole.o 
fusb30x_whole-objs :=	Platform_Linux/fusb30x_driver.o \
			Platform_Linux/fusb30x_global.o \
			Platform_Linux/platform.o \
			Platform_Linux/platform_helpers.o \
			Platform_Linux/hostcomm.o \
			core/core.o \
			core/fusb30X.o \
			core/Log.o \
			core/PDPolicy.o \
			core/PDProtocol.o \
			core/TypeC.o \
			core/vdm/bitfield_translators.o \
			core/vdm/vdm.o \
			core/vdm/vdm_callbacks.o \
			core/vdm/vdm_config.o \
			core/vdm/DisplayPort/dp.o \
			core/vdm/DisplayPort/dp_system_stubs.o

