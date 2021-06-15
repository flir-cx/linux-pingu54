#ifdef FSC_HAVE_DP

#include "linux/printk.h"

#include "../../platform.h"
#include "interface_dp.h"

void informStatus(DisplayPortStatus_t stat) 
{
    pr_info("DP: informStatus %X\n", stat.word );
	/* TODO: 'system' should implement this */
	/* this function is called to inform the 'system' of the DP status of the port partner */
    DpPpStatus.word = stat.word;
    platform_inform_dp_status(stat.word);
}

void informConfigResult (FSC_BOOL success) 
{
    pr_info("DP: informConfigResult %d 0x%08x\n", success, DpPpRequestedConfig.word);
	/* TODO: 'system' should implement this */
	/* this function is called when a config message is either ACKd or NAKd by the other side */
    if (success == TRUE) DpPpConfig.word = DpPpRequestedConfig.word;
    platform_inform_dp_config_result(success);
}

void updateStatusData(void) 
{
    pr_info("DP: updateStatusData\n");
	/* TODO: 'system' should implement this */
	/* this function is called to get an update of our status - to be sent to the port partner */
    DpStatus.word = 0x0001;  // [0-1] = 1 DFP_D is connected
}

FSC_BOOL DpReconfigure(DisplayPortConfig_t config) 
{
    pr_info("DP: DpReconfigure\n");
    /* TODO: 'system' should implement this */
    /* called with a DisplayPort configuration to do! */
    /* return TRUE if/when successful, FALSE otherwise */
    DpConfig.word = config.word;
    /* must actually change configurations here before returning TRUE! */
    return TRUE;
}

#endif // FSC_HAVE_DP

