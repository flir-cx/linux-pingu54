/*
 * FLIR Platform adaption
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/printk.h>                                                       // pr_err, printk, etc
#include "fusb30x_global.h"                                                     // Chip structure
#include "platform_helpers.h"                                                   // Implementation details
#include "../core/core.h"
#include "../core/platform.h"
#include <linux/gpio.h>
#include "../core/vdm/DisplayPort/interface_dp.h"
#include <linux/power/bq24298_charger.h>
// #include <linux/video/mxc/tc358767_lcd2dp.h>


/* Number of VCONN re-starts made by TypeC state machine */
static FSC_U8 vconn_ocp_restart_count;

/*** LCD to DisplayPort converter configuration state machine ****************/

static enum {
    DPST_INIT,
    DPST_IDLE,
    DPST_CONFIG_REQD,
    DPST_CONFIG_FAIL,
    DPST_CONFIG_OK,
    DPST_MUX_ENABLED,
    DPST_HOTPLUGD,
} platform_dp_state;

static unsigned dp_pinassign;
// static void *lcd2dp_handle;

static void platform_set_dp_usbmux_enable(FSC_BOOL enable)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();

    pr_info("DP: USBMUX %s\n", enable ? "ENABLE" : "DISABLE");

    if (chip) {
        if (chip->gpio_usbmux_amsel)
            gpio_set_value_cansleep(chip->gpio_usbmux_amsel, enable ? 1 : 0);
        if (chip->gpio_usbmux_en)
            gpio_set_value_cansleep(chip->gpio_usbmux_en, enable ? 1 : 0);
    }
}

static inline void platform_set_dp_hotplug_enable(FSC_BOOL enable)
{
/*    pr_info("DP: Hotplug %s\n", enable ? "ENABLE" : "DISABLE");
*    lcd2dp_set_hotplug(lcd2dp_handle, enable);
*/
    pr_info("DP: Hotplug temporarily DISABLED\n");
}

static inline void platform_signal_dp_hotplug_irq(void)
{
/*    pr_info("DP: Hotplug IRQ\n");
*    lcd2dp_signal_hotplug_irq(lcd2dp_handle);
*/
    pr_info("DP: Hotplug IRQ temporarily DISABLED\n");
}

static inline void platform_request_dp_config(void)
{
    DisplayPortConfig_t in;

    in.word = 0;
    in.Conf = DP_CONF_UFP_D;
    in.SigConf = DP_CONF_SIG_DP_V1P3;
    in.UfpPa = dp_pinassign;

    requestDpConfig(in);
}

static void platform_dp_statemachine_update(void)
{
    DisplayPortStatus_t peer_status;
    bool enable;

    peer_status.word = DpPpStatus.word;
    enable = peer_status.Enabled &&
             !peer_status.PowerLow &&
             (peer_status.Connection == DP_CONN_UFP_D);
    /* Ignoring peer_status.UsbConfigRequest for now... */
    /* Ignoring peer_status.ExitDpModeRequest for now... */

    switch (platform_dp_state) {

        case DPST_INIT:
            /* lcd2dp_handle = lcd2dp_get_handle("lcd2dp");
            *if (!lcd2dp_handle)
            *    break;
			*/
            platform_dp_state = DPST_IDLE;
            /*fallthrough*/

        case DPST_IDLE:
            if (enable) {
                platform_request_dp_config();
                platform_dp_state = DPST_CONFIG_REQD;
            }
            break;

        case DPST_CONFIG_REQD:
            if (!enable) {
                DpPpStatus.word = 0;
                platform_dp_state = DPST_IDLE;
            } else {
                /* Relying on platform_inform_dp_config_result() being called */
                /* to set state to either DPST_CONFIG_FAIL or DPST_CONFIG_OK. */
            }
            break;

        case DPST_CONFIG_FAIL:
            DpPpStatus.word = 0;
            platform_dp_state = DPST_IDLE;
            break;

        case DPST_CONFIG_OK:
            platform_set_dp_usbmux_enable(true);

            if (peer_status.HpdState) {
	        platform_set_dp_hotplug_enable(true);
                platform_dp_state = DPST_HOTPLUGD;
            } else {
                platform_dp_state = DPST_MUX_ENABLED;
            }
            break;

        case DPST_MUX_ENABLED:
            if (!enable) {
                platform_set_dp_usbmux_enable(false);
                DpPpStatus.word = 0;
                platform_dp_state = DPST_IDLE;
            } else if (peer_status.HpdState) {
	        platform_set_dp_hotplug_enable(true);
                platform_dp_state = DPST_HOTPLUGD;
            }
            break;

        case DPST_HOTPLUGD:
            if (!enable) {
	        platform_set_dp_hotplug_enable(false);
                platform_set_dp_usbmux_enable(false);
                DpPpStatus.word = 0;
                platform_dp_state = DPST_IDLE;
            } else if (!peer_status.HpdState) {
                platform_set_dp_hotplug_enable(false);
                platform_dp_state = DPST_CONFIG_OK;
            } else if (peer_status.IrqHpd) {
                platform_signal_dp_hotplug_irq();
            }
            break;

        default:
            BUG();
            break;
    }
}

/*******************************************************************************
* Function:        platform_dp_reset
* Input:           None
* Return:          None
* Description:     Called by driver pm resume code to reset state machine.
******************************************************************************/
void platform_dp_reset(void)
{
    platform_dp_state = DPST_INIT;
    dp_pinassign = 0;
}

/*******************************************************************************
* Function:        platform_evaluate_dp_mode
* Input:           Disc Mode Ack VDO (FUSB driver typedef is outdated)
* Return:          TRUE if VDO contains a supported configuration.
* Description:     Called by driver VDM code to evaluate received DP modes.
******************************************************************************/
FSC_BOOL platform_evaluate_dp_mode(FSC_U32 mode)
{
    FSC_BOOL is_receptacle;
    FSC_U8 pin_assignment;

    if (!(mode & 0x1))
        return FALSE;  // Not UFP_D capable
    if (!(mode & 0x4))
        return FALSE;  // Not DP v1.3 compatible

    is_receptacle = (mode & 0x40) ? TRUE : FALSE;
    pin_assignment = (FSC_U8)(mode >> (is_receptacle ? 16 : 8));

    if (pin_assignment & 0x04) {   // "C" supported
        dp_pinassign = DP_UFPPA_C;
        return TRUE;
    }

    if (pin_assignment & 0x08) {   // "D" supported
        dp_pinassign = DP_UFPPA_D;
        return TRUE;
    }

    dp_pinassign = 0;   // Nothing we can use
    return FALSE;
}

/*******************************************************************************
* Function:        platform_inform_dp_status
* Input:           peer_status - DisplayPort peer device status.
*                                See DisplayPortStatus_t for details.
* Return:          None
* Description:     Called by driver DP code when peer status changes.
******************************************************************************/
void platform_inform_dp_status(FSC_U32 peer_status)
{
    /* UGLY FIX!
     * Some USB-C to HDMI dongles does not respond with correct status but
     * may instead send zero or just connection. This fix forces the
     * "DisplayPort function enabled" flag on making at least the tested
     * dongle work.
     * Uncertain what effect this may have on a compliant dongle...
     */
    if (DpPpStatus.Connection == DP_CONN_UFP_D)
        DpPpStatus.Enabled = 1;

    platform_dp_statemachine_update();
}

/*******************************************************************************
* Function:        platform_inform_dp_config_result
* Input:           Boolean - true = success/ack, false = fail/nack.
* Return:          None
* Description:     Called by driver DP code upon reception of DisplayPort
*                  Configure Ack/Nack message.
******************************************************************************/
void platform_inform_dp_config_result(FSC_BOOL success)
{
    if (platform_dp_state == DPST_CONFIG_REQD) {
        platform_dp_state = success ? DPST_CONFIG_OK : DPST_CONFIG_FAIL;
        platform_dp_statemachine_update();
    }
}

/*******************************************************************************
* Function:        platform_restart_vconn_allowed
* Input:           None
* Return:          TRUE if it's OK to re-enable VCONN.
* Description:     Called by TypeC state machine when the VCONN over-current-
*                  protection is triggered.
******************************************************************************/
FSC_BOOL platform_restart_vconn_allowed(void)
{
    if (vconn_ocp_restart_count < 3) {
        ++vconn_ocp_restart_count;
        return TRUE;
    }

    return FALSE;
}

/*******************************************************************************
* Function:        platform_unattached
* Input:           None
* Return:          None
* Description:     Inform platform that USB-C cable has been unattached.
******************************************************************************/
void platform_unattached(void)
{
    fusb_GPIO_Set_OTG(FALSE);
    vconn_ocp_restart_count = 0;
}

/*******************************************************************************
* Function:        platform_vdm_reset
* Input:           None
* Return:          None
* Description:     Inform platform that VDM modes are reset.
******************************************************************************/
void platform_vdm_reset(void)
{
    pr_info("PD: VDM reset\n");
    DpPpStatus.word = 0;
    platform_dp_statemachine_update();
}

/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i;

    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Enable/Disable the 5V Source
        fusb_GPIO_Set_VBus5v(blnEnable == TRUE ? true : false);
        break;
    case VBUS_LVL_12V:
        // Enable/Disable the 12V Source
        fusb_GPIO_Set_VBusOther(blnEnable == TRUE ? true : false);
        break;
    default:
        // Otherwise, do nothing.
        break;
    }

    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return fusb_GPIO_Get_VBus5v() ? TRUE : FALSE;

    case VBUS_LVL_12V:
        // Return the state of the 12V VBUS Source.
        return fusb_GPIO_Get_VBusOther() ? TRUE : FALSE;

    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

/*******************************************************************************
* Function:        platform_update_current_limit
* Input:           None
* Return:          None
* Description:     Set a new input current limit based on attachment state
*                  and negotiation result.
******************************************************************************/
void platform_update_current_limit(void)
{
    ConnectionState conn_state = core_getConnstate();
    static FSC_U16 saved_current = 0;
    FSC_U16 adv_current;

    if (conn_state == AttachedSink) {
        adv_current = core_get_advertised_current();
        if (saved_current != adv_current) {
            saved_current = adv_current;
            pr_debug("PD: Adv. current = %u mA\n", adv_current);
            bq24298_set_iinlim(adv_current);
        }
    } else if (conn_state == Unattached) {
        if (saved_current != -1) {
            saved_current = -1;
            bq24298_set_iinlim(-1);
        }
    }
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_update_conn_state
* Input:           None
* Return:          None
* Description:     Inform platform about changes in connection state.
******************************************************************************/
void platform_update_conn_state(void)
{
    void fusb_update_conn_state(ConnectionState conn_state);

    ConnectionState conn_state = core_getConnstate();
    static ConnectionState saved_state = Disabled;

    if (saved_state != conn_state) {
        saved_state = conn_state;
#if 0  // ConnState debugging
        printk("PD: ConnState = %u\n", conn_state);
#endif
        fusb_update_conn_state(conn_state);
    }
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    if (Data == NULL)
    {
        pr_err("%s - Error: Write data buffer is NULL!\n", __func__);
        ret = TRUE;
    }
    else if (fusb_I2C_WriteData((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = FALSE;
    }
    else  // I2C Write failure
    {
        ret = TRUE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    FSC_S32 i = 0;
    FSC_U8 temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        pr_err("FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return TRUE;
    }

    if (Data == NULL)
    {
        pr_err("%s - Error: Read data buffer is NULL!\n", __func__);
        ret = TRUE;
    }
    else if (DataLength > 1 && chip->use_i2c_blocks)    // Do block reads if able and necessary
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            ret = TRUE;
        }
        else
        {
            ret = FALSE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((FSC_U8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = FALSE;
            }
            else
            {
                ret = TRUE;
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{

}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_Delay10us(delayCount);
}

/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/
void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();

	// Optional: Notify platform of CC orientation
    pr_info("CC orientation: %s\n", (orientation == CC1) ? "CC1" :
                                    (orientation == CC2) ? "CC2" :
                                    (orientation == NONE) ? "None" : "Unknown");

    if (chip && chip->gpio_usbmux_pol)
        gpio_set_value_cansleep(chip->gpio_usbmux_pol, (orientation == CC2) ? 1 : 0);
}

/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
void platform_notify_pd_contract(FSC_BOOL contract)
{
    // Optional: Notify platform of PD contract
    static FSC_BOOL old_contract = FALSE;

    if (old_contract != contract) {
        old_contract = contract;
        pr_info("PD: Contract %s\n", (contract) ? "TRUE" : "FALSE");
    }
}

/*******************************************************************************
* Function:        platform_notify_unsupported_accessory
* Input:           None
* Return:          None
* Description:     A callback used by the core to report entry to the
*                  Unsupported Accessory state. The platform may implement
*                  USB Billboard.
*******************************************************************************/
void platform_notify_unsupported_accessory(void)
{
    // Optional: Implement USB Billboard
    pr_info("PD: Unsupported accessory\n");
}

/*******************************************************************************
* Function:        platform_set_data_role
* Input:           PolicyIsDFP - Current data role
* Return:          None
* Description:     A callback used by the core to report the new data role after
*                  a data role swap.
*******************************************************************************/
void platform_set_data_role(FSC_BOOL PolicyIsDFP)
{
    // Optional: Control Data Direction
    fusb_GPIO_Set_OTG(PolicyIsDFP);
}

/*******************************************************************************
* Function:        platform_notify_bist
* Input:           bistEnabled - TRUE when BIST enabled, FALSE when disabled
* Return:          None
* Description:     A callback that may be used to limit current sinking during
*                  BIST
*******************************************************************************/
void platform_notify_bist(FSC_BOOL bistEnabled)
{
    // Do something
}

void platform_set_timer(TIMER *timer, FSC_U16 timeout)
{
    timer->start_time = get_system_time();
    timer->timeout = timeout;
}

FSC_BOOL platform_check_timer(TIMER *timer)
{
    return (((FSC_U16)(get_system_time() - timer->start_time) > timer->timeout) ? TRUE: FALSE);
}

FSC_U16 platform_get_system_time()
{
    return get_system_time();
}
