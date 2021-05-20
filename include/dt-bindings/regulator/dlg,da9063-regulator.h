/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _DT_BINDINGS_REGULATOR_DLG_DA9063_H
#define _DT_BINDINGS_REGULATOR_DLG_DA9063_H

/*
 * These buck mode constants may be used to specify values in device tree
 * properties (e.g. regulator-initial-mode).
 * A description of the following modes is in the manufacturers datasheet.
 */

/* Define these only if they are no already defined. Copy definitions from
 * include/linux/mfd/da9063/registers.h
 */
#ifndef DA9063_BUCK_MODE_SLEEP
#define	DA9063_BUCK_MODE_SLEEP		0x40
#endif

#ifndef DA9063_BUCK_MODE_SYNC
#define	DA9063_BUCK_MODE_SYNC		0x80
#endif

#ifndef DA9063_BUCK_MODE_AUTO
#define	DA9063_BUCK_MODE_AUTO		0xC0
#endif

#endif
