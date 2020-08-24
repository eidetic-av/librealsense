// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL

/*++

Copyright (c) Microsoft Corporation.  All rights reserved.

Module Name: 

    internal.h

Abstract:

    This module contains the common internal type and function
    definitions for the SPB controller driver.

Environment:

    kernel-mode only

Revision History:

--*/

#ifndef _INTERNAL_H_
#define _INTERNAL_H_

#pragma warning(push)
#pragma warning(disable:4512)
#pragma warning(disable:4480)

#define SI2C_POOL_TAG ((ULONG) 'C2IS')

/////////////////////////////////////////////////
//
// Common includes.
//
/////////////////////////////////////////////////

#include <initguid.h>
#include <ntddk.h>
#include <wdm.h>
#include <wdf.h>
#include <ntstrsafe.h>

//#include "SPBCx.h"
#include "i2ctrace.h"


/////////////////////////////////////////////////
//
// Hardware definitions.
//
/////////////////////////////////////////////////

#include "hddlsmbus.h"

/////////////////////////////////////////////////
//
// Resource and descriptor definitions.
//
/////////////////////////////////////////////////

#include "reshub.h"

//
// I2C Serial peripheral bus descriptor
//

#include "pshpack1.h"

typedef struct _PNP_I2C_SERIAL_BUS_DESCRIPTOR {
    PNP_SERIAL_BUS_DESCRIPTOR SerialBusDescriptor;
    ULONG ConnectionSpeed;
    USHORT SlaveAddress;
    // follwed by optional Vendor Data
    // followed by PNP_IO_DESCRIPTOR_RESOURCE_NAME
} PNP_I2C_SERIAL_BUS_DESCRIPTOR, *PPNP_I2C_SERIAL_BUS_DESCRIPTOR;

#include "poppack.h"

#define I2C_SERIAL_BUS_TYPE 0x01
#define I2C_SERIAL_BUS_SPECIFIC_FLAG_10BIT_ADDRESS 0x0001

/////////////////////////////////////////////////
//
// Settings.
//
/////////////////////////////////////////////////

//
// Power settings.
//

#define MONITOR_POWER_ON         1
#define MONITOR_POWER_OFF        0

#define IDLE_TIMEOUT_MONITOR_ON  2000
#define IDLE_TIMEOUT_MONITOR_OFF 50

//
// Target settings.
//

typedef enum ADDRESS_MODE
{
    AddressMode7Bit,
    AddressMode10Bit
}
ADDRESS_MODE, *PADDRESS_MODE;

typedef struct I801PCH_TARGET_SETTINGS
{
    // TODO: Update this structure to include other
    //       target settings needed to configure the
    //       controller (i.e. connection speed, phase/
    //       polarity for SPI).

    ADDRESS_MODE                  AddressMode;
    USHORT                        Address;
    ULONG                         ConnectionSpeed;
}
I801PCH_TARGET_SETTINGS, *PI801PCH_TARGET_SETTINGS;


//
// Transfer settings. 
//

typedef enum BUS_CONDITION
{
    BusConditionFree,
    BusConditionBusy,
    BusConditionDontCare
}
BUS_CONDITION, *PBUS_CONDITION;


/////////////////////////////////////////////////
//
// Context definitions.
//
/////////////////////////////////////////////////

//typedef struct PBC_DEVICE   PBC_DEVICE,   *PPBC_DEVICE;
typedef struct I801PCH_DEVICE   I801PCH_DEVICE, *PI801PCH_DEVICE;

//
// Device context.
//

struct I801PCH_DEVICE
{
    // TODO: Update this structure with variables that 
    //       need to be stored in the device context.

    // Handle to the WDF device.
    WDFDEVICE                      FxDevice;

    // Structure mapped to the controller's
    // register interface.
    PI801PCH_REGISTERS         pRegisters;
    ULONG                          RegistersCb;
    PHYSICAL_ADDRESS               pRegistersPhysicalAddress;

    // Controller driver spinlock.
    WDFSPINLOCK                    Lock;

    // Delay timer used to stall between transfers.
    WDFTIMER                       DelayTimer;

    // The power setting callback handle
    PVOID                          pMonitorPowerSettingHandle;
    //
    // Handle to the sequential HDDL queue
    //

    WDFQUEUE HddlQueue;
};

//
// Declate contexts for device, target, and request.
//

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(I801PCH_DEVICE,  GetDeviceContext);

#pragma warning(pop)

#endif // _INTERNAL_H_
