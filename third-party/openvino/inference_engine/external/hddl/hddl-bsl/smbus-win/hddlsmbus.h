// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL


//
// Includes for hardware register definitions.
//

#ifndef _HDDLSMBUS_H_
#define _HDDLSMBUS_H_

#include "hw.h"

//
// Intel PCH SMBus controller registers.
//
typedef struct I801PCH_REGISTERS
{
    __declspec(align(1)) HWREG<UCHAR>   HostStatusReg;  // 0
	__declspec(align(1)) HWREG<UCHAR>   Reserved_0;  // 1
    __declspec(align(1)) HWREG<UCHAR>   HostControlReg; // 2
	__declspec(align(1)) HWREG<UCHAR>   HostCommandReg; // 3
	__declspec(align(1)) HWREG<UCHAR>   TransferSlaveAddrReg; // 4
	__declspec(align(1)) HWREG<UCHAR>   Data0Reg;
	__declspec(align(1)) HWREG<UCHAR>   Data1Reg;
	__declspec(align(1)) HWREG<UCHAR>   BlockDataReg;
	__declspec(align(1)) HWREG<UCHAR>   PackErrorCheckReg; // 8
	__declspec(align(1)) HWREG<UCHAR>   RecvSlaveAddrReg; // 9
	__declspec(align(1)) HWREG<UCHAR>   SlaveDataReg; // 10 Notice: This is USHORT, but it doesn't used
	__declspec(align(1)) HWREG<UCHAR>   Reserved_1; // 11
	__declspec(align(1)) HWREG<UCHAR>   AuxStatusReg; // 12
	__declspec(align(1)) HWREG<UCHAR>   AuxControlReg; // 13
	__declspec(align(1)) HWREG<UCHAR>   SmlinkPinCtrlReg; // 14
	__declspec(align(1)) HWREG<UCHAR>   SmbusPinCtrlReg; // 15
	__declspec(align(1)) HWREG<UCHAR>   SlaveStatusReg; // 16
	__declspec(align(1)) HWREG<UCHAR>   SlaveCommandReg; // 17
	__declspec(align(1)) HWREG<UCHAR>   Reserved_2; // 18
	__declspec(align(1)) HWREG<UCHAR>   Reserved_3; // 19
	__declspec(align(1)) HWREG<UCHAR>   NotifyDeviceAddrReg; // 20
	__declspec(align(1)) HWREG<UCHAR>   Reserved_4; // 21
	__declspec(align(1)) HWREG<UCHAR>   NotifyDataLowByteReg; // 22
	__declspec(align(1)) HWREG<UCHAR>   NotifyDataHighByteReg; // 23
}
I801PCH_REGISTERS, *PI801PCH_REGISTERS;

/* I801 Hosts Status register bits, offset 0 */
#define SMBHOSTSTATUS_BYTE_DONE		0x80
#define SMBHOSTSTATUS_INUSE_STS		0x40
#define SMBHOSTSTATUS_SMBALERT_STS	0x20
#define SMBHOSTSTATUS_FAILED		0x10
#define SMBHOSTSTATUS_BUS_ERR		0x08
#define SMBHOSTSTATUS_DEV_ERR		0x04
#define SMBHOSTSTATUS_INTR			0x02
#define SMBHOSTSTATUS_HOST_BUSY		0x01

#define STATUS_ERROR_FLAGS	(SMBHOSTSTATUS_FAILED | SMBHOSTSTATUS_BUS_ERR | \
				 SMBHOSTSTATUS_DEV_ERR)

#define STATUS_FLAGS		(SMBHOSTSTATUS_BYTE_DONE | SMBHOSTSTATUS_INTR | \
				 STATUS_ERROR_FLAGS)

/* I801 Host Control register bits */
#define SMBHOSTCONTROL_INTREN		0x01
#define SMBHOSTCONTROL_KILL			0x02
#define SMBHOSTCONTROL_LAST_BYTE	0x20
#define SMBHOSTCONTROL_START		0x40
#define SMBHOSTCONTROL_PEC_EN		0x80	/* ICH3 and later */

/* I801 command constants */
#define I801_QUICK			0x00
#define I801_BYTE			0x04
#define I801_BYTE_DATA		0x08
#define I801_WORD_DATA		0x0C
#define I801_PROC_CALL		0x10	/* unimplemented */
#define I801_BLOCK_DATA		0x14
#define I801_I2C_BLOCK_DATA	0x18	/* ICH5 and later */

/* Auxiliary control register bits, ICH4+ only */
#define SMBAUXCTL_CRC		1
#define SMBAUXCTL_E32B		2
// TODO: Define other controller-specific values.

#define SI2C_MAX_TRANSFER_LENGTH            0x00001000

#define SI2C_STATUS_ADDRESS_NACK            0x00000000
#define SI2C_STATUS_DATA_NACK               0x00000000
#define SI2C_STATUS_GENERIC_ERROR           0x00000000

//
// Device path names
//
#define SPBHDDL_I2C_NAME L"HDDLSMBUS"

#define SPBHDDL_I2C_SYMBOLIC_NAME L"\\DosDevices\\" SPBHDDL_I2C_NAME
#define SPBHDDL_I2C_USERMODE_PATH L"\\\\.\\" SPBHDDL_I2C_NAME
#define SPBHDDL_I2C_USERMODE_PATH_SIZE sizeof(SPBHDDL_I2C_USERMODE_PATH)

//
// IO operation code
//
#define FILE_DEVICE_HDDLI2C             0x0116
#define IOCTL_HDDLI2C_REGISTER_READ              CTL_CODE(FILE_DEVICE_HDDLI2C, 0x700, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HDDLI2C_REGISTER_WRITE             CTL_CODE(FILE_DEVICE_HDDLI2C, 0x701, METHOD_BUFFERED, FILE_ANY_ACCESS)

//
// Register evaluation functions.
//

FORCEINLINE
bool
TestAnyBits(
    _In_ ULONG V1,
    _In_ ULONG V2
    )
{
    return (V1 & V2) != 0;
}

FORCEINLINE
bool
TestAllBits(
    _In_ ULONG V1,
    _In_ ULONG V2
    )
{
    return ((V1 & V2) == V2);
}

#endif
