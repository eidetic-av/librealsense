// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL

#include "controller.h"
#include "device.h"

#include "controller.tmh"

VOID
ControllerInitialize(
    _In_  PI801PCH_DEVICE  pDevice
    )
/*++
 
  Routine Description:

    This routine initializes the controller hardware.

  Arguments:

    pDevice - a pointer to the PBC device context

  Return Value:

    None.

--*/
{
    FuncEntry(TRACE_FLAG_PBCLOADING);
                        
    NT_ASSERT(pDevice != NULL);

	UCHAR ctrlReg = pDevice->pRegisters->AuxControlReg.Read();
	ctrlReg &= ~(SMBAUXCTL_CRC | SMBAUXCTL_E32B);
	pDevice->pRegisters->AuxControlReg.Write(ctrlReg);

    FuncExit(TRACE_FLAG_PBCLOADING);
}

VOID
ControllerUninitialize(
    _In_  PI801PCH_DEVICE  pDevice
    )
/*++
 
  Routine Description:

    This routine uninitializes the controller hardware.

  Arguments:

    pDevice - a pointer to the PBC device context

  Return Value:

    None.

--*/
{
    FuncEntry(TRACE_FLAG_PBCLOADING);

    NT_ASSERT(pDevice != NULL);

    UNREFERENCED_PARAMETER(pDevice);

    FuncExit(TRACE_FLAG_PBCLOADING);
}


NTSTATUS i801_check_pre(_In_  PI801PCH_DEVICE   pDevice)
{
    UCHAR status;
    status = pDevice->pRegisters->HostStatusReg.Read();
    if (status & SMBHOSTSTATUS_HOST_BUSY) {
        Trace(
            TRACE_LEVEL_INFORMATION,
            TRACE_FLAG_SPBDDI, "SMBus is busy, can't use it!\n");
        return STATUS_DEVICE_BUSY;
    }

    status &= STATUS_FLAGS;
    if (status) {
        Trace(
            TRACE_LEVEL_INFORMATION,
            TRACE_FLAG_SPBDDI, "Clearing status flags (%02x)\n",
            status);

        pDevice->pRegisters->HostStatusReg.Write(status);
        status = pDevice->pRegisters->HostStatusReg.Read() & STATUS_FLAGS;
        if (status) {
            Trace(
                TRACE_LEVEL_INFORMATION,
                TRACE_FLAG_SPBDDI,
                "Failed clearing status flags (%02x)\n",
                status);
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }
    }

    return STATUS_SUCCESS;
}
VOID WaitMicroSecond(ULONG ulMircoSecond)
{

    KEVENT kEvent = { 0 };
    DbgPrint("Thread WaitMicroSecond1 %d MircoSeconds...\n", ulMircoSecond);

    KeInitializeEvent(&kEvent, SynchronizationEvent, FALSE);

    LARGE_INTEGER timeout = RtlConvertLongToLargeInteger(-10 * ulMircoSecond);

    KeWaitForSingleObject(&kEvent, Executive, KernelMode, FALSE, &timeout);

    DbgPrint("Thread is WaitMicroSecond1 runing again\n");
    return;
}
/* Wait for BUSY being cleared and either INTR or an error flag being set */
static NTSTATUS i801_wait_intr(_In_  PI801PCH_DEVICE   pDevice, UCHAR* pStatusReg)
{
    int timeout = 0;
    UCHAR status;
    // We will always wait for a fraction of a second!
    do {
        WaitMicroSecond(400);
        status = pDevice->pRegisters->HostStatusReg.Read();
    } while (((status & SMBHOSTSTATUS_HOST_BUSY) ||
        !(status & (STATUS_ERROR_FLAGS | SMBHOSTSTATUS_INTR))) &&
        (timeout++ < 400));

    if (timeout > 400) {
        Trace(
            TRACE_LEVEL_INFORMATION,
            TRACE_FLAG_SPBDDI, "INTR Timeout!\n");
        return STATUS_TIMEOUT;
    }
    Trace(
        TRACE_LEVEL_INFORMATION,
        TRACE_FLAG_SPBDDI,
        "HOST status: 0x%X",
        status);
    *pStatusReg = status & (STATUS_ERROR_FLAGS | SMBHOSTSTATUS_INTR);
    return STATUS_SUCCESS;
}

NTSTATUS hddl_read_register(_In_  PI801PCH_DEVICE   pDevice,
    _In_ UCHAR address,
    _In_ UCHAR regAddr,
    _In_ PUCHAR pData)
{
    NTSTATUS status;
    UCHAR satReg = 0;
    UCHAR ctrlReg = pDevice->pRegisters->AuxControlReg.Read();
    ctrlReg &= ~(SMBAUXCTL_CRC | SMBAUXCTL_E32B);
    pDevice->pRegisters->AuxControlReg.Write(ctrlReg);
    UCHAR addrReg = (address << 1) | 0x01; // Read 
    pDevice->pRegisters->TransferSlaveAddrReg.Write(addrReg);
    pDevice->pRegisters->HostCommandReg.Write(regAddr);
    UCHAR auxReg = pDevice->pRegisters->AuxControlReg.Read();
    auxReg &= (~0x01);
    pDevice->pRegisters->AuxControlReg.Write(auxReg);
    status = i801_check_pre(pDevice);
    if (STATUS_SUCCESS != status)
    {
        return STATUS_DEVICE_BUSY;
    }
    pDevice->pRegisters->HostControlReg.Write(I801_BYTE_DATA | SMBHOSTCONTROL_START | SMBHOSTCONTROL_LAST_BYTE);
    status = i801_wait_intr(pDevice, &satReg);
    if (status == STATUS_SUCCESS)
    {
        if (satReg & SMBHOSTSTATUS_DEV_ERR)
        {
            status = STATUS_DEVICE_DOES_NOT_EXIST;
        }
        pDevice->pRegisters->HostStatusReg.Write(satReg);
    }
    *pData = pDevice->pRegisters->Data0Reg.Read();
    return status;
}

NTSTATUS hddl_write_register(_In_  PI801PCH_DEVICE   pDevice,
    _In_ UCHAR address,
    _In_ UCHAR regAddr,
    _In_ UCHAR Data)
{
    //UCHAR ctrlReg = pDevice->pRegisters->AuxControlReg.Read();
    //ctrlReg &= ~(SMBAUXCTL_CRC | SMBAUXCTL_E32B);
    //pDevice->pRegisters->AuxControlReg.Write(ctrlReg);
    UCHAR addrReg = (address << 1); // Write & (~0x01)
    NTSTATUS status;
    // SPD reserved address
    if (address >= 0x50 && address <= 0x57)
    {
        return STATUS_ACCESS_DENIED;
    }
    pDevice->pRegisters->TransferSlaveAddrReg.Write(addrReg);
    pDevice->pRegisters->HostCommandReg.Write(regAddr);
    pDevice->pRegisters->Data0Reg.Write(Data);
    UCHAR auxReg = pDevice->pRegisters->AuxControlReg.Read();
    UCHAR staReg = 0;
    auxReg &= (~0x01);
    pDevice->pRegisters->AuxControlReg.Write(auxReg);
    status = i801_check_pre(pDevice);
    if (STATUS_SUCCESS != status)
    {
        return status;
    }

    pDevice->pRegisters->HostControlReg.Write(I801_BYTE_DATA | SMBHOSTCONTROL_START);
    status = i801_wait_intr(pDevice, &staReg);
    if (status == STATUS_SUCCESS)
    {
        pDevice->pRegisters->HostStatusReg.Write(staReg);
    }
    return status;
}
