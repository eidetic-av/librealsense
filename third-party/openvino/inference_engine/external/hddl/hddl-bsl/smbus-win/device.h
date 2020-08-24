// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL


#ifndef _DEVICE_H_
#define _DEVICE_H_

//
// WDF event callbacks.
//

EVT_WDF_DEVICE_PREPARE_HARDWARE         OnPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE         OnReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY                 OnD0Entry;
EVT_WDF_DEVICE_D0_EXIT                  OnD0Exit;
EVT_WDF_DEVICE_SELF_MANAGED_IO_INIT     OnSelfManagedIoInit;
EVT_WDF_DEVICE_SELF_MANAGED_IO_CLEANUP  OnSelfManagedIoCleanup;

EVT_WDF_INTERRUPT_ISR                   OnInterruptIsr;
EVT_WDF_INTERRUPT_DPC                   OnInterruptDpc;

EVT_WDF_REQUEST_CANCEL                  OnCancel;

//
// Power framework event callbacks.
//

__drv_functionClass(POWER_SETTING_CALLBACK)
_IRQL_requires_same_
NTSTATUS
OnMonitorPowerSettingCallback(
    _In_                          LPCGUID SettingGuid,
    _In_reads_bytes_(ValueLength) PVOID   Value,
    _In_                          ULONG   ValueLength,
    _Inout_opt_                   PVOID   Context
   );

// HDDL callback
EVT_WDF_IO_QUEUE_IO_READ             OnHddlIoRead;
EVT_WDF_IO_QUEUE_IO_WRITE            OnHddlIoWrite;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL   OnHddlIoDeviceControl;

#endif
