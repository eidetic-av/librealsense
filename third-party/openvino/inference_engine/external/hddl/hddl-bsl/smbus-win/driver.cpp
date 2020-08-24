// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL

#include "internal.h"
#include "driver.h"
#include "device.h"
#include "ntstrsafe.h"

#include "controller.h"

#include "driver.tmh"

#include "wdmsec.h"

NTSTATUS
#pragma prefast(suppress:__WARNING_DRIVER_FUNCTION_TYPE, "thanks, i know this already")
DriverEntry(
    _In_ PDRIVER_OBJECT  DriverObject,
    _In_ PUNICODE_STRING RegistryPath
    )
{
    WDF_DRIVER_CONFIG driverConfig;
    WDF_OBJECT_ATTRIBUTES driverAttributes;

    WDFDRIVER fxDriver;

    NTSTATUS status;

    WPP_INIT_TRACING(DriverObject, RegistryPath);

    FuncEntry(TRACE_FLAG_WDFLOADING);

    WDF_DRIVER_CONFIG_INIT(&driverConfig, OnDeviceAdd);
    driverConfig.DriverPoolTag = SI2C_POOL_TAG;

    WDF_OBJECT_ATTRIBUTES_INIT(&driverAttributes);
    driverAttributes.EvtCleanupCallback = OnDriverCleanup;

    status = WdfDriverCreate(
        DriverObject,
        RegistryPath,
        &driverAttributes,
        &driverConfig,
        &fxDriver);

    if (!NT_SUCCESS(status))
    {
        Trace(
            TRACE_LEVEL_ERROR, 
            TRACE_FLAG_WDFLOADING,
            "Error creating WDF driver object - %!STATUS!", 
            status);

        goto exit;
    }

    Trace(
        TRACE_LEVEL_VERBOSE, 
        TRACE_FLAG_WDFLOADING,
        "Created WDFDRIVER %p",
        fxDriver);

exit:

    FuncExit(TRACE_FLAG_WDFLOADING);

    return status;
}

VOID
OnDriverCleanup(
    _In_ WDFOBJECT Object
    )
{
    UNREFERENCED_PARAMETER(Object);

    WPP_CLEANUP(NULL);
}

NTSTATUS
OnDeviceAdd(
    _In_    WDFDRIVER       FxDriver,
    _Inout_ PWDFDEVICE_INIT FxDeviceInit
    )
/*++
 
  Routine Description:

    This routine creates the device object for an Intel PCH
    SMBus controller and the device's child objects.

  Arguments:

    FxDriver - the WDF driver object handle
    FxDeviceInit - information about the PDO that we are loading on

  Return Value:

    Status

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

	PI801PCH_DEVICE pDevice;
    NTSTATUS status;
    
    UNREFERENCED_PARAMETER(FxDriver);

    //
    // Setup PNP/Power callbacks.
    //
    {
        WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

        pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
        pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
        pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
        pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;
        pnpCallbacks.EvtDeviceSelfManagedIoInit = OnSelfManagedIoInit;
        pnpCallbacks.EvtDeviceSelfManagedIoCleanup = OnSelfManagedIoCleanup;

        WdfDeviceInitSetPnpPowerEventCallbacks(FxDeviceInit, &pnpCallbacks);
    }

    //
    // Note: The SPB class extension sets a default
    //       security descriptor to allow access to
    //       user-mode drivers. This can be overridden
    //       by calling WdfDeviceInitAssignSDDLString()
    //       with the desired setting. This must be done
    //       after calling SpbDeviceInitConfig() but
    //       before WdfDeviceCreate().
    //

    //before calling WdfDeviceInitAssignSDDLString, we need assign device name to it
    //you can see it using WinObj
    DECLARE_CONST_UNICODE_STRING(strDeviceName, L"\\Device\\HDDLSMBusControllerDevice");
    status = WdfDeviceInitAssignName(FxDeviceInit, &strDeviceName);
    if (!NT_SUCCESS(status))
    {
        Trace(
            TRACE_LEVEL_ERROR,
            TRACE_FLAG_WDFLOADING,
            "Failed to InitAssignName to WDFDEVICE_INIT %p - %!STATUS!",
            FxDeviceInit,
            status);
    }

    status = WdfDeviceInitAssignSDDLString(
        FxDeviceInit,
        &SDDL_DEVOBJ_SYS_ALL_ADM_RWX_WORLD_RW_RES_R
    );
    if (!NT_SUCCESS(status))
    {
        Trace(
            TRACE_LEVEL_ERROR,
            TRACE_FLAG_WDFLOADING,
            "Failed to AssignSDDLString SDDL_DEVOBJ_SYS_ALL_ADM_RWX_WORLD_RW_RES_R to WDFDEVICE_INIT %p - %!STATUS!",
            FxDeviceInit,
            status);
    }

    //
    // Create the device.
    //
    {
        WDF_OBJECT_ATTRIBUTES deviceAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, I801PCH_DEVICE);
        WDFDEVICE fxDevice;

        status = WdfDeviceCreate(
            &FxDeviceInit, 
            &deviceAttributes,
            &fxDevice);

        if (!NT_SUCCESS(status))
        {
            Trace(
                TRACE_LEVEL_ERROR, 
                TRACE_FLAG_WDFLOADING,
                "Failed to create WDFDEVICE from WDFDEVICE_INIT %p - %!STATUS!", 
                FxDeviceInit,
                status);

            goto exit;
        }

        pDevice = GetDeviceContext(fxDevice);
        NT_ASSERT(pDevice != NULL);

        pDevice->FxDevice = fxDevice;
    }
        
    //
    // Ensure device is disable-able
    //
    {
        WDF_DEVICE_STATE deviceState;
        WDF_DEVICE_STATE_INIT(&deviceState);
        
        deviceState.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(pDevice->FxDevice, &deviceState);
    }
    
    //
    // Create the spin lock to synchronize access
    // to the controller driver.
    //
    
    WDF_OBJECT_ATTRIBUTES attributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
    attributes.ParentObject = pDevice->FxDevice;

    status = WdfSpinLockCreate(
       &attributes,
       &pDevice->Lock);
    
    if (!NT_SUCCESS(status))
    {
        Trace(
            TRACE_LEVEL_ERROR, 
            TRACE_FLAG_WDFLOADING, 
            "Failed to create device spinlock for WDFDEVICE %p - %!STATUS!", 
            pDevice->FxDevice,
            status);

        goto exit;
    }
    
    //
    // Configure idle settings to use system
    // managed idle timeout.
    //
    {    
        WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS idleSettings;
        WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS_INIT(
            &idleSettings, 
            IdleCannotWakeFromS0);

        //
        // Explicitly set initial idle timeout delay.
        //

        idleSettings.IdleTimeoutType = SystemManagedIdleTimeoutWithHint;
        idleSettings.IdleTimeout = IDLE_TIMEOUT_MONITOR_ON;

        status = WdfDeviceAssignS0IdleSettings(
            pDevice->FxDevice, 
            &idleSettings);

        if (!NT_SUCCESS(status))
        {
            Trace(
                TRACE_LEVEL_ERROR,
                TRACE_FLAG_WDFLOADING,
                "Failed to initalize S0 idle settings for WDFDEVICE %p- %!STATUS!",
                pDevice->FxDevice,
                status);
                
            goto exit;
        }
    }

    //
    // Create a symbolic link.
    //

    {
        DECLARE_UNICODE_STRING_SIZE(symbolicLinkName, 128);

        status = RtlUnicodeStringPrintf(
            &symbolicLinkName, L"%ws",
            SPBHDDL_I2C_SYMBOLIC_NAME);

        if (!NT_SUCCESS(status))
        {
            Trace(
                TRACE_LEVEL_ERROR,
                TRACE_FLAG_WDFLOADING,
                "Error creating symbolic link string for device "
                "- %!STATUS!",
                status);

            goto exit;
        }

        status = WdfDeviceCreateSymbolicLink(
            pDevice->FxDevice,
            &symbolicLinkName);

        if (!NT_SUCCESS(status))
        {
            Trace(
                TRACE_LEVEL_ERROR,
                TRACE_FLAG_WDFLOADING,
                "Error creating symbolic link for device "
                "- %!STATUS!",
                status);

            goto exit;
        }
    }

    //
    // Create queues to handle IO
    //

    {
        WDF_IO_QUEUE_CONFIG queueConfig;
        WDFQUEUE queue;

        //
        // Top-level queue
        //  WdfIoQueueDispatchSequential - IO request is sequent
        //  WdfIoQueueDispatchParallel   - IO request is parallel
        // The user can decide IO queue's mode with the above two based on design.
        //
        WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(
            &queueConfig,
            WdfIoQueueDispatchSequential);

        queueConfig.EvtIoRead = OnHddlIoRead;
        queueConfig.EvtIoWrite = OnHddlIoWrite;
        queueConfig.EvtIoDeviceControl = OnHddlIoDeviceControl;
        queueConfig.PowerManaged = WdfTrue;

        status = WdfIoQueueCreate(
            pDevice->FxDevice,
            &queueConfig,
            WDF_NO_OBJECT_ATTRIBUTES,
            &queue
        );

        if (!NT_SUCCESS(status))
        {
            Trace(
                TRACE_LEVEL_ERROR,
                TRACE_FLAG_WDFLOADING,
                "Error creating top-level IO queue - %!STATUS!",
                status);

            goto exit;
        }

    }
exit:

    FuncExit(TRACE_FLAG_WDFLOADING);

    return status;
}
