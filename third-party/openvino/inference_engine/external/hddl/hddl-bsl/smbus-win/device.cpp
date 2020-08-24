// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL

#include "internal.h"
#include "device.h"
#include "controller.h"

#include "device.tmh"


/////////////////////////////////////////////////
//
// WDF and Intel PCH SMBus DDI callbacks.
//
/////////////////////////////////////////////////


NTSTATUS
OnPrepareHardware(
    _In_  WDFDEVICE    FxDevice,
    _In_  WDFCMRESLIST FxResourcesRaw,
    _In_  WDFCMRESLIST FxResourcesTranslated
    )
/*++
 
  Routine Description:

    This routine maps the hardware resources to the SPB
    controller register structure.

  Arguments:

    FxDevice - a handle to the framework device object
    FxResourcesRaw - list of translated hardware resources that 
        the PnP manager has assigned to the device
    FxResourcesTranslated - list of raw hardware resources that 
        the PnP manager has assigned to the device

  Return Value:

    Status

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

	PI801PCH_DEVICE pDevice = GetDeviceContext(FxDevice);
    NT_ASSERT(pDevice != NULL);
    
    NTSTATUS status = STATUS_SUCCESS; 

    UNREFERENCED_PARAMETER(FxResourcesRaw);
    
    //
    // Get the register base for the I2C controller.
    //

    {
        ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

        for(ULONG i = 0; i < resourceCount; i++)
        {
            PCM_PARTIAL_RESOURCE_DESCRIPTOR res;
           
            res = WdfCmResourceListGetDescriptor(FxResourcesTranslated, i);

            if (res->Type == CmResourceTypeMemory)
            {
                PCM_PARTIAL_RESOURCE_DESCRIPTOR rawRes = WdfCmResourceListGetDescriptor(FxResourcesRaw, i);
                Trace(
                    TRACE_LEVEL_INFORMATION,
                    TRACE_FLAG_WDFLOADING,
                    "RAW Res (PA:%I64x, length:%d) "
                    "for WDFDEVICE %p - %!STATUS!",
                    rawRes->u.Memory.Start.QuadPart,
                    rawRes->u.Memory.Length,
                    pDevice->FxDevice,
                    status);
                pDevice->pRegisters = 
                    (PI801PCH_REGISTERS) MmMapIoSpaceEx(
                        res->u.Memory.Start,
                        res->u.Memory.Length,
                        PAGE_NOCACHE | PAGE_READWRITE);

                pDevice->RegistersCb = res->u.Memory.Length;

                if (pDevice->pRegisters == NULL)
                {
                    status = STATUS_INSUFFICIENT_RESOURCES;

                    Trace(
                        TRACE_LEVEL_ERROR,
                        TRACE_FLAG_WDFLOADING,
                        "Error mapping controller registers (PA:%I64x, length:%d) "
                        "for WDFDEVICE %p - %!STATUS!", 
                        res->u.Memory.Start.QuadPart,
                        res->u.Memory.Length,
                        pDevice->FxDevice,
                        status);
                    
                    NT_ASSERT(pDevice->pRegisters != NULL);

                    goto exit;
                }

                //
                // Save the physical address to help identify
                // the underlying controller while debugging.
                //

                pDevice->pRegistersPhysicalAddress = res->u.Memory.Start;

                Trace(
                    TRACE_LEVEL_INFORMATION, 
                    TRACE_FLAG_WDFLOADING, 
                    "I2C controller @ paddr %I64x vaddr @ %p for WDFDEVICE %p", 
                    pDevice->pRegistersPhysicalAddress.QuadPart,
                    pDevice->pRegisters,
                    pDevice->FxDevice);
            }
        }
    }

exit:
    
    FuncExit(TRACE_FLAG_WDFLOADING);

    return status;
}

NTSTATUS
OnReleaseHardware(
    _In_  WDFDEVICE    FxDevice,
    _In_  WDFCMRESLIST FxResourcesTranslated
    )
/*++
 
  Routine Description:

    This routine unmaps the SPB controller register structure.

  Arguments:

    FxDevice - a handle to the framework device object
    FxResourcesRaw - list of translated hardware resources that 
        the PnP manager has assigned to the device
    FxResourcesTranslated - list of raw hardware resources that 
        the PnP manager has assigned to the device

  Return Value:

    Status

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

	PI801PCH_DEVICE pDevice = GetDeviceContext(FxDevice);
    NT_ASSERT(pDevice != NULL);
    
    NTSTATUS status = STATUS_SUCCESS;
    
    UNREFERENCED_PARAMETER(FxResourcesTranslated);
    
    if (pDevice->pRegisters != NULL)
    {
        MmUnmapIoSpace(pDevice->pRegisters, pDevice->RegistersCb);
        
        pDevice->pRegisters = NULL;
        pDevice->RegistersCb = 0;
    }

    FuncExit(TRACE_FLAG_WDFLOADING);

    return status;
}

NTSTATUS
OnD0Entry(
    _In_  WDFDEVICE              FxDevice,
    _In_  WDF_POWER_DEVICE_STATE FxPreviousState
    )
/*++
 
  Routine Description:

    This routine allocates objects needed by the driver 
    and initializes the controller hardware.

  Arguments:

    FxDevice - a handle to the framework device object
    FxPreviousState - previous power state

  Return Value:

    Status

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);
    
	PI801PCH_DEVICE pDevice = GetDeviceContext(FxDevice);
    NT_ASSERT(pDevice != NULL);
    
    UNREFERENCED_PARAMETER(FxPreviousState);

    //
    // Initialize controller.
    //

    ControllerInitialize(pDevice);
    
    FuncExit(TRACE_FLAG_WDFLOADING);

    return STATUS_SUCCESS;
}

NTSTATUS
OnD0Exit(
    _In_  WDFDEVICE              FxDevice,
    _In_  WDF_POWER_DEVICE_STATE FxPreviousState
    )
/*++
 
  Routine Description:

    This routine destroys objects needed by the driver 
    and uninitializes the controller hardware.

  Arguments:

    FxDevice - a handle to the framework device object
    FxPreviousState - previous power state

  Return Value:

    Status

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

	PI801PCH_DEVICE pDevice = GetDeviceContext(FxDevice);
    NT_ASSERT(pDevice != NULL);
    
    NTSTATUS status = STATUS_SUCCESS;
    
    UNREFERENCED_PARAMETER(FxPreviousState);

    //
    // Uninitialize controller.
    //

    ControllerUninitialize(pDevice);


    FuncExit(TRACE_FLAG_WDFLOADING);

    return status;
}

NTSTATUS
OnSelfManagedIoInit(
    _In_  WDFDEVICE  FxDevice
    )
/*++
 
  Routine Description:

    Initializes and starts the device's self-managed I/O operations.

  Arguments:
  
    FxDevice - a handle to the framework device object

  Return Value:

    None

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

	PI801PCH_DEVICE pDevice = GetDeviceContext(FxDevice);
    NTSTATUS status;

    // 
    // Register for monitor power setting callback. This will be
    // used to dynamically set the idle timeout delay according
    // to the monitor power state.
    // 

    NT_ASSERT(pDevice->pMonitorPowerSettingHandle == NULL);
    
    status = PoRegisterPowerSettingCallback(
        WdfDeviceWdmGetDeviceObject(pDevice->FxDevice), 
        &GUID_MONITOR_POWER_ON,
        OnMonitorPowerSettingCallback, 
        (PVOID)pDevice->FxDevice, 
        &pDevice->pMonitorPowerSettingHandle);

    if (!NT_SUCCESS(status))
    {
        Trace(
            TRACE_LEVEL_ERROR,
            TRACE_FLAG_WDFLOADING,
            "Failed to register monitor power setting callback - %!STATUS!",
            status);
                
        goto exit;
    }

exit:

    FuncExit(TRACE_FLAG_WDFLOADING);

    return status;
}

VOID
OnSelfManagedIoCleanup(
    _In_  WDFDEVICE  FxDevice
    )
/*++
 
  Routine Description:

    Cleanup for the device's self-managed I/O operations.

  Arguments:
  
    FxDevice - a handle to the framework device object

  Return Value:

    None

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

	PI801PCH_DEVICE pDevice = GetDeviceContext(FxDevice);

    //
    // Unregister for monitor power setting callback.
    //

    if (pDevice->pMonitorPowerSettingHandle != NULL)
    {
        PoUnregisterPowerSettingCallback(pDevice->pMonitorPowerSettingHandle);
        pDevice->pMonitorPowerSettingHandle = NULL;
    }

    FuncExit(TRACE_FLAG_WDFLOADING);
}

__drv_functionClass(POWER_SETTING_CALLBACK)
_IRQL_requires_same_
NTSTATUS
OnMonitorPowerSettingCallback(
    _In_                          LPCGUID SettingGuid,
    _In_reads_bytes_(ValueLength) PVOID   Value,
    _In_                          ULONG   ValueLength,
    _Inout_opt_                   PVOID   Context
   )
/*++
 
  Routine Description:

    This routine updates the idle timeout delay according 
    to the current monitor power setting.

  Arguments:

    SettingGuid - the setting GUID
    Value - pointer to the new value of the power setting that changed
    ValueLength - value of type ULONG that specifies the size, in bytes, 
                  of the new power setting value 
    Context - the WDFDEVICE pointer context

  Return Value:

    Status

--*/
{
    FuncEntry(TRACE_FLAG_WDFLOADING);

    UNREFERENCED_PARAMETER(ValueLength);

    WDFDEVICE Device;
    WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS idleSettings;
    BOOLEAN isMonitorOff;
    NTSTATUS status = STATUS_SUCCESS;

    if (Context == NULL)
    {
        status = STATUS_INVALID_PARAMETER;

        Trace(
            TRACE_LEVEL_ERROR,
            TRACE_FLAG_WDFLOADING,
            "%!FUNC! parameter Context is NULL - %!STATUS!",
            status);

        goto exit;
    }

    Device = (WDFDEVICE)Context;

    //
    // We only expect GUID_MONITOR_POWER_ON notifications
    // in this callback, but let's check just to be sure.
    //

    if (IsEqualGUID(*SettingGuid, GUID_MONITOR_POWER_ON))
    {
        NT_ASSERT(Value != NULL);
        NT_ASSERT(ValueLength == sizeof(ULONG));

        //
        // Determine power setting.
        //

        isMonitorOff = ((*(PULONG)Value) == MONITOR_POWER_OFF);

        //
        // Update the idle timeout delay.
        //

        WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS_INIT(
            &idleSettings, 
            IdleCannotWakeFromS0);

        idleSettings.IdleTimeoutType = SystemManagedIdleTimeoutWithHint;

        if (isMonitorOff)
        { 
            idleSettings.IdleTimeout = IDLE_TIMEOUT_MONITOR_OFF;
        }
        else
        {
            idleSettings.IdleTimeout = IDLE_TIMEOUT_MONITOR_ON;
 
        }

        status = WdfDeviceAssignS0IdleSettings(
            Device, 
            &idleSettings);

        if (!NT_SUCCESS(status))
        {
            Trace(
                TRACE_LEVEL_ERROR,
                TRACE_FLAG_WDFLOADING,
                "Failed to assign S0 idle settings - %!STATUS!",
                status);
                
            goto exit;
        }
    }

exit:
    
    FuncExit(TRACE_FLAG_WDFLOADING);
 
    return status;
}




VOID
HddlEvtRequestCancel(
	IN WDFREQUEST Request
)
/*++
Routine Description:
Called when an I/O request is cancelled after the driver has marked
the request cancellable. This callback is automatically synchronized
with the I/O callbacks since we have chosen to use frameworks Device
level locking.
Arguments:
Request - Request being cancelled.
Return Value:
VOID
--*/
{
	//PQUEUE_CONTEXT queueContext = QueueGetContext(WdfRequestGetIoQueue(Request));

	Trace(
		TRACE_LEVEL_INFORMATION,
		TRACE_FLAG_SPBDDI, "EchoEvtRequestCancel called on Request 0x%p\n", Request);

	//
	// The following is race free by the callside or DPC side
	// synchronizing completion by calling
	// WdfRequestMarkCancelable(Queue, Request, FALSE) before
	// completion and not calling WdfRequestComplete if the
	// return status == STATUS_CANCELLED.
	//
	WdfRequestCompleteWithInformation(Request, STATUS_CANCELLED, 0L);

	//
	// This book keeping is synchronized by the common
	// Queue presentation lock
	//
	//ASSERT(queueContext->CurrentRequest == Request);
	//queueContext->CurrentRequest = NULL;

	return;
}

VOID
OnHddlIoRead(
	_In_  WDFQUEUE    FxQueue,
	_In_  WDFREQUEST  FxRequest,
	_In_  size_t      Length
)
/*++

Routine Description:

Performs read from the toaster device. This event is called when the
framework receives IRP_MJ_READ requests.

Arguments:

FxQueue -  Handle to the framework queue object that is associated with the
I/O request.
FxRequest - Handle to a framework request object.
Length - Length of the data buffer associated with the request.
By default, the queue does not dispatch zero length read & write
requests to the driver and instead to complete such requests with
status success. So we will never get a zero length request.

Return Value:

None.

--*/
{
	FuncEntry(TRACE_FLAG_SPBDDI);

	UNREFERENCED_PARAMETER(Length);

	WDFDEVICE device;
	PI801PCH_DEVICE pDevice;

	Trace(
		TRACE_LEVEL_INFORMATION,
		TRACE_FLAG_SPBDDI,
		"Read request %p received",
		FxRequest);

	device = WdfIoQueueGetDevice(FxQueue);
	pDevice = GetDeviceContext(device);

	//
	// Send the read request.
	//

	//SpbPeripheralRead(pDevice, FxRequest);
	// Set transfer information
	WdfRequestSetInformation(FxRequest, (ULONG_PTR)Length);

	// Mark the request is cancelable
	//WdfRequestMarkCancelable(FxRequest, HddlEvtRequestCancel);
	WdfRequestComplete(FxRequest, STATUS_SUCCESS);
	FuncExit(TRACE_FLAG_SPBDDI);
}

VOID
OnHddlIoWrite(
	_In_  WDFQUEUE    FxQueue,
	_In_  WDFREQUEST  FxRequest,
	_In_  size_t      Length
)
/*++

Routine Description:

Performs write to the toaster device. This event is called when the
framework receives IRP_MJ_WRITE requests.

Arguments:

Queue -  Handle to the framework queue object that is associated
with the I/O request.
Request - Handle to a framework request object.
Lenght - Length of the data buffer associated with the request.
The default property of the queue is to not dispatch
zero lenght read & write requests to the driver and
complete is with status success. So we will never get
a zero length request.

Return Value:

None
--*/
{
	FuncEntry(TRACE_FLAG_SPBDDI);

	UNREFERENCED_PARAMETER(Length);

	WDFDEVICE device;
	PI801PCH_DEVICE pDevice;

	Trace(
		TRACE_LEVEL_INFORMATION,
		TRACE_FLAG_SPBDDI,
		"Write request %p received",
		FxRequest);

	device = WdfIoQueueGetDevice(FxQueue);
	pDevice = GetDeviceContext(device);

	//
	// Send the write request.
	//

	//SpbPeripheralWrite(pDevice, FxRequest);
	WdfRequestSetInformation(FxRequest, (ULONG_PTR)Length);

	// Mark the request is cancelable
	//WdfRequestMarkCancelable(FxRequest, HddlEvtRequestCancel);
	WdfRequestComplete(FxRequest, STATUS_SUCCESS);
	FuncExit(TRACE_FLAG_SPBDDI);
}

VOID
OnHddlIoDeviceControl(
	_In_  WDFQUEUE    FxQueue,
	_In_  WDFREQUEST  FxRequest,
	_In_  size_t      OutputBufferLength,
	_In_  size_t      InputBufferLength,
	_In_  ULONG       IoControlCode
)
/*++
Routine Description:

This event is called when the framework receives IRP_MJ_DEVICE_CONTROL
requests from the system.

Arguments:

FxQueue - Handle to the framework queue object that is associated
with the I/O request.
FxRequest - Handle to a framework request object.
OutputBufferLength - length of the request's output buffer,
if an output buffer is available.
InputBufferLength - length of the request's input buffer,
if an input buffer is available.
IoControlCode - the driver-defined or system-defined I/O control code
(IOCTL) that is associated with the request.

Return Value:

VOID

--*/
{
	FuncEntry(TRACE_FLAG_SPBDDI);

	WDFDEVICE device;
	PI801PCH_DEVICE pDevice;
	NTSTATUS status = STATUS_SUCCESS;

	PVOID inputRequestBuffer = NULL;
	PVOID outputRequestBuffer = NULL;
	size_t inputBufSize;
	size_t outputBufSize;
	UCHAR address;
	UCHAR regAddr;

#define EXIT_ON(cond, st) do{if((cond)) {status = st;  goto DEVIOCTL_ERROR_EXIT;}}while(0)

	EXIT_ON(InputBufferLength < 2, STATUS_INVALID_PARAMETER);
	status = WdfRequestRetrieveInputBuffer(FxRequest,
		sizeof(UCHAR),
		&inputRequestBuffer,
		&inputBufSize);

	EXIT_ON(!NT_SUCCESS(status), status);
	ASSERT(inputBufSize == InputBufferLength);

	//check buffer size before accessing
	EXIT_ON(inputBufSize < 2, STATUS_INVALID_BUFFER_SIZE);
	address = ((UCHAR *)inputRequestBuffer)[0];
	regAddr = ((UCHAR *)inputRequestBuffer)[1];
	Trace(
		TRACE_LEVEL_INFORMATION,
		TRACE_FLAG_SPBDDI,
		"DeviceIoControl request %p received with IOCTL=%lu, status:  %!STATUS!",
		FxRequest,
		IoControlCode, status);
	Trace(
		TRACE_LEVEL_INFORMATION,
		TRACE_FLAG_SPBDDI,
		"Device address %d Reg addr: %d",
		address,
		regAddr);
	device = WdfIoQueueGetDevice(FxQueue);
	pDevice = GetDeviceContext(device);
	UCHAR regstatus = pDevice->pRegisters->HostStatusReg.Read();
	Trace(
		TRACE_LEVEL_INFORMATION,
		TRACE_FLAG_SPBDDI,
		"Reg HOST status: 0x%X",
		regstatus);

	if (0xFF == regstatus)
	{
		pDevice->pRegisters->HostStatusReg.Write(regstatus);
	}
	//
	// Translate the test IOCTL into the appropriate 
	// SMBus API method.  Open and close are completed 
	// synchronously.
	//

	switch (IoControlCode)
	{
	case IOCTL_HDDLI2C_REGISTER_READ:
		EXIT_ON(OutputBufferLength < 1, STATUS_INVALID_BUFFER_SIZE);
		status = WdfRequestRetrieveOutputBuffer(FxRequest,
			sizeof(UCHAR),
			&outputRequestBuffer,
			&outputBufSize);

		EXIT_ON(!NT_SUCCESS(status), status);
		ASSERT(outputBufSize == OutputBufferLength);
		EXIT_ON(outputBufSize < 1, STATUS_INVALID_BUFFER_SIZE);

		status = hddl_read_register(pDevice, address, regAddr, (PUCHAR)outputRequestBuffer);
		EXIT_ON(!NT_SUCCESS(status), status);

		WdfRequestSetInformation(FxRequest, 1);
		break;
	case IOCTL_HDDLI2C_REGISTER_WRITE:
		EXIT_ON(inputBufSize < 3, STATUS_INVALID_BUFFER_SIZE);
		status = hddl_write_register(pDevice, address, regAddr, ((UCHAR *)inputRequestBuffer)[2]);
		break;
	default:
		status = STATUS_INVALID_DEVICE_REQUEST;
		Trace(
			TRACE_LEVEL_WARNING,
			TRACE_FLAG_SPBDDI,
			"Request %p received with unexpected IOCTL=%lu",
			FxRequest,
			IoControlCode);
	}

DEVIOCTL_ERROR_EXIT:
	//
	// Complete the request if necessary.
	//
	{
		Trace(
			TRACE_LEVEL_INFORMATION,
			TRACE_FLAG_SPBDDI,
			"Completing request %p with %!STATUS!",
			FxRequest,
			status);

		WdfRequestComplete(FxRequest, status);
	}

	FuncExit(TRACE_FLAG_SPBDDI);
}
