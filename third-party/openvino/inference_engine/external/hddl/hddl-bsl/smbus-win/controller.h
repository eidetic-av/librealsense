// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MS-PL

#include "internal.h"


#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

//
// Controller specific function prototypes.
//

VOID ControllerInitialize(
    _In_  PI801PCH_DEVICE   pDevice);

VOID ControllerUninitialize(
    _In_  PI801PCH_DEVICE   pDevice);

NTSTATUS hddl_write_register(_In_  PI801PCH_DEVICE   pDevice,
    _In_ UCHAR address,
    _In_ UCHAR regAddr,
    _In_ UCHAR Data);

NTSTATUS hddl_read_register(_In_  PI801PCH_DEVICE   pDevice,
    _In_ UCHAR address,
    _In_ UCHAR regAddr,
    _In_ PUCHAR pData);

#endif
