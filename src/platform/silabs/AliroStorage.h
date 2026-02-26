/*
 *
 *    Copyright (c) 2020-2025 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/**
 *    @file
 *          Aliro NVM3 storage helpers using SL_TOKEN_NVM3_REGION_ALIRO (0x87800).
 */

#pragma once

#include <cstddef>
#include <lib/core/CHIPError.h>

namespace chip {
namespace DeviceLayer {
namespace Internal {

/**
 * Aliro NVM3 key region 0x87800-0x87FFF. Keys are 20-bit; we use region base + sub-id.
 */
constexpr uint32_t kAliroNvm3KeyBase = 0x87800U;

constexpr uint32_t kAliroNvm3Key_ReaderId           = kAliroNvm3KeyBase + 0x00;
constexpr uint32_t kAliroNvm3Key_ReaderPublicKeyStr = kAliroNvm3KeyBase + 0x01;
constexpr uint32_t kAliroNvm3Key_ReaderGroupSubId   = kAliroNvm3KeyBase + 0x02;

/** Max length for string values (excluding null). */
constexpr size_t kAliroMaxReaderIdLen           = 64;
constexpr size_t kAliroMaxReaderPublicKeyStrLen = 128;
constexpr size_t kAliroMaxReaderGroupSubIdLen   = 40;

class AliroStorage
{
public:
    /** Ensure Aliro keys exist; write defaults if missing. Call after NVM3 is initialized. */
    static CHIP_ERROR Init(void);

    /** Read reader_id into buf; outLen is length without null. */
    static CHIP_ERROR GetReaderId(char * buf, size_t bufSize, size_t & outLen);

    /** Read reader public key string into buf; outLen is length without null. */
    static CHIP_ERROR GetReaderPublicKeyStr(char * buf, size_t bufSize, size_t & outLen);

    /** Read reader_group_sub_id into buf; outLen is length without null. */
    static CHIP_ERROR GetReaderGroupSubId(char * buf, size_t bufSize, size_t & outLen);
};

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
