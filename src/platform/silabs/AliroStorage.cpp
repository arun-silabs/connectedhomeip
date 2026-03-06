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

#include <platform/silabs/AliroStorage.h>
#include <platform/silabs/SilabsConfig.h>

#include <lib/support/CodeUtils.h>

#include <string.h>

namespace chip {
namespace DeviceLayer {
namespace Internal {

namespace {

const char kDefaultReaderId[]           = "aliro-reader-001";
const char kDefaultReaderPublicKeyStr[] = "04a1b2c3d4e5f6071829300000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
const char kDefaultReaderGroupSubId[]  = "reader-group-sub-00";

CHIP_ERROR EnsureKeyWithDefault(SilabsConfig::Key key, const char * defaultStr)
{
    if (SilabsConfig::ConfigValueExists(key))
        return CHIP_NO_ERROR;
    return SilabsConfig::WriteConfigValueStr(key, defaultStr, strlen(defaultStr));
}

} // namespace

CHIP_ERROR AliroStorage::Init(void)
{
    ReturnErrorOnFailure(EnsureKeyWithDefault(kAliroNvm3Key_ReaderId, kDefaultReaderId));
    ReturnErrorOnFailure(EnsureKeyWithDefault(kAliroNvm3Key_ReaderPublicKeyStr, kDefaultReaderPublicKeyStr));
    ReturnErrorOnFailure(EnsureKeyWithDefault(kAliroNvm3Key_ReaderGroupSubId, kDefaultReaderGroupSubId));
    return CHIP_NO_ERROR;
}

CHIP_ERROR AliroStorage::GetReaderId(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(kAliroNvm3Key_ReaderId, buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::GetReaderPublicKeyStr(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(kAliroNvm3Key_ReaderPublicKeyStr, buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::GetReaderGroupSubId(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(kAliroNvm3Key_ReaderGroupSubId, buf, bufSize, outLen);
}

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
