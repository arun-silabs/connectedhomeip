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

const char kDefaultReaderId[]            = "aliro-reader-001";
const char kDefaultReaderPublicKeyStr[]  = "04a1b2c3d4e5f6071829300000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
const char kDefaultReaderGroupSubId[]    = "reader-group-sub-00";

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

// Identity & Trust — Store/Retrieve API
CHIP_ERROR AliroStorage::StoreReaderId(const char * id, size_t len)
{
    return SilabsConfig::WriteConfigValueStr(ToAliroNvm3Key(AliroNvm3Key::kReaderId), id, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderId(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(ToAliroNvm3Key(AliroNvm3Key::kReaderId), buf, bufSize, outLen);
}

// Optional (Confluence NVM3 guide): redundant with binary Reader public key; logic can be removed if confirmed optional.
CHIP_ERROR AliroStorage::StoreReaderPublicKeyStr(const char * str, size_t len)
{
    return SilabsConfig::WriteConfigValueStr(ToAliroNvm3Key(AliroNvm3Key::kReaderPublicKeyStr), str, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderPublicKeyStr(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(ToAliroNvm3Key(AliroNvm3Key::kReaderPublicKeyStr), buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::StoreReaderPrivateKey(const uint8_t * key)
{
    return SilabsConfig::WriteConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kReaderPrivateKey), key, kAliroMaxPrivKeyLen);
}

CHIP_ERROR AliroStorage::RetrieveReaderPrivateKey(uint8_t * buf)
{
    size_t outLen = 0;
    return SilabsConfig::ReadConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kReaderPrivateKey), buf, kAliroMaxPrivKeyLen,
                                            outLen);
}

CHIP_ERROR AliroStorage::StoreReaderPublicKey(const uint8_t * key)
{
    return SilabsConfig::WriteConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kReaderPublicKey), key, kAliroMaxPubKeyLen);
}

CHIP_ERROR AliroStorage::RetrieveReaderPublicKey(uint8_t * buf)
{
    size_t outLen = 0;
    return SilabsConfig::ReadConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kReaderPublicKey), buf, kAliroMaxPubKeyLen,
                                            outLen);
}

// Optional (Confluence NVM3 guide): only if reader cert validation used; logic can be removed if confirmed optional.
CHIP_ERROR AliroStorage::StoreReaderCertificate(const uint8_t * cert, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kReaderCertificate), cert, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderCertificate(uint8_t * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kReaderCertificate), buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::StoreReaderGroupSubId(const char * id, size_t len)
{
    return SilabsConfig::WriteConfigValueStr(ToAliroNvm3Key(AliroNvm3Key::kReaderGroupSubId), id, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderGroupSubId(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(ToAliroNvm3Key(AliroNvm3Key::kReaderGroupSubId), buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::StoreCredentialIssuerRootPublicKey(const uint8_t * key, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kCredentialIssuerRootPublicKey), key, len);
}

CHIP_ERROR AliroStorage::RetrieveCredentialIssuerRootPublicKey(uint8_t * buf)
{
    size_t outLen = 0;
    return SilabsConfig::ReadConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kCredentialIssuerRootPublicKey), buf,
                                            kAliroMaxPubKeyLen, outLen);
}

CHIP_ERROR AliroStorage::StoreCredentialIssuerRootCertificate(const uint8_t * cert, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kCredentialIssuerRootCertificate), cert, len);
}

CHIP_ERROR AliroStorage::RetrieveCredentialIssuerRootCertificate(uint8_t * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueBin(ToAliroNvm3Key(AliroNvm3Key::kCredentialIssuerRootCertificate), buf,
                                            bufSize, outLen);
}

// BLE-specific
CHIP_ERROR AliroStorage::StoreBleAdvertisingAddress(const uint8_t * addr)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_BleAdvertisingAddress, addr, kAliroBleAdvAddressLen);
}

CHIP_ERROR AliroStorage::RetrieveBleAdvertisingAddress(uint8_t * addr)
{
    size_t outLen = 0;
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_BleAdvertisingAddress, addr, kAliroBleAdvAddressLen,
                                            outLen);
}

CHIP_ERROR AliroStorage::StoreReaderGroupIdentifier(const char * id, size_t len)
{
    return SilabsConfig::WriteConfigValueStr(kAliroNvm3Key_ReaderGroupIdentifier, id, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderGroupIdentifier(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(kAliroNvm3Key_ReaderGroupIdentifier, buf, bufSize, outLen);
}

// Optional (Confluence NVM3 guide): only if BLE path uses GRK; logic can be removed if confirmed optional.
CHIP_ERROR AliroStorage::StoreGroupResolvingKey(const uint8_t * key)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_GroupResolvingKey, key, kAliroGroupResolvingKeyLen);
}

CHIP_ERROR AliroStorage::RetrieveGroupResolvingKey(uint8_t * buf)
{
    size_t outLen = 0;
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_GroupResolvingKey, buf, kAliroGroupResolvingKeyLen, outLen);
}

CHIP_ERROR AliroStorage::StoreReaderCapabilityBitmap(uint32_t value)
{
    return SilabsConfig::WriteConfigValue(kAliroNvm3Key_ReaderCapabilityBitmap, value);
}

CHIP_ERROR AliroStorage::RetrieveReaderCapabilityBitmap(uint32_t & out)
{
    return SilabsConfig::ReadConfigValue(kAliroNvm3Key_ReaderCapabilityBitmap, out);
}

// Optional (Confluence NVM3 guide): UWB-specific; omit for non-UWB. Logic can be removed if confirmed optional.
// UWB-specific
CHIP_ERROR AliroStorage::StoreUwbStaticConfig(const uint8_t * config, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_UwbStaticConfig, config, len);
}

CHIP_ERROR AliroStorage::RetrieveUwbStaticConfig(uint8_t * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_UwbStaticConfig, buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::StoreSupportedRangingRounds(uint8_t value)
{
    return SilabsConfig::WriteConfigValue(kAliroNvm3Key_SupportedRangingRounds, static_cast<uint32_t>(value));
}

CHIP_ERROR AliroStorage::RetrieveSupportedRangingRounds(uint8_t & out)
{
    uint32_t val = 0;
    CHIP_ERROR err = SilabsConfig::ReadConfigValue(kAliroNvm3Key_SupportedRangingRounds, val);
    if (err == CHIP_NO_ERROR)
        out = static_cast<uint8_t>(val & 0xFF);
    return err;
}

CHIP_ERROR AliroStorage::StoreUwbCapabilityConfig(uint32_t value)
{
    return SilabsConfig::WriteConfigValue(kAliroNvm3Key_UwbCapabilityConfig, value);
}

CHIP_ERROR AliroStorage::RetrieveUwbCapabilityConfig(uint32_t & out)
{
    return SilabsConfig::ReadConfigValue(kAliroNvm3Key_UwbCapabilityConfig, out);
}

// Optional (Confluence NVM3 guide): omit if access policy fixed/empty; logic can be removed if confirmed optional.
// Access policy
CHIP_ERROR AliroStorage::StoreSupportedAccessDocumentDetails(const uint8_t * data, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_SupportedAccessDocDetails, data, len);
}

CHIP_ERROR AliroStorage::RetrieveSupportedAccessDocumentDetails(uint8_t * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_SupportedAccessDocDetails, buf, bufSize, outLen);
}

// Validity iteration (spec 7.2.3)
CHIP_ERROR AliroStorage::StoreAccessIteration(uint32_t value)
{
    return SilabsConfig::WriteConfigValue(kAliroNvm3Key_AccessIteration, value);
}

CHIP_ERROR AliroStorage::RetrieveAccessIteration(uint32_t & out)
{
    return SilabsConfig::ReadConfigValue(kAliroNvm3Key_AccessIteration, out);
}

CHIP_ERROR AliroStorage::StoreRevocationIteration(uint32_t value)
{
    return SilabsConfig::WriteConfigValue(kAliroNvm3Key_RevocationIteration, value);
}

CHIP_ERROR AliroStorage::RetrieveRevocationIteration(uint32_t & out)
{
    return SilabsConfig::ReadConfigValue(kAliroNvm3Key_RevocationIteration, out);
}

// Optional (Confluence NVM3 guide): Reader Descriptor; use build constants; logic can be removed if confirmed optional.
// Reader Descriptor (Table 8-16)
CHIP_ERROR AliroStorage::StoreReaderVendorId(const uint8_t * id)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_ReaderVendorId, id, kAliroReaderVendorIdLen);
}

CHIP_ERROR AliroStorage::RetrieveReaderVendorId(uint8_t * buf)
{
    size_t outLen = 0;
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_ReaderVendorId, buf, kAliroReaderVendorIdLen, outLen);
}

CHIP_ERROR AliroStorage::StoreReaderProductId(const char * id, size_t len)
{
    return SilabsConfig::WriteConfigValueStr(kAliroNvm3Key_ReaderProductId, id, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderProductId(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(kAliroNvm3Key_ReaderProductId, buf, bufSize, outLen);
}

CHIP_ERROR AliroStorage::StoreReaderFirmwareVersion(const char * version, size_t len)
{
    return SilabsConfig::WriteConfigValueStr(kAliroNvm3Key_ReaderFirmwareVersion, version, len);
}

CHIP_ERROR AliroStorage::RetrieveReaderFirmwareVersion(char * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueStr(kAliroNvm3Key_ReaderFirmwareVersion, buf, bufSize, outLen);
}

// Optional (Confluence NVM3 guide): Kpersistent is derived; can omit and recompute; logic can be removed if confirmed optional.
// Kpersistent cache
CHIP_ERROR AliroStorage::StoreKpersistentCache(const uint8_t * data, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_KpersistentCache, data, len);
}

CHIP_ERROR AliroStorage::RetrieveKpersistentCache(uint8_t * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_KpersistentCache, buf, bufSize, outLen);
}

// DeviceRequest data element identifiers
CHIP_ERROR AliroStorage::StoreDeviceRequestDataElementIds(const uint8_t * data, size_t len)
{
    return SilabsConfig::WriteConfigValueBin(kAliroNvm3Key_DeviceRequestDataElementIds, data, len);
}

CHIP_ERROR AliroStorage::RetrieveDeviceRequestDataElementIds(uint8_t * buf, size_t bufSize, size_t & outLen)
{
    return SilabsConfig::ReadConfigValueBin(kAliroNvm3Key_DeviceRequestDataElementIds, buf, bufSize, outLen);
}

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
