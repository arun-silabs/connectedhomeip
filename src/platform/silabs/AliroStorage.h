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
 *          Aliro NVM3 storage helpers using SL_TOKEN_NVM3_REGION_ALIRO (0x89a00-0x89AFF).
 *          Optional items per Confluence "Aliro Data Items NVM3 Persistence Guide" (MATTER space);
 *          APIs marked optional there have comments here—logic can be removed if confirmed optional.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <lib/core/CHIPError.h>

namespace chip {
namespace DeviceLayer {
namespace Internal {

/**
 * Aliro NVM3 key region (SL_TOKEN_NVM3_REGION_ALIRO), carved from reserved: 0x89a00-0x89AFF.
 */
constexpr uint32_t kAliroNvm3KeyBase = 0x89a00U;

/**
 * Aliro NVM3 key identifiers (Matter-style naming). Use with ToAliroNvm3Key() to get the full key.
 */
enum class AliroNvm3Key : uint8_t
{
    kReaderId                       = 0x00,
    kReaderPublicKeyStr             = 0x01,
    kReaderGroupSubId               = 0x02,
    kReaderPrivateKey               = 0x03,
    kReaderPublicKey                = 0x04,
    kReaderCertificate              = 0x05,
    kCredentialIssuerRootPublicKey  = 0x06,
    kCredentialIssuerRootCertificate = 0x07,
};

inline constexpr uint32_t ToAliroNvm3Key(AliroNvm3Key key)
{
    return kAliroNvm3KeyBase + static_cast<uint32_t>(key);
}

// Identity & Trust (raw keys; prefer AliroNvm3Key enum + ToAliroNvm3Key where applicable)
constexpr uint32_t kAliroNvm3Key_ReaderId                   = kAliroNvm3KeyBase + 0x00;
constexpr uint32_t kAliroNvm3Key_ReaderPublicKeyStr         = kAliroNvm3KeyBase + 0x01;
constexpr uint32_t kAliroNvm3Key_ReaderGroupSubId          = kAliroNvm3KeyBase + 0x02;
constexpr uint32_t kAliroNvm3Key_ReaderPrivateKey           = kAliroNvm3KeyBase + 0x03;
constexpr uint32_t kAliroNvm3Key_ReaderPublicKey            = kAliroNvm3KeyBase + 0x04;
constexpr uint32_t kAliroNvm3Key_ReaderCertificate         = kAliroNvm3KeyBase + 0x05;
constexpr uint32_t kAliroNvm3Key_CredentialIssuerRootPubKey = kAliroNvm3KeyBase + 0x06;
constexpr uint32_t kAliroNvm3Key_CredentialIssuerRootCert   = kAliroNvm3KeyBase + 0x07;
// BLE-specific
constexpr uint32_t kAliroNvm3Key_BleAdvertisingAddress     = kAliroNvm3KeyBase + 0x08;
constexpr uint32_t kAliroNvm3Key_ReaderGroupIdentifier      = kAliroNvm3KeyBase + 0x09;
constexpr uint32_t kAliroNvm3Key_GroupResolvingKey          = kAliroNvm3KeyBase + 0x0A;
constexpr uint32_t kAliroNvm3Key_ReaderCapabilityBitmap     = kAliroNvm3KeyBase + 0x0B;
// UWB-specific
constexpr uint32_t kAliroNvm3Key_UwbStaticConfig            = kAliroNvm3KeyBase + 0x0C;
constexpr uint32_t kAliroNvm3Key_SupportedRangingRounds     = kAliroNvm3KeyBase + 0x0D;
constexpr uint32_t kAliroNvm3Key_UwbCapabilityConfig        = kAliroNvm3KeyBase + 0x0E;
// Access policy
constexpr uint32_t kAliroNvm3Key_SupportedAccessDocDetails = kAliroNvm3KeyBase + 0x0F;
// Validity iteration (spec 7.2.3): per Credential Issuer; one pair for primary issuer (index 0)
constexpr uint32_t kAliroNvm3Key_AccessIteration     = kAliroNvm3KeyBase + 0x10;
constexpr uint32_t kAliroNvm3Key_RevocationIteration = kAliroNvm3KeyBase + 0x11;
// Reader Descriptor (Table 8-16, optional)
constexpr uint32_t kAliroNvm3Key_ReaderVendorId         = kAliroNvm3KeyBase + 0x12;
constexpr uint32_t kAliroNvm3Key_ReaderProductId       = kAliroNvm3KeyBase + 0x13;
constexpr uint32_t kAliroNvm3Key_ReaderFirmwareVersion = kAliroNvm3KeyBase + 0x14;
// Kpersistent cache (spec 8.3.1.11, 8.3.3.2.8): opaque blob, app-defined format
constexpr uint32_t kAliroNvm3Key_KpersistentCache = kAliroNvm3KeyBase + 0x15;
// DeviceRequest data element identifiers (spec 6.2, 8.4.1): list for step-up DeviceRequest
constexpr uint32_t kAliroNvm3Key_DeviceRequestDataElementIds = kAliroNvm3KeyBase + 0x16;

/**
 * Max lengths per Aliro spec v0.9.4 (excluding null for strings).
 * - Reader cert: spec 6.3.1 requires User Device to accept at least 274 bytes (compressed).
 * - reader_group_sub_identifier: 16-byte value (spec 6.2); 40 chars allows hex string + margin.
 * - AdvA: Table 11-1, 6 octets. Group Resolving Key: 128-bit (spec 11.3.1).
 * - Reader/Access Credential/Credential Issuer keys: ECC P-256 (spec 6.3, 6.4, 6.5): 32B priv, 65B uncompressed pub.
 * - Reader public key hex string: 65 bytes -> 130 hex chars; 132 with margin.
 */
constexpr size_t kAliroMaxReaderIdLen           = 64;
constexpr size_t kAliroMaxReaderPublicKeyStrLen = 132; // P-256 uncompressed hex: 1 + 32 + 32 bytes = 130 chars
constexpr size_t kAliroMaxReaderGroupSubIdLen   = 40;  // 16-byte value as hex = 32 chars
constexpr size_t kAliroMaxReaderGroupIdLen      = 64;  // practical global uniqueness (spec 6.2)
constexpr size_t kAliroBleAdvAddressLen         = 6;   // Table 11-1, AdvA
constexpr size_t kAliroGroupResolvingKeyLen      = 16;  // 128-bit, spec 11.3.1
constexpr size_t kAliroMaxCertLen               = 384; // at least 274 bytes compressed (spec 6.3.1)
constexpr size_t kAliroMaxPubKeyLen             = 65;  // ECC P-256 uncompressed (04 || x || y)
constexpr size_t kAliroMaxPrivKeyLen            = 32;  // ECC P-256
constexpr size_t kAliroMaxUwbStaticConfigLen    = 256; // implementation limit - spec does not define size
constexpr size_t kAliroMaxSupportedAccessDocLen = 512; // implementation limit for Access Document details
constexpr size_t kAliroReaderVendorIdLen        = 3;   // Table 8-16, IEEE OUI or CID
constexpr size_t kAliroMaxReaderProductIdLen   = 64;   // Table 8-16, variable
constexpr size_t kAliroMaxReaderFirmwareVersionLen = 32; // Table 8-16, variable
constexpr size_t kAliroMaxKpersistentCacheLen = 1024; // app-defined; N*(16+32) or with cred pub key
constexpr size_t kAliroMaxDeviceRequestDataElementIdsLen = 512; // list of doc/namespace/element IDs

class AliroStorage
{
public:
    /** Ensure Aliro keys exist; write defaults if missing. Call after NVM3 is initialized. */
    static CHIP_ERROR Init(void);

    // Identity & Trust
    static CHIP_ERROR StoreReaderId(const char * id, size_t len);
    static CHIP_ERROR RetrieveReaderId(char * buf, size_t bufSize, size_t & outLen);
    /** Optional (Confluence NVM3 guide): redundant with Reader public key (binary); derive hex from binary if not needed. Logic can be removed if confirmed optional. */
    static CHIP_ERROR StoreReaderPublicKeyStr(const char * str, size_t len);
    static CHIP_ERROR RetrieveReaderPublicKeyStr(char * buf, size_t bufSize, size_t & outLen);
    static CHIP_ERROR StoreReaderPrivateKey(const uint8_t * key);       // fixed 32 B (P-256)
    static CHIP_ERROR RetrieveReaderPrivateKey(uint8_t * buf);        // fixed 32 B
    static CHIP_ERROR StoreReaderPublicKey(const uint8_t * key);       // fixed 65 B (P-256 uncompressed)
    static CHIP_ERROR RetrieveReaderPublicKey(uint8_t * buf);        // fixed 65 B
    /** Optional (Confluence NVM3 guide): only if reader cert validation / PKI is used. Logic can be removed if confirmed optional. */
    static CHIP_ERROR StoreReaderCertificate(const uint8_t * cert, size_t len);
    static CHIP_ERROR RetrieveReaderCertificate(uint8_t * buf, size_t bufSize, size_t & outLen);
    static CHIP_ERROR StoreReaderGroupSubId(const char * id, size_t len);
    static CHIP_ERROR RetrieveReaderGroupSubId(char * buf, size_t bufSize, size_t & outLen);
    static CHIP_ERROR StoreCredentialIssuerRootPublicKey(const uint8_t * key, size_t len);
    static CHIP_ERROR RetrieveCredentialIssuerRootPublicKey(uint8_t * buf); // fixed 65 B (P-256)
    static CHIP_ERROR StoreCredentialIssuerRootCertificate(const uint8_t * cert, size_t len);
    static CHIP_ERROR RetrieveCredentialIssuerRootCertificate(uint8_t * buf, size_t bufSize, size_t & outLen);

    // BLE-specific
    static CHIP_ERROR StoreBleAdvertisingAddress(const uint8_t * addr); // fixed 6 B (AdvA)
    static CHIP_ERROR RetrieveBleAdvertisingAddress(uint8_t * addr); // fixed 6 B
    static CHIP_ERROR StoreReaderGroupIdentifier(const char * id, size_t len);
    static CHIP_ERROR RetrieveReaderGroupIdentifier(char * buf, size_t bufSize, size_t & outLen);
    /** Optional (Confluence NVM3 guide): only if BLE path uses GRK. Logic can be removed if confirmed optional. */
    static CHIP_ERROR StoreGroupResolvingKey(const uint8_t * key);     // fixed 16 B (128-bit)
    static CHIP_ERROR RetrieveGroupResolvingKey(uint8_t * buf);       // fixed 16 B
    static CHIP_ERROR StoreReaderCapabilityBitmap(uint32_t value);
    static CHIP_ERROR RetrieveReaderCapabilityBitmap(uint32_t & out);

    /** Optional (Confluence NVM3 guide): UWB-specific; omit for non-UWB builds. Logic can be removed if confirmed optional. */
    // UWB-specific
    static CHIP_ERROR StoreUwbStaticConfig(const uint8_t * config, size_t len);
    static CHIP_ERROR RetrieveUwbStaticConfig(uint8_t * buf, size_t bufSize, size_t & outLen);
    static CHIP_ERROR StoreSupportedRangingRounds(uint8_t value);
    static CHIP_ERROR RetrieveSupportedRangingRounds(uint8_t & out);
    static CHIP_ERROR StoreUwbCapabilityConfig(uint32_t value);
    static CHIP_ERROR RetrieveUwbCapabilityConfig(uint32_t & out);

    /** Optional (Confluence NVM3 guide): omit if access policy is fixed or empty. Logic can be removed if confirmed optional. */
    // Access policy
    static CHIP_ERROR StoreSupportedAccessDocumentDetails(const uint8_t * data, size_t len);
    static CHIP_ERROR RetrieveSupportedAccessDocumentDetails(uint8_t * buf, size_t bufSize, size_t & outLen);

    // Validity iteration (spec 7.2.3): per Credential Issuer; default 0 if unknown
    static CHIP_ERROR StoreAccessIteration(uint32_t value);
    static CHIP_ERROR RetrieveAccessIteration(uint32_t & out);
    static CHIP_ERROR StoreRevocationIteration(uint32_t value);
    static CHIP_ERROR RetrieveRevocationIteration(uint32_t & out);

    /** Optional (Confluence NVM3 guide): Reader Descriptor (Table 8-16); use build/firmware constants instead. Logic can be removed if confirmed optional. */
    // Reader Descriptor (Table 8-16, optional)
    static CHIP_ERROR StoreReaderVendorId(const uint8_t * id);         // fixed 3 B (OUI/CID)
    static CHIP_ERROR RetrieveReaderVendorId(uint8_t * buf); // fixed 3 B; buf must hold ≥3 bytes
    static CHIP_ERROR StoreReaderProductId(const char * id, size_t len);
    static CHIP_ERROR RetrieveReaderProductId(char * buf, size_t bufSize, size_t & outLen);
    static CHIP_ERROR StoreReaderFirmwareVersion(const char * version, size_t len);
    static CHIP_ERROR RetrieveReaderFirmwareVersion(char * buf, size_t bufSize, size_t & outLen);

    /** Optional (Confluence NVM3 guide): Kpersistent is derived (spec 8.3.1.11); can omit and recompute. Logic can be removed if confirmed optional. */
    // Kpersistent cache (spec 8.3.1.11): opaque blob, app serializes/deserializes
    static CHIP_ERROR StoreKpersistentCache(const uint8_t * data, size_t len);
    static CHIP_ERROR RetrieveKpersistentCache(uint8_t * buf, size_t bufSize, size_t & outLen);

    // DeviceRequest data element identifiers (spec 6.2, 8.4.1): list for step-up
    static CHIP_ERROR StoreDeviceRequestDataElementIds(const uint8_t * data, size_t len);
    static CHIP_ERROR RetrieveDeviceRequestDataElementIds(uint8_t * buf, size_t bufSize, size_t & outLen);
};

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
