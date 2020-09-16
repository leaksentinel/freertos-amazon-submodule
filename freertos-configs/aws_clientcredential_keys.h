/*
 * Amazon FreeRTOS V201906.00 Major
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#ifndef AWS_CLIENT_CREDENTIAL_KEYS_H
#define AWS_CLIENT_CREDENTIAL_KEYS_H

/*
 * @brief PEM-encoded client certificate.
 *
 * @todo If you are running one of the Amazon FreeRTOS demo projects, set this
 * to the certificate that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----\n"
 */
#define keyCLIENT_CERTIFICATE_PEM    " "

/*
 * @brief PEM-encoded issuer certificate for AWS IoT Just In Time Registration (JITR).
 *
 * @todo If you are using AWS IoT Just in Time Registration (JITR), set this to
 * the issuer (Certificate Authority) certificate of the client certificate above.
 *
 * @note This setting is required by JITR because the issuer is used by the AWS
 * IoT gateway for routing the device's initial request. (The device client
 * certificate must always be sent as well.) For more information about JITR, see:
 *  https://docs.aws.amazon.com/iot/latest/developerguide/jit-provisioning.html,
 *  https://aws.amazon.com/blogs/iot/just-in-time-registration-of-device-certificates-on-aws-iot/.
 *
 * If you're not using JITR, set below to NULL.
 *
 * Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----\n"
 */
#define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM                         \
    "-----BEGIN CERTIFICATE-----\n"                                      \
    "MIIDeTCCAmGgAwIBAgIJAJLkbaXQuzohMA0GCSqGSIb3DQEBCwUAMFMxCzAJBgNV\n" \
    "BAYTAlVTMRMwEQYDVQQIDApDYWxpZm9ybmlhMRIwEAYDVQQHDAlQYWxvIEFsdG8x\n" \
    "GzAZBgNVBAoMEkxlYWtTZW50aW5lbCwgSW5jLjAeFw0xOTA4MDkxNDE5NTdaFw0y\n" \
    "MjA1MjkxNDE5NTdaMFMxCzAJBgNVBAYTAlVTMRMwEQYDVQQIDApDYWxpZm9ybmlh\n" \
    "MRIwEAYDVQQHDAlQYWxvIEFsdG8xGzAZBgNVBAoMEkxlYWtTZW50aW5lbCwgSW5j\n" \
    "LjCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAJznzM3oC3KRKsfiJCFM\n" \
    "ritAzI3BW2dJfv6lTIVdxi+vups465h+tIOYJpXuRGKkOFOHAv4AO/vSWgwjqSh4\n" \
    "pNjAzSZulXNzr6kI39iDpItXk9vmWi5X3nQWxdbzGnhOzQesDIYYjIfrqurFDEnX\n" \
    "uuVnRTLbWeID2qLVoMacFRLqFbKgQL6JBaLm10DXaFSl1NwabhpJxR4orhX4Ltys\n" \
    "SBHI+jIL6y4Htzp48b3ic42qbn81cKNoA525SYYF182A2wzxQj6k+jNHYH61b+Om\n" \
    "DOFAee3VrYKE2OkpUJVZqkHV6DjHirU3ghSYhf6CR5ec06/8kAlSV1x53oJ3VAWI\n" \
    "F18CAwEAAaNQME4wHQYDVR0OBBYEFKr1F67igkDHbAeKgvfmO8PR1SZSMB8GA1Ud\n" \
    "IwQYMBaAFKr1F67igkDHbAeKgvfmO8PR1SZSMAwGA1UdEwQFMAMBAf8wDQYJKoZI\n" \
    "hvcNAQELBQADggEBAFvCIZOperd7oMekTMxPBQXc30dCORAuIdJn6EKPe1waU5xC\n" \
    "y/bYyA7bU1OwLlqyrAFiC1GwqbtpGtkxFgFaCjCWt2IxStfKEwf6Cpm+20eKWE3P\n" \
    "9RPY3jOvdOHAjDlwiujIdPqcr1fGWD039prh+LpFVbWIgocbJ/zEtbWXBEhibXur\n" \
    "4c3sbd8JSx2PVGLrodMenJc1VoiVn4eTZ2aA6cxOC1ZMt0CByEQE2L6sjPiyI6k9\n" \
    "uxHyrMKOj5mGfwzDLdPRatwL2j0flIliHGh76g1rvz5l5WQ1m8D1XtJ+WOroMhA9\n" \
    "wi6hhE32nyLFp+HrOlPw3G5LOgco+HqGuO3I54c=\n"                         \
    "-----END CERTIFICATE-----\n"

/*
 * @brief PEM-encoded client private key.
 *
 * @todo If you are running one of the Amazon FreeRTOS demo projects, set this
 * to the private key that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN RSA PRIVATE KEY-----\n"\
 * "...base64 data...\n"\
 * "-----END RSA PRIVATE KEY-----\n"
 */
#define keyCLIENT_PRIVATE_KEY_PEM    " "

#endif /* AWS_CLIENT_CREDENTIAL_KEYS_H */
