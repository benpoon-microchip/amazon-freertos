
/*
 * FreeRTOS V201906.00 Major
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * @todo If you are running one of the FreeRTOS demo projects, set this
 * to the certificate that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----
"\
 * "...base64 data...
" * "-----END CERTIFICATE-----
"
 */
#define keyCLIENT_CERTIFICATE_PEM   \
"-----BEGIN CERTIFICATE-----\n"\
"MIIBqTCCAU+gAwIBAgIQcMMUKLEUJncS9fo+FeZdpzAKBggqhkjOPQQDAjA0MRQw\n"\
"EgYDVQQKDAtFeGFtcGxlIEluYzEcMBoGA1UEAwwTRXhhbXBsZSBTaWduZXIgRkZG\n"\
"RjAgFw0yMDA5MjIwNTAwMDBaGA8zMDAwMTIzMTIzNTk1OVowMzEUMBIGA1UECgwL\n"\
"RXhhbXBsZSBJbmMxGzAZBgNVBAMMEjAxMjNDMTY1OEMyNTlGNzFFRTBZMBMGByqG\n"\
"SM49AgEGCCqGSM49AwEHA0IABCcGK3C6PxDidmBwyM8R5HPCh9AlJd9xNQQuI02p\n"\
"wYgemohSkwhWBcelyWtZl1MNR9fPn9GMvSqC0NF6Zh8nK02jQjBAMB0GA1UdDgQW\n"\
"BBSQNFG6eEOXdm+blEyDcvEQPLVxqDAfBgNVHSMEGDAWgBSTo9nCBwOjTujyF+hV\n"\
"prpGZkUqODAKBggqhkjOPQQDAgNIADBFAiAuVp/viVpa0rtWM9zcEp5yqI1eG65K\n"\
"tHGwiwI+QqgS2AIhAJ653+BsTmf4XtA/S6f5Oibe1jSkZjMipf3fBcMHT10C\n"\
"-----END CERTIFICATE-----\n"\

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
 * "-----BEGIN CERTIFICATE-----
"\
 * "...base64 data...
" * "-----END CERTIFICATE-----
"
 */
#define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM  \
"-----BEGIN CERTIFICATE-----\n"\
"MIIByTCCAW6gAwIBAgIQbD0yvJw/NBGFRgRHlnVWhjAKBggqhkjOPQQDAjAwMRQw\n"\
"EgYDVQQKDAtFeGFtcGxlIEluYzEYMBYGA1UEAwwPRXhhbXBsZSBSb290IENBMB4X\n"\
"DTIwMDkyMjA0NTM1MloXDTMwMDkyMjA0NTM1MlowNDEUMBIGA1UECgwLRXhhbXBs\n"\
"ZSBJbmMxHDAaBgNVBAMME0V4YW1wbGUgU2lnbmVyIEZGRkYwWTATBgcqhkjOPQIB\n"\
"BggqhkjOPQMBBwNCAAS7UbvoUnkDxy55d3YudF7tZFls4hLahqBGCvx9dF7voHJ5\n"\
"A3eQeKx/ufhsz3RizR9R6f4cEpBnqXvZNVtgKY8Jo2YwZDASBgNVHRMBAf8ECDAG\n"\
"AQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUk6PZwgcDo07o8hfoVaa6\n"\
"RmZFKjgwHwYDVR0jBBgwFoAUOaIzAJuZjm9INSQpWGak/QD7rrMwCgYIKoZIzj0E\n"\
"AwIDSQAwRgIhAObugQft1yM0uWXIZhQhgR+Xt30P35YRDPWFkgYdcSUaAiEAvCuA\n"\
"KRGgpVSiDEHz7YQlzD2bkhIBcYNl6ZVA4dsREj8=\n"\
"-----END CERTIFICATE-----\n"\

/*
 * @brief PEM-encoded client private key.
 *
 * @todo If you are running one of the FreeRTOS demo projects, set this
 * to the private key that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN RSA PRIVATE KEY-----
"\
 * "...base64 data...
" * "-----END RSA PRIVATE KEY-----
"
 */
#define keyCLIENT_PRIVATE_KEY_PEM                   ""

#endif /* AWS_CLIENT_CREDENTIAL_KEYS_H */

