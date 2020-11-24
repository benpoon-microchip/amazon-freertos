/*
 * FreeRTOS V202002.00
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
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----\n"
 */
#define keyCLIENT_CERTIFICATE_PEM \
"-----BEGIN CERTIFICATE-----\n"\
"MIIDWjCCAkKgAwIBAgIVAOcQhbWJWbdSHU2FxSPYsLAuLRGNMA0GCSqGSIb3DQEB\n"\
"CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\n"\
"IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMDEwMjEwNTQx\n"\
"NDhaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\n"\
"dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDEleUfrFSVKHGv0IKG\n"\
"LliH52GGxt2/QUZRsCdyZfpNcD92vRsnQOTgmZYBXTr6nVIH5E5/e7g1BU03XL8L\n"\
"A+yGA8tiWA0U52mo/UGwqFiQnX4AwtDILwYs6bvhHi640LXF/ZYZ1t6bDvsQBYej\n"\
"KCQjPTAyl+teSZB5an/P3tTfhPwHToBtE0V0BAVe/mW/geArnPXSsiekDZoPOs54\n"\
"P1De1bCGiKVK1Uh1Q9Y0Y+FlbVpnBLchtqltNPddBRq70xwViYMlmVZ/Y+7kegjB\n"\
"/PecqWt4/F0olfXcl9P/mVJ8Yk4Ps0SBfWkSsAu7e3wAzw37SGeUlw0GnpcVLmH+\n"\
"QMrXAgMBAAGjYDBeMB8GA1UdIwQYMBaAFMRf5QsXFVDE07FvGz+04TIoF5IdMB0G\n"\
"A1UdDgQWBBS+YvDYwGoB3spnlOjh6QCiRNJ9ozAMBgNVHRMBAf8EAjAAMA4GA1Ud\n"\
"DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAVCREvyooKhTMmCqwe9gNLK1V\n"\
"PPdRl2UmSKyRpLHvb2erDbSKnNxwuqHmK3inXpwSzx1od6vxfc51/6jrXQYLGPXm\n"\
"sraO1iffwdIDL4LXmgtgNjhjDqGTnnzHY41/1r8LtS9Yw7lQ7jGml61NR/xCzijt\n"\
"q2GL2lbqk6DAiBT2IgeL7hYZtKMD5bFDYzpJim00R6u82jyh3u1FW3mzr46BSfHU\n"\
"i81qhQXNBg8my61XtI2xROi3hMvbWwkgxm7NdaHoUhl87w8sck9GSM7bC2RpTxes\n"\
"n7FIOICm277eW8BwDEY/H0ceoEVcRw9maBZUV0ICn2gyWyKm9baUn0ClVrN+Vw==\n"\
"-----END CERTIFICATE-----"

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
#define keyCLIENT_PRIVATE_KEY_PEM \
"-----BEGIN RSA PRIVATE KEY-----\n"\
"MIIEpQIBAAKCAQEAxJXlH6xUlShxr9CChi5Yh+dhhsbdv0FGUbAncmX6TXA/dr0b\n"\
"J0Dk4JmWAV06+p1SB+ROf3u4NQVNN1y/CwPshgPLYlgNFOdpqP1BsKhYkJ1+AMLQ\n"\
"yC8GLOm74R4uuNC1xf2WGdbemw77EAWHoygkIz0wMpfrXkmQeWp/z97U34T8B06A\n"\
"bRNFdAQFXv5lv4HgK5z10rInpA2aDzrOeD9Q3tWwhoilStVIdUPWNGPhZW1aZwS3\n"\
"IbapbTT3XQUau9McFYmDJZlWf2Pu5HoIwfz3nKlrePxdKJX13JfT/5lSfGJOD7NE\n"\
"gX1pErALu3t8AM8N+0hnlJcNBp6XFS5h/kDK1wIDAQABAoIBAQCA8PiKH9SSySZY\n"\
"dWFHdZqn+YIkjsffrIbdCbe192GH7XoS/dKTmqMFFngniOkofIvTlVJPl+ypCkMt\n"\
"bu334aj/9fkvFiXuciMBrqupnaIJqON+yEl9JCrexqhu7gcjPfIcXUpYxHDaL6dJ\n"\
"SPdAcxHQ809rv+2nuITEvq/6vmPN/xtTItBvjpOd+c0Llx4eq+x9S2LVHJwxm2A4\n"\
"WQN3D3afBxC0e21RyiJIfemJ3rJDm4hn+GVLH7zg4cuhCepmnp4hW6swCbDongJH\n"\
"ezSyqX4muaelQEP3+6/jMTt4QqcQN8VY5pAYj4/XiHnGi/0az/ByFB53gXZVaqMp\n"\
"rbxrwh8RAoGBAOukXNra2IvXoahdybEAWHucIYGA/YURtu0nmi7WFbFz129XbwZv\n"\
"aYo3YL+MyR9ch1hiI4Lj8Y0BuUwT8xHMy1tzHTJ81PtzdQKz6H0Zj2Q5e41FWCBx\n"\
"8VN/nqZVGQSUow2MH74nhdAkKIcHeE5wjOGY2FbFS0mJAuP4+5N4cDslAoGBANWR\n"\
"upv4BoNFMDZnvtaD56lr+grMRYjj+mcuJ/rJakzxuwsTXu5UvHZqYoXFucXkXnxx\n"\
"GNH6xvUJ8/c2UEW/Uj0OLorJNPqjaPrOsC3fZT7FewlEF0nrzaTiDv7AjzJo6lMB\n"\
"7Vm9FFX8eZ9dkJj+oVJbMU3a6NI1DavRnock22tLAoGBAMjThceH6DIWjlHdzL0r\n"\
"XD3M7MlgU7CLTmmD7OWp00cvQRZFX/wOC9aeJ7hYljhCNZ9f1IADIKRaKb/q6Kfs\n"\
"8At6ahHTTfkXh0vMZIHpvMlgLCpURS0xrsjI5T9BG1dKDkTHfW4kZOjlSBz8gs3n\n"\
"IkpKZ2ZcMzPyryH32wDuDkh5AoGAUttw/KXjUPZ81/60CWGvdQmRPrM1nwBj27gK\n"\
"hr+x4BxApvFi+TAi1gY8Jw4pHt+U0M7IBF845URxRE3sIsMxUsK3x9x8E2lmhze9\n"\
"Hm73Rr0zGMs0BdfYNsoFa0ylwbb2gjHTNeenhhynyu5v9QuE5wC+RihR/4vZu/ld\n"\
"ea0Q+7UCgYEAp1PpB56J+1juHjDn8XgvVQIWFz3Z6GkTMQjFff6zM2uc7YnSa6n0\n"\
"d7BqQWAiCqknkfjcIYVAJK1DFc8ckR37lKXs7B7hp7J5yHEn4L800KiK/OALclYP\n"\
"XYQ+lcU+zbxB8dHQTnUNmVLrs/UFd5TYMa+Ew/+VYvzUz1A5NMvjtmU=\n"\
"-----END RSA PRIVATE KEY-----"

/*
 * @brief PEM-encoded client private key.
 *
 * @todo If you are running one of the FreeRTOS demo projects, set this
 * to the private key that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN RSA PRIVATE KEY-----\n"\
 * "...base64 data...\n"\
 * "-----END RSA PRIVATE KEY-----\n"
 */
#define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM  ""


#endif /* AWS_CLIENT_CREDENTIAL_KEYS_H */
