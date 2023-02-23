#include <pgmspace.h>
 
#define SECRET
#define THINGNAME "Temp_1"                         //change this
 
const char WIFI_SSID[] = "Herin Dodia_4G";               //change this
const char WIFI_PASSWORD[] = "herin241175";           //change this
const char AWS_IOT_ENDPOINT[] = "a35x3d81mfdvy6-ats.iot.ap-south-1.amazonaws.com";       //change this
 
// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";
 
// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUB9ycUj2LopE7w+aGw3kKCMA8x/EwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIyMTEyMDE2MjE0
MVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMEurMLK9XHQPLxI0nte
ns8Vk0IMOkmVpp/pBZxy8lkWobYJGa40u3xsOAl7C10B8Xqluy2Eyy3ZqGRfsdun
yfTMeb2YkTh413Emy9u07Ut+FzKrI1qpBopBZLkLO5/XrqSwiCSJch5PGRR4389t
ln9G5DObt5V+FREKkcFroZ4kF15fg21ePQ8deAs8+stAn/jcxzBM6bOupAbHtXe3
jh09ohStr26l3sXiJTsyeIzMVRlgkkSkBkAPbIMEOSZxD0jqtU4ySpe1kEL5VZeE
mEDSRHf0Xel3f9UirFIDtqfMAQjjsdrqCN/DA8ymHffCyrIAWNyDrZhijs/shVKF
mycCAwEAAaNgMF4wHwYDVR0jBBgwFoAUK7Fxbgy4wruwYnwM7BrUvj8HRQAwHQYD
VR0OBBYEFKlBAz9nx1zTsboUXnp1mijiIbhSMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQDIZZA9nQbvGLZhCbq0t/rcr8Qr
eoXHoeev2RRSlixza2FsPn9Vkir6bBs8FDBs5vaLSHApQt6sovTPHuR9pohU4p3r
QLMvswsy2i95PJzLIelbmd+SyspMJ5Wrf2bENvMSKkMsmZF9P2kBlpUq4lt3r3YS
bPymC/9KHYDkHOYdMWzAfeM7yVyjHpV6JXDxbXDYWgTHJK1o3m9iX4rRGAzQYcn/
dVAKpNcZtvCpNk+Fe5Iwi7GWo/VgAXgUu//zTayRJTRxIaYNdGSewIC+7yHnZKhG
d8wo1YkUTh3hAQjKN8jHo/oxTfqIPuclEi/SyX9IInv0TjvcyygGasfAsjgp
-----END CERTIFICATE-----

)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAwS6swsr1cdA8vEjSe16ezxWTQgw6SZWmn+kFnHLyWRahtgkZ
rjS7fGw4CXsLXQHxeqW7LYTLLdmoZF+x26fJ9Mx5vZiROHjXcSbL27TtS34XMqsj
WqkGikFkuQs7n9eupLCIJIlyHk8ZFHjfz22Wf0bkM5u3lX4VEQqRwWuhniQXXl+D
bV49Dx14Czz6y0Cf+NzHMEzps66kBse1d7eOHT2iFK2vbqXexeIlOzJ4jMxVGWCS
RKQGQA9sgwQ5JnEPSOq1TjJKl7WQQvlVl4SYQNJEd/Rd6Xd/1SKsUgO2p8wBCOOx
2uoI38MDzKYd98LKsgBY3IOtmGKOz+yFUoWbJwIDAQABAoIBAQC+J7GKtLlRZMbE
F3KIWaU89mHTe7HMSPCRK5CIZWVEaFCZnO+YWPPMD8JBIssWrBkEvyo2LibJZRyq
YL/0FX9n5fZXuFwKGHkJOkUkaT3DN8Sh7W+JiCOV57S7qODyAB46okIApQKd7wIi
8rBrv9WKRz/NxRO6pTcbZSoAGHAn9momgHCPxV1dqwOvareA8pUsPKPUXFXpiNAU
J84eA7PYdxxdtGk/rEWcQz7OcoUUZwEeIPpTIGVrlgR7iTA/Coxi/kJNPTo6Neex
Zq5q/PXNHYAs+P0sjV07/WGQsoaVA2ZkdEyYXEGEoX26tRM6DBkA0s87Pf2i+qcQ
CW8eetvBAoGBAPkD7SABInZNORP27SLQ1VBYs9+EzS5CiX7dHvamMlmxnyaMsdvM
Mi9+8R36zzeRH8gW4nsqtncPrW14xSFwsholjna+M9fkYgR++RztO+KxHFToGJak
iPFvAYz4ytbZPfc/jJiD8Q6g/FqNRV2rK8JHImHl5g33aCj7owaXsVHXAoGBAMaZ
1dj5JJawNSJUXip1Uck0nA0WdkEvSn03mLhRpumnzsAmLQ36MdIFqIefLdVbaCVV
2hVWwqSkYXTOrrM6z3XZrEYCVzLuT9Dp03Fh7VaS8Zdb4k9MkqQ8StT0/xj4HJvG
lVvQvCB9Ka/DSqG247zI0rXNaULkVhEuV+1qhncxAoGAVNxPrP51/hbnArS9r4W3
jEAOXHfnbIJkVQWKDcgplIOBlyOPYTY/TAxYozzb7TXZvNh/qjWuJPkEwy/LDlBX
ga2W1USQalx8Qsf6oS7/n4t3+j92UbAUsyJ/RslqZrl18pKHmVothiy6kyldoEQ7
D2QqhW5RbiUILB9FOXPsi9UCgYEAnC0DZwHdjQZ6ckmoV3k03KYbhYtBaVdYG4op
9eB4AsysoJwk2BYFMaSzeBtsPpkIGDwgKKG2pSSDYwjZhyZbWNDZUaCrk+imKaKH
XWVl76vUgnOGT1fnBXJIYplbNfZ/AN00PSSznFTxAD/yI6OgCahfUwp6mVjYeeBY
0uHvZjECgYAjdyQAKmZxxbQUecMWkZ/uZkVgRadt6YnyYYQnVlicjuCAbINptpFa
8DReJJQM5bRq3RrTpKPW0QYgUBqybaPuOXDK/MzHRHCqitSQgrmADPPK+WdU1PrK
YGYjdTN1oaIPJBuu+Ikytj3veqMBmr+UuXLQnx/9CyPKV5NMtalF3g==
-----END RSA PRIVATE KEY-----

)KEY";