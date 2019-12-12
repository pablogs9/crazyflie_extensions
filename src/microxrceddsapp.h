#define STREAM_HISTORY  2

#define MAX_TRANSPORT_MTU UXR_CONFIG_SERIAL_TRANSPORT_MTU
#define BUFFER_SIZE    MAX_TRANSPORT_MTU * STREAM_HISTORY

typedef struct Point32
{
    float x;
    float y;
    float z;
} Point32;

void appMain();