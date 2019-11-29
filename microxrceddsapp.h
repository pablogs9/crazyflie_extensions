#define STREAM_HISTORY  2

#define MAX_TRANSPORT_MTU UXR_CONFIG_SERIAL_TRANSPORT_MTU
#define BUFFER_SIZE    MAX_TRANSPORT_MTU * STREAM_HISTORY

typedef struct Point32
{
    float roll;
    float pitch;
    float yaw;
} Point32;

typedef struct Point32_odometry
{
    float x;
    float y;
    float z;
} Point32_odo;

struct ucdrBuffer;

void appMain();