/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include "config.h"
#include "crtp.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"

#include "microxrceddsapp.h"

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

uxrSession session;
uxrSerialTransport transport;
uxrSerialPlatform serial_platform;

static bool Point32_serialize_topic(ucdrBuffer* writer, const Point32* topic);
static bool Point32_deserialize_topic(ucdrBuffer * reader, Point32* topic);
static uint32_t Point32_size_of_topic(const Point32* topic, uint32_t size);

static int count = 0;

void on_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id;

    Point32 topic;
    Point32_deserialize_topic(ub, &topic);

    DEBUG_PRINT("Received %d topic: %f %f %f\n", ++count, (double) topic.x, (double) topic.y, (double) topic.z);
}

void appMain(){

    // Wait for Crazyflie radio connection
    vTaskDelay(2000);
    int radio_connected = logGetVarId("radio", "isConnected");
    while(!logGetUint(radio_connected)) vTaskDelay(100);
    DEBUG_PRINT("Radio connected\n");

    // Micro-XRCE-DDS init transport and session
    if(!uxr_init_serial_transport(&transport, &serial_platform, 0, 0, 1)){
        DEBUG_PRINT("Error: Init serial transport fail\n");
        vTaskSuspend( NULL );
    }

    uxr_init_session(&session, &transport.comm, 0xBA5EBA11);
    uxr_set_topic_callback(&session, on_topic, NULL);

    if(!uxr_create_session(&session))
    {
        DEBUG_PRINT("Error: Create session fail\n");
        vTaskSuspend( NULL );
    }

    // Create Micro-XRCE-DDS buffers
    uint8_t out_stream_buff[BUFFER_SIZE];
    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session,
                                                                    out_stream_buff,
                                                                    BUFFER_SIZE,
                                                                    STREAM_HISTORY);

    uint8_t in_stream_buff[BUFFER_SIZE];
    uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, 
                                                                in_stream_buff, 
                                                                BUFFER_SIZE,
                                                                STREAM_HISTORY);

    // Create Micro-XRCE-DDS participant
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                        "<participant>"
                                            "<rtps>"
                                                "<name>crazyflie_node</name>"
                                            "</rtps>"
                                        "</participant>"
                                    "</dds>";
                                    
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session,
                                                                    reliable_out,
                                                                    participant_id,
                                                                    0,
                                                                    participant_xml,
                                                                    UXR_REPLACE);

    // Create Micro-XRCE-DDS topic
    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml =     "<dds>"
                                    "<topic>"
                                        "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                    "</topic>"
                                "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out,
                                                        topic_id, participant_id,
                                                        topic_xml, UXR_REPLACE);

    // Create Micro-XRCE-DDS publisher
    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session,
                                                                reliable_out,
                                                                publisher_id,
                                                                participant_id,
                                                                publisher_xml,
                                                                UXR_REPLACE);

    // Create Micro-XRCE-DDS subscriber
    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml = "";
    uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session,
                                                                reliable_out,
                                                                subscriber_id,
                                                                participant_id,
                                                                subscriber_xml,
                                                                UXR_REPLACE);

    // Create Micro-XRCE-DDS datareader
    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    const char* datareader_xml = "<dds>"
                                     "<data_reader>"
                                         "<topic>"
                                             "<kind>NO_KEY</kind>"
                                             "<name>rt/drone/echo</name>"
                                             "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                         "</topic>"
                                     "</data_reader>"
                                 "</dds>";
    uint16_t datareader_req = uxr_buffer_create_datareader_xml(&session, 
                                                               reliable_out, 
                                                               datareader_id, 
                                                               subscriber_id, 
                                                               datareader_xml, 
                                                               UXR_REPLACE);


    // Create Micro-XRCE-DDS datawriter 1
    uxrObjectId datawriter_1_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_1_xml = "<dds>"
                                        "<data_writer>"
                                            "<topic>"
                                                "<kind>NO_KEY</kind>"
                                                "<name>rt/drone/attitude</name>"
                                                "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                            "</topic>"
                                        "</data_writer>"
                                    "</dds>";
    uint16_t datawriter_1_req = uxr_buffer_create_datawriter_xml(&session,
                                                                reliable_out,
                                                                datawriter_1_id,
                                                                publisher_id,
                                                                datawriter_1_xml,
                                                                UXR_REPLACE);

    // Create Micro-XRCE-DDS datawriter 2
    uxrObjectId datawriter_2_id = uxr_object_id(0x02, UXR_DATAWRITER_ID);
    const char* datawriter_2_xml = "<dds>"
                                    "<data_writer>"
                                        "<topic>"
                                            "<kind>NO_KEY</kind>"
                                            "<name>rt/drone/odometry</name>"
                                            "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                        "</topic>"
                                    "</data_writer>"
                                "</dds>";
    uint16_t datawriter_2_req = uxr_buffer_create_datawriter_xml(&session,
                                                                reliable_out,
                                                                datawriter_2_id,
                                                                publisher_id,
                                                                datawriter_2_xml,
                                                                UXR_REPLACE);

    // Run Micro-XRCE-DDS session until all status
    uint8_t status[7];
    uint16_t requests[7] = {    participant_req, 
                                topic_req, 
                                publisher_req, 
                                subscriber_req,
                                datareader_req,
                                datawriter_1_req, 
                                datawriter_2_req};

    if(!uxr_run_session_until_all_status(&session, 1000, requests, status, 7))
    {
        vTaskSuspend( NULL );
    }

    // Crazyflie loggers
    pitchid = logGetVarId("stateEstimate", "pitch");
    rollid = logGetVarId("stateEstimate", "roll");
    yawid = logGetVarId("stateEstimate", "yaw");

    Xid = logGetVarId("stateEstimate", "x");
    Yid = logGetVarId("stateEstimate", "y");
    Zid = logGetVarId("stateEstimate", "z");

    // Main loop
    while(1){
        float pitch = logGetFloat(pitchid);
        float roll  = logGetFloat(rollid);
        float yaw   = logGetFloat(yawid);
        float x     = logGetFloat(Xid);
        float y     = logGetFloat(Yid);
        float z     = logGetFloat(Zid);


        // Creating messages
        Point32 attitude = {pitch, roll, yaw};
        Point32 odometry = {x, y, z};

        ucdrBuffer ub;

        // Publish topics
        uint32_t topic_size = Point32_size_of_topic(&attitude, 0);
        uxr_prepare_output_stream(&session, reliable_out, datawriter_1_id, &ub, topic_size);
        Point32_serialize_topic(&ub, &attitude);

        topic_size = Point32_size_of_topic(&odometry, 0);
        uxr_prepare_output_stream(&session, reliable_out, datawriter_2_id, &ub, topic_size);
        Point32_serialize_topic(&ub, &odometry);


        // Request topic subscription
        uxr_buffer_request_data(&session, reliable_out, datareader_id, reliable_in, NULL);

        // Run session
        uxr_run_session_until_timeout(&session, 500);

        vTaskDelay(10/portTICK_RATE_MS);
    }

  uxr_delete_session(&session);
  vTaskSuspend( NULL );
}


static bool Point32_serialize_topic(ucdrBuffer* writer, const Point32* topic)
{
    (void) ucdr_serialize_float(writer, topic->x);
    (void) ucdr_serialize_float(writer, topic->y);
    (void) ucdr_serialize_float(writer, topic->z);

    return !writer->error;
}

static bool Point32_deserialize_topic(ucdrBuffer * reader, Point32* topic)
{
    bool rv = false;
    rv = ucdr_deserialize_float(reader, &topic->x);
    rv = ucdr_deserialize_float(reader, &topic->y);
    rv = ucdr_deserialize_float(reader, &topic->z);

    return rv;
}

static uint32_t Point32_size_of_topic(const Point32* topic, uint32_t size)
{
    uint32_t previousSize = size;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;
    size += ucdr_alignment(size, 4) + 4;

    return size - previousSize;
}
