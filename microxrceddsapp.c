/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <uxr/client/client.h>
// #include <uxr/client/profile/transport/serial/serial_transport.h>
// #include <uxr/client/profile/transport/serial/serial_transport_crazyflie.h>
#include <ucdr/microcdr.h>

#include <stdio.h> //printf
#include <string.h> //strcmp
#include <stdlib.h> //atoi

#include <stdint.h>
#include <stdbool.h>

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
static uint32_t Point32_size_of_topic(const Point32* topic, uint32_t size);
static bool Point32_odo_serialize_topic(ucdrBuffer* writer,const Point32_odo* topic);
static uint32_t Point32_odo_size_of_topic(const Point32_odo* topic, uint32_t size);


void appMain(){

  //Init micro-XRCE-DDS session.
  vTaskDelay(10000);
  int radio_connected = logGetVarId("radio", "isConnected");
  while(!logGetUint(radio_connected)) vTaskDelay(100);
  DEBUG_PRINT("Radio connected\r\n");


  if(!uxr_init_serial_transport(&transport, &serial_platform, 0, 0, 1)){
    DEBUG_PRINT("Error: Init serial transport fail \r\n");
    vTaskSuspend( NULL );//Suspend this task to avoid future crash.
  }

  uxr_init_session(&session, &transport.comm, 0xAAAABCCB);

  if(!uxr_create_session(&session))
  {
    DEBUG_PRINT("Error: Create session fail\r\n");
    vTaskSuspend( NULL );
  }

  uint8_t out_stream_buff[BUFFER_SIZE];
  uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session,
                                                                out_stream_buff,
                                                                BUFFER_SIZE,
                                                                STREAM_HISTORY);

  uint8_t in_stream_buff[BUFFER_SIZE];
  uxr_create_input_reliable_stream(&session, in_stream_buff, BUFFER_SIZE,
                                    STREAM_HISTORY);

  // Create entities
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

  //Topic creation
  uxrObjectId topic_id_odo = uxr_object_id(0x01, UXR_TOPIC_ID);
  const char* topic_xml_odo = "<dds>"
                                "<topic>"
                                    "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                "</topic>"
                            "</dds>";
  uint16_t topic_req_odo = uxr_buffer_create_topic_xml(&session, reliable_out,
                                                    topic_id_odo, participant_id,
                                                    topic_xml_odo, UXR_REPLACE);

  uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session,
                                                            reliable_out,
                                                            publisher_id,
                                                            participant_id,
                                                            publisher_xml,
                                                            UXR_REPLACE);

  //Datawriter 1 creation.
  uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
  const char* datawriter_xml = "<dds>"
                                   "<data_writer>"
                                       "<topic>"
                                           "<kind>NO_KEY</kind>"
                                           "<name>rt/drone/attitude</name>"
                                           "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                       "</topic>"
                                   "</data_writer>"
                               "</dds>";
  uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session,
                                                              reliable_out,
                                                              datawriter_id,
                                                              publisher_id,
                                                              datawriter_xml,
                                                              UXR_REPLACE);

  //Datawriter 2 creation
  uxrObjectId datawriter_id_odo = uxr_object_id(0x02, UXR_DATAWRITER_ID);
  const char* datawriter_xml_odo = "<dds>"
                                   "<data_writer>"
                                       "<topic>"
                                           "<kind>NO_KEY</kind>"
                                           "<name>rt/drone/odometry</name>"
                                           "<dataType>geometry_msgs::msg::dds_::Point32_</dataType>"
                                       "</topic>"
                                   "</data_writer>"
                               "</dds>";
  uint16_t datawriter_req_odo = uxr_buffer_create_datawriter_xml(&session,
                                                              reliable_out,
                                                              datawriter_id_odo,
                                                              publisher_id,
                                                              datawriter_xml_odo,
                                                              UXR_REPLACE);

  // Send create entities message and wait its status
  uint8_t status[5];
  uint16_t requests[5] = {participant_req,topic_req_odo,publisher_req,datawriter_req_odo,datawriter_req};
  if(!uxr_run_session_until_all_status(&session, 1000, requests, status, 5))
  {
      vTaskSuspend( NULL );
  }

  //Get pitch, roll and yaw value
  pitchid = logGetVarId("stateEstimate", "pitch");
  rollid = logGetVarId("stateEstimate", "roll");
  yawid = logGetVarId("stateEstimate", "yaw");

  //Get X,Y and Z value
  Xid = logGetVarId("stateEstimate", "x");
  Yid = logGetVarId("stateEstimate", "y");
  Zid = logGetVarId("stateEstimate", "z");

  while(1){
    //Get attitude value.
    float pitch = logGetFloat(pitchid);
    float roll  = logGetFloat(rollid);
    float yaw   = logGetFloat(yawid);
    float x     = logGetFloat(Xid);
    float y     = logGetFloat(Yid);
    float z     = logGetFloat(Zid);

    Point32 cmd = {pitch,roll,yaw};
    Point32_odo cmd_odo = {x,y,z};

    ucdrBuffer ub;

    uint32_t topic_size = Point32_size_of_topic(&cmd,0);
    uxr_prepare_output_stream(&session, reliable_out, datawriter_id,
                              &ub, topic_size);
    Point32_serialize_topic(&ub,&cmd);

    uint32_t topic_size_odo = Point32_odo_size_of_topic(&cmd_odo, 0);
    uxr_prepare_output_stream(&session, reliable_out, datawriter_id_odo,
                              &ub,topic_size_odo);
    Point32_odo_serialize_topic(&ub, &cmd_odo);

    uxr_run_session_until_timeout(&session, 300);

    vTaskDelay(100/portTICK_RATE_MS);
  }

  uxr_delete_session(&session);
  vTaskSuspend( NULL );
}


static bool Point32_serialize_topic(ucdrBuffer* writer, const Point32* topic)
{
    (void) ucdr_serialize_float(writer, topic->roll);

    (void) ucdr_serialize_float(writer, topic->pitch);

    (void) ucdr_serialize_float(writer, topic->yaw);

    return !writer->error;
}

static bool Point32_odo_serialize_topic(ucdrBuffer* writer, const Point32_odo* topic)
{
    (void) ucdr_serialize_float(writer, topic->x);

    (void) ucdr_serialize_float(writer, topic->y);

    (void) ucdr_serialize_float(writer, topic->z);

    return !writer->error;
}

static uint32_t Point32_size_of_topic(const Point32* topic, uint32_t size)
{
    uint32_t previousSize = size;
    size += ucdr_alignment(size, 4) + 4;

    size += ucdr_alignment(size, 4) + 4;

    size += ucdr_alignment(size, 4) + 4;

    return size - previousSize;
}

static uint32_t Point32_odo_size_of_topic(const Point32_odo* topic, uint32_t size)
{
    uint32_t previousSize = size;
    size += ucdr_alignment(size, 4) + 4;

    size += ucdr_alignment(size, 4) + 4;

    size += ucdr_alignment(size, 4) + 4;

    return size - previousSize;
}