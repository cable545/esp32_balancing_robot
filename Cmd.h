#ifndef _CMD_H
#define _CMD_H

#include "Config.h"

#define CR                        0x0D
#define LF                        0x0A

#define COMMAND_LENGTH            8
#define PAYLOAD_LENGTH            12

#define DEFAULT_GROUP 0x00
#define DEFAULT_COMMAND 0x00

/* ---------------------- GET_REQUEST_GROUP ---------------------*/
#define GET_REQUEST_GROUP 0x01
/* commands */ 
#define GET_CONTROLLER_P_GAIN 0x01
#define GET_CONTROLLER_I_GAIN 0x02
#define GET_CONTROLLER_D_GAIN 0x03
#define GET_CONTROLLER_I_LIMIT 0x04
#define GET_CONTROLLER_LOOP_TIME 0x05
#define GET_LAST_RESET_REASON 0x06

/* ---------------------- SET_REQUEST_GROUP ---------------------*/
#define SET_REQUEST_GROUP 0x02
/* commands */ 
#define SET_CONTROLLER_P_GAIN 0x01
#define SET_CONTROLLER_I_GAIN 0x02
#define SET_CONTROLLER_D_GAIN 0x03
#define SET_CONTROLLER_I_LIMIT 0x04
#define SET_STORE_CONFIG_IN_FLASH 0x05

typedef struct
{
  char* commandString;
  uint32_t groupId;
  uint32_t commandId;
  float parameter;
  float response;
}CommandContainer;

class Cmd
{
  public:
    Cmd(Config *p_config, QueueHandle_t *p_queue);
    bool requestHandler(char data, CommandContainer* cmdContainer);
    void processRequest(CommandContainer* cmdContainer);
    void buildResponse(CommandContainer* cmdContainer, char *responseBuffer);

  private:
    void resetArray(char* commandArray, uint32_t length);
    float charArrayToFloat(char* startAddress, uint32_t length);
    bool findGrpAndCmdId(char* commandString, CommandContainer* cmdContainer);
    void getRequestGroupHandler(CommandContainer *cmdContainer);
    void setRequestGroupHandler(CommandContainer *cmdContainer);
    bool isSetRequest(CommandContainer* cmdContainer);
    bool isGetRequest(CommandContainer* cmdContainer);
    void putPidDataInQueue();
    
    Config *p_myConfig;
    QueueHandle_t *p_messageQueue;
};

#endif
