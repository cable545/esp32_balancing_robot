#ifndef _CMD_H
#define _CMD_H

#define COMMAND_LENGTH            8

#define DEFAULT_GROUP 0x00
#define DEFAULT_COMMAND 0x00

/* ---------------------- GENERAL_GROUP ---------------------*/
#define GENERAL_GROUP 0x01
/* commands */ 
#define SET_CONTROLLER_P_GAIN 0x01
#define GET_CONTROLLER_P_GAIN 0x02
#define SET_CONTROLLER_I_GAIN 0x03
#define GET_CONTROLLER_I_GAIN 0x04

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
    static bool requestHandler(char data, CommandContainer* cmdContainer);

  private:
    static void resetArray(char* commandArray, uint32_t length);
    static float charArrayToFloat(char* startAddress, uint32_t length);
    static bool findGrpAndCmdId(char* commandString, CommandContainer* cmdContainer);
};

#endif
