#include "Arduino.h"
#include "Cmd.h"

#define CR                        0x0D
#define LF                        0x0A

#define PAYLOAD_LENGTH            12

#define START_GET_CHAR            0x47
#define START_SET_CHAR            0x53
#define SET_SEPERATOR             0x5f
#define GET_SEPERATOR             0x3f

#define IDLE_STATE                0x00
#define COMMAND_IDENTIFIER_STATE  0x01
#define PARAMETER_SEPERATOR_STATE 0x02
#define PAYLOAD_STATE             0x03

typedef struct
{
  char commandString[9];
  char groupId;
  char commandId; 
}Command;

static Command commands[] = {
  {"SCOPGAIN", GENERAL_GROUP, SET_CONTROLLER_P_GAIN},
  {"GCOPGAIN", GENERAL_GROUP, GET_CONTROLLER_P_GAIN},
  {"SCOIGAIN", GENERAL_GROUP, SET_CONTROLLER_I_GAIN},
  {"GCOIGAIN", GENERAL_GROUP, GET_CONTROLLER_I_GAIN}, 
};

bool Cmd::requestHandler(char data, CommandContainer* cmdContainer)
{
  static char state = IDLE_STATE;
  static int characterCounter = 0;
  static char command[COMMAND_LENGTH + 1] = "";
  static char payload[PAYLOAD_LENGTH];

  bool newCommandParsed = false;

  if(data > 0x60 && data < 0x7b)
    data -= 0x20;

  switch(state)
  {
    case IDLE_STATE:
      resetArray(command, COMMAND_LENGTH);
      resetArray(payload, PAYLOAD_LENGTH);
      characterCounter = 0;

      if(data == START_GET_CHAR || data == START_SET_CHAR)
      {
        state = COMMAND_IDENTIFIER_STATE;
        command[characterCounter] = data;
        characterCounter++;
      }

      break;
    case COMMAND_IDENTIFIER_STATE:
      command[characterCounter++] = data;

      if(characterCounter == COMMAND_LENGTH)
      {
        state = PARAMETER_SEPERATOR_STATE;
        characterCounter = 0;
      }

      break;
    case PARAMETER_SEPERATOR_STATE:
      if((command[0] == START_GET_CHAR && data == GET_SEPERATOR) || (command[0] == START_SET_CHAR && data == SET_SEPERATOR))
        state = PAYLOAD_STATE;
      else
        state = IDLE_STATE;
    
      break;
    case PAYLOAD_STATE:
      payload[characterCounter++] = data;

      if(characterCounter == 2)
      {
        if(payload[0] == 0x0a && payload[1] == 0x0d)
          state = IDLE_STATE;
      }
      else if(characterCounter > 2)
      {
        if(payload[characterCounter - 2] == CR && payload[characterCounter - 1] == LF)
        {
          cmdContainer->commandString = command;
          cmdContainer->parameter = charArrayToFloat(payload, characterCounter - 2);
          
          if(!findGrpAndCmdId(command, cmdContainer))
          {
            cmdContainer->groupId = DEFAULT_GROUP;
            cmdContainer->commandId = DEFAULT_COMMAND;
          }

          newCommandParsed = true;
          state = IDLE_STATE;
        }
      }
      
      break;
    default: break; 
  }

  return newCommandParsed;
}

void Cmd::resetArray(char* commandArray, uint32_t length)
{
  for(int i = 0; i < length; i++)
    commandArray[i] = 0;
}

float Cmd::charArrayToFloat(char* startAddress, uint32_t length)
{
  char buffer[PAYLOAD_LENGTH - 2] = {0};
  float result;
  
  if(length > sizeof(buffer))
  {
    result = 0.0;
  }
  else
  {
    for(int i = 0; i < length; i++)
      buffer[i] = startAddress[i];

    result = atof(buffer);
  }

  return result;
}

bool Cmd::findGrpAndCmdId(char* commandString, CommandContainer* cmdContainer)
{
  bool assigned = false;
  
  for(int i = 0; i < sizeof(commands); i++)
  {
    if(strcmp(commands[i].commandString, commandString) == 0)
    {
      cmdContainer->groupId = commands[i].groupId;
      cmdContainer->commandId = commands[i].commandId;
      assigned = true;

      break;
    }
  }

  return assigned;
}
