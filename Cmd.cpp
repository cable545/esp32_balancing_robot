#include "Arduino.h"
#include "Cmd.h"
#include "Pid.h"

#define PAYLOAD_INDEX             COMMAND_LENGTH + 1

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
  {"SCOPGAIN", SET_REQUEST_GROUP, SET_CONTROLLER_P_GAIN},
  {"GCOPGAIN", GET_REQUEST_GROUP, GET_CONTROLLER_P_GAIN},
  {"SCOIGAIN", SET_REQUEST_GROUP, SET_CONTROLLER_I_GAIN},
  {"GCOIGAIN", GET_REQUEST_GROUP, GET_CONTROLLER_I_GAIN},
  {"SCODGAIN", SET_REQUEST_GROUP, SET_CONTROLLER_D_GAIN},
  {"GCODGAIN", GET_REQUEST_GROUP, GET_CONTROLLER_D_GAIN},
  {"SCONILIM", SET_REQUEST_GROUP, SET_CONTROLLER_I_LIMIT},
  {"GCONILIM", GET_REQUEST_GROUP, GET_CONTROLLER_I_LIMIT},
  {"SSTOCONF", SET_REQUEST_GROUP, SET_STORE_CONFIG_IN_FLASH},
  {"GCOLOTIM", GET_REQUEST_GROUP, GET_CONTROLLER_LOOP_TIME},
};

extern uint32_t elapsedMicros;

Cmd::Cmd(Config *p_config, QueueHandle_t *p_queue)
{
  p_myConfig = p_config;
  p_messageQueue = p_queue;
}

bool Cmd::requestHandler(char data, CommandContainer *cmdContainer)
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

      // command without parameter
      if(characterCounter == 2 && payload[0] == CR && payload[1] == LF)
      {
        if(command[0] == START_GET_CHAR)
        {
          cmdContainer->commandString = command;
          
          if(!findGrpAndCmdId(command, cmdContainer))
          {
            cmdContainer->groupId = DEFAULT_GROUP;
            cmdContainer->commandId = DEFAULT_COMMAND;
          }
          
          newCommandParsed = true;
        }
        
        state = IDLE_STATE;
      }
      else if(characterCounter > 2)
      {
        if(payload[characterCounter - 2] == CR && payload[characterCounter - 1] == LF)
        {
          cmdContainer->commandString = command;
          cmdContainer->parameter = charArrayToFloat(payload, characterCounter - 2);
          cmdContainer->response = cmdContainer->parameter;
          
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

void Cmd::resetArray(char *commandArray, uint32_t length)
{
  for(int i = 0; i < length; i++)
    commandArray[i] = 0;
}

float Cmd::charArrayToFloat(char *startAddress, uint32_t length)
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

bool Cmd::findGrpAndCmdId(char *commandString, CommandContainer *cmdContainer)
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

void Cmd::processRequest(CommandContainer *cmdContainer)
{
  switch(cmdContainer->groupId)
  {
    case GET_REQUEST_GROUP:
      getRequestGroupHandler(cmdContainer);
      
      break;
    case SET_REQUEST_GROUP:
      setRequestGroupHandler(cmdContainer);
      
      break; 
    default:
      break;  
  }
}

void Cmd::buildResponse(CommandContainer *cmdContainer, char *responseBuffer)
{
  strcpy(responseBuffer, cmdContainer->commandString);  
  responseBuffer[COMMAND_LENGTH] = SET_SEPERATOR;

  // TODO dependent from parameter type build response
  // at the moment response is always a decimal number
  sprintf(&responseBuffer[PAYLOAD_INDEX], "%f", cmdContainer->response);
}

void Cmd::getRequestGroupHandler(CommandContainer *cmdContainer)
{
  switch(cmdContainer->commandId)
  {
    case GET_CONTROLLER_P_GAIN:
      cmdContainer->response = p_myConfig->getPGainAngle();
      
      break;
    case GET_CONTROLLER_I_GAIN:
      cmdContainer->response = p_myConfig->getIGainAngle();
    
      break;
    case GET_CONTROLLER_D_GAIN:
      cmdContainer->response = p_myConfig->getDGainAngle();

      break;
    case GET_CONTROLLER_I_LIMIT:
      cmdContainer->response = p_myConfig->getILimitAngle();
    
      break;
    case GET_CONTROLLER_LOOP_TIME:
      cmdContainer->response = elapsedMicros;
      
      break;  
    default:
      break;  
  }
}

void Cmd::setRequestGroupHandler(CommandContainer *cmdContainer)
{
  switch(cmdContainer->commandId)
  {
    case SET_CONTROLLER_P_GAIN:
      p_myConfig->setPGainAngle(cmdContainer->parameter);
      
      break;
    case SET_CONTROLLER_I_GAIN:
      p_myConfig->setIGainAngle(cmdContainer->parameter);
      
      break;
    case SET_CONTROLLER_D_GAIN:
      p_myConfig->setDGainAngle(cmdContainer->parameter);
      
      break;
    case SET_CONTROLLER_I_LIMIT:
      p_myConfig->setILimitAngle(cmdContainer->parameter);

      break;
    case SET_STORE_CONFIG_IN_FLASH:
      cmdContainer->response = p_myConfig->update();
      putPidDataInQueue();

      break;
    default:
      break;  
  }
}

void Cmd::putPidDataInQueue()
{
  PidGainContainer pidGainContainer;
  
  pidGainContainer.pGainAngle = p_myConfig->getPGainAngle();
  pidGainContainer.iGainAngle = p_myConfig->getIGainAngle();
  pidGainContainer.dGainAngle = p_myConfig->getDGainAngle();
  pidGainContainer.iLimitAngle = p_myConfig->getILimitAngle();
  
  xQueueSend(*p_messageQueue, &pidGainContainer.pGainAngle, portMAX_DELAY);
  xQueueSend(*p_messageQueue, &pidGainContainer.iGainAngle, portMAX_DELAY);
  xQueueSend(*p_messageQueue, &pidGainContainer.dGainAngle, portMAX_DELAY);
  xQueueSend(*p_messageQueue, &pidGainContainer.iLimitAngle, portMAX_DELAY);
}

bool Cmd::isSetRequest(CommandContainer *cmdContainer)
{
  return cmdContainer->groupId % 2 == 0 ? true : false;
}

bool Cmd::isGetRequest(CommandContainer *cmdContainer)
{
  return cmdContainer->groupId % 2 > 0 ? true : false;
}
