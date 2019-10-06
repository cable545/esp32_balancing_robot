#include "Arduino.h"
#include "Cmd.h"

#define COMMAND_LENGTH            8

#define START_GET_CHAR            0x67
#define START_SET_CHAR            0x73
#define SET_SEPERATOR             0x5f
#define GET_SEPERATOR             0x3f

#define IDLE_STATE                0x00
#define COMMAND_IDENTIFIER_STATE  0x01
#define PARAMETER_SEPERATOR_STATE 0x02
#define PAYLOAD_STATE             0x03

bool Cmd::requestHandler(char data)
{
  static char state = IDLE_STATE;
  static int characterCounter = 0;
  static char command[8];

  switch(state)
  {
    case IDLE_STATE:
      command[0] = 0;
      command[1] = 0;
      command[2] = 0;
      command[3] = 0;
      command[4] = 0;
      command[5] = 0;
      command[6] = 0;
      command[7] = 0;

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
          
      }
      else if(characterCounter > 2
      {
        if(payload[characterCounter - 1] == 0x0a && payload[characterCounter] == 0x0d)
        {
          /* TODO take data from characterCounter == 0 to characterCounter - 2
           *  and transform it to a decimal
           */
        }
      }
      
      Serial1.println(command);
      state = IDLE_STATE;
      
      break;
     default: break; 
  }

  /*
   * TODO process command if command already parsed
   */
}
