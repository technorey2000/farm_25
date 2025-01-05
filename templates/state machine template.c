/*Creating a New State Machine (SM):
 - Create a workflow for the commmands needed to perform the operation. 
 - Each command that is called needs to respond with a pass or fail result.
 - Create two structures to contain the response of the commands (complete 
   and results). One to indicate if a command was executed (complete) and 
   the other to indicate if the execution passed or failed (results). These 
   structures are located in the common.h file.
 - Assign a set of state machine states (SCTL_SMXX_ST00, SCTL_SMXX_ST98, SCTL_SMXX_ST99)
 - In the sysControlTask.c, define a static variable to correspond with the complete 
   and results structures.
 - Create two functions. One to start the SM and the other to operate the SM.
   The state machine functions are located in the stateMachine.c file. Make sure to put 
   the function prototypes at the top of the stateMachines.h file.
 - Add a reference to the state machine in the enum system_state_machines.
 - Add the start function and reference to the sysProcessTestStateMachineStart function.
 - Add a case statement for the state machine to the bool 
   sysProcessTestStateMachine(uint16_t stateMachineId) function

 -
 -
 -
 -
 -
 -
 -
 -

 
*/

/*Create a workflow for the commmands needed to perform the operation.
Example Here are the commads the SM needs to run:

Wifi Command - "wifi get device data from storage.txt"
Wifi Command - "wifi https get device access token.txt
Wifi Command - "wifi https create lox device.txt"

*/




















//----------------------------------------------------------------------------------------------------------
//These structures are located in the common.h file:
//----------------------------------------------------------------------------------------------------------

//These flags tell the state machine when individual commands are completed.
typedef struct sys_<smName>_complete
{
  uint8_t start_SM          : 1;    //Start the state machine? 1 = yes, 0 = no
  uint8_t is_<step1>Done    : 1;    //<step1> complete
  uint8_t is_<step2>Done    : 1;    //<step2> complete
  uint8_t is_<step3>Done    : 1;    //<step3> complete
  uint8_t is_<stepN>Done    : 1;    //<step4> complete
  uint8_t unused<smName>    : 2;    //unused flags. Total flag number should be in an increment of 8.
  uint8_t is_SystemDone     : 1;    //State machine complete
} sys_<smName>_complete_t;

//These flags tell the state machine the results of execution of individual commands.
typedef struct sys_<smName>_result
{
  uint8_t <result1>			: 1;    //result 1 from step1
  uint8_t <result2>			: 1;    //result 2 from step2
  uint8_t <result3>			: 1;    //result 3 from step3
  uint8_t <resultN>			: 1;    //result N from stepN
  uint8_t unused<smName>	: 3;	//unused flags. Total flag number should be in an increment of 8.
  uint8_t <resultSummary>	: 1;    //Final results of the state machine
} sys_<smName>_result_t;

//Go to enum system_control_sys_state 
//Add a state SCTL_SMX_ST00 = X00, example: SCTL_SM1_ST00          = 100

//----------------------------------------------------------------------------------------------------------
//This code is located in the sysControlTask.c file
//----------------------------------------------------------------------------------------------------------
static sys<smName>Complete_t          sys<smName>Complete;
static sys<smName>Result_t            sys<smName>Result; 

//----------------------------------------------------------------------------------------------------------
//This code is located in the stateMachines.h file
//----------------------------------------------------------------------------------------------------------

void sysProcessSm<smName>Start(sys_<smName>_complete_t * sys<smName>Complete);
bool sysProcessSm<smName>(sys_<smName>_complete_t * sys<smName>Complete, sys_<smName>_result_t * sys<smName>Result);

//----------------------------------------------------------------------------------------------------------
//This code is located in the stateMachines.c file
//----------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------
//State Machine Name
//----------------------------------------------------------------------

/// @brief 
/// @param sysInitComplete 
void sysProcessSm<smName>Start(sys_<smName>_complete_t * sys<smName>Complete)
{
    sys<smName>Complete->start_SM = true;    //start the state machine
} 

/// @brief 
/// @param sys_<smName>_complete_t 
/// @param sys<smName>Result 
/// @return 
bool sysProcessSm<smName>(sys_<smName>_complete_t * sys<smName>Complete, sys_<smName>_result_t * sys<smName>Result)
{
    static uint16_t smSys<smName>State = SCTL_SMX_ST00;

    if (sys<smName>Complete->start_SM)
    {
        switch(smSys<smName>State)
        {
            case SCTL_SMX_ST00: 
                memset((void*)sys<smName>Complete,0,sizeof(sys<smName>Complete));
                sys<smName>Complete->start_SM = true;
                memset((void*)sys<smName>Result,0,sizeof(sys<smName>Result));
				ESP_LOGI(SM_TAG,"sysProcessSm<smName> - started");
				ESP_LOGI(SM_TAG,"Initialize Dispatcher");
                sysSendMessage(MSG_ADDR_DSPR, MSG_DATA_0, DSPR_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                smSys<smName>State = SCTL_SM1_ST10;
                break;

            //Intialization Sequence
            case SCTL_SMX_ST10:
                if (sys<smName>Complete->is_<step1>Done)
                {
                    ESP_LOGI(SM_TAG,"Initialize Storage");
                    sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    smSys<smName>State = SCTL_SM1_ST20;
                }
                break;
            case SCTL_SMX_ST20:
                if (sys<smName>Complete->is_<step2>Done)
                {
                    ESP_LOGI(SM_TAG,"Initialize Serial");
                    sysSendMessage(MSG_ADDR_SER, MSG_DATA_0, SER_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    smSys<smName>State = SCTL_SM1_ST30;
                }
                break;
            case SCTL_SMX_ST30:
                if (sys<smName>Complete->is_<step3>Done)
                {
                    ESP_LOGI(SM_TAG,"Initialize LED");
                    sysSendMessage(MSG_ADDR_LED, MSG_DATA_0, LED_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    smSys<smName>State = SCTL_SM1_ST40;
                }
                break;
				
			//There is a simplified version below for this last state.	
            case SCTL_SMX_ST40:
                {
                    sys<smName>Complete->is_SystemDone = true;

                    if (sys<smName>Result-><result1>      && 
                        sys<smName>Result-><result2>      &&
                        sys<smName>Result-><result3>      &&
                        sys<smName>Result-><resultN>)
                    {
                        sys<smName>Result-><resultSummary> = true;
                        ESP_LOGI(SM_TAG,"sysProcessSm<smName> - Passed");
                    }
                    else
                    {
                        ESP_LOGI(SM_TAG,"sysProcessSm<smName> - Failed");
                        if (!sys<smName>Result-><result1>)    {ESP_LOGE(SM_TAG,"result1 - Failed");}
                        if (!sys<smName>Result-><result2>)    {ESP_LOGE(SM_TAG,"result2 - Failed");}
                        if (!sys<smName>Result-><result3>)    {ESP_LOGE(SM_TAG,"result3 - Failed");}
                        if (!sys<smName>Result-><resultN>)    {ESP_LOGE(SM_TAG,"resultN - Failed");}
                    }
                    sys<smName>Complete->start_SM = false;
                    smSys<smName>State = SCTL_SMX_ST00;
					ESP_LOGI(SM_TAG,"sysProcessSm<smName> - complete");
                }
                break;

            default:   
                sys<smName>Complete->start_SM = false;
                smSys<smName>State = SCTL_SMX_ST00;
                ESP_LOGE(SM_TAG,"sysProcessSm<smName> - default");
                break;
        }
    }
    return sys<smName>Complete->is_SystemDone;
}



Simplified version:

            case SCTL_SMX_ST40:
                {
                    sys<smName>Complete->is_SystemDone = true;
                    sys<smName>Result-><resultSummary> = true;
                    ESP_LOGI(SM_TAG,"sysProcessSm<smName> - Passed");
                    sys<smName>Complete->start_SM = false;
                    smSys<smName>State = SCTL_SMX_ST00;
					ESP_LOGI(SM_TAG,"sysProcessSm<smName> - complete");
                }
                break;

