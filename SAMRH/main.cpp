/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <stdio.h>                      // For printf debugging
#include <string.h>                     // For strings



// *****************************************************************************
// Section: Global Variables
// *****************************************************************************

// System configuration flags (simulated for testing)
bool configured = true;
bool self_test_performed = true;
bool homed = true;
bool reference_position_set = true;

bool error_flag = false;

uint16_t buff[1];
uint8_t duty_buff[1];
uint8_t czt[3];
uint8_t zetat[3];
uint8_t xit[3];
int xit_conv;
int czt_conv;
int zetat_conv;
int duty_conv;
int pos_limit = 360;
uint8_t duty[] = {0x4};

// Timer threshold for operation mode
uint32_t timer = 49000000;

// *****************************************************************************
// Section: State Machine Definitions
// *****************************************************************************

// Forward declaration of state structure
struct state;

// Function pointer type for state transitions
typedef void state_change(struct state *);

// State structure holding next and previous state functions
struct state {
    state_change *next;
    state_change *prev;
};

// State function declarations
state_change start_up;
state_change initialization;
state_change stand_by;
state_change operation;
state_change maintenance;
state_change error;


int convert(uint8_t *buf)
{
    int num = 0;

    for (int i = 0; i < 3; i++) {
        num = num * 10 + (buf[i] - '0'); 
    }
    return num;
}




// *****************************************************************************
// Section: State Functions
// *****************************************************************************

// START_UP state: Entry point of the system
void start_up(struct state * state_cur)
{
    printf("Current State: %s\n", __func__);
    printf("Enter n to enter next state (initialization)\n");
    printf("Enter m to enter next maintenance\n");
    printf("Command: ");
    FLEXCOM1_USART_Read(buff, (sizeof(buff) - 1));
    printf("\n");
    

    if ((buff[0] == 'n') && (configured))
    {
        
        state_cur->next = initialization;
        state_cur->prev = start_up;
    }

    else if (buff[0] == 'm')
    {
        
        state_cur->next = maintenance;
        state_cur->prev = start_up;
    }

    else
    {
        state_cur->next = error;
        state_cur->prev = start_up;
        error_flag = true;
    }
}

// INITIALIZATION state: Checks system readiness
void initialization(struct state * state_cur)
{
    printf("Current State: %s\n", __func__);
    printf("Enter n to enter next state (Stand By)\n");
    printf("Enter m to enter next maintenance\n");
    printf("Command: ");
    FLEXCOM1_USART_Read(buff, (sizeof(buff) - 1));
    printf("\n");


    if ((buff[0] == 'n') && (self_test_performed) && (homed) && (reference_position_set))
    {
        state_cur->next = stand_by;
        state_cur->prev = initialization;
    }
    
    
    else if (buff[0] == 'm')
    {
        state_cur->next = maintenance;
        state_cur->prev = initialization;
    }
    

    else
    {
        state_cur->next = error;
        state_cur->prev = initialization;
        error_flag = true;
    }
}

// STAND_BY state: Waits for user action 
void stand_by(struct state * state_cur)
{
    printf("Current State: %s\n", __func__);
    printf("Enter n to set a point and enter Operation\n");
    printf("Enter m to enter next maintenance\n");
    printf("Command: ");
    FLEXCOM1_USART_Read(buff, (sizeof(buff) - 1));
    printf("\n");

    if (buff[0] == 'n')
    {   
        
        printf("Set you czt, zetat, and xit in allowed degrees\n");
        printf("czt: ");
        FLEXCOM1_USART_Read(czt, (sizeof (czt)));
        czt_conv = convert(czt);
        
        if(czt_conv > pos_limit)
        {
            printf("Error Out of Range: czt > 360 degrees\n");
            state_cur->next = stand_by;
            state_cur->prev = stand_by;
            return;
        }
        printf("\nzetat: ");
        FLEXCOM1_USART_Read(zetat, (sizeof (zetat)));
        zetat_conv = convert(zetat);
        
        if(zetat_conv > pos_limit)
        {
            printf("Error Out of Range: zetat > 360 degrees\n");
            state_cur->next = stand_by;
            state_cur->prev = stand_by;
            return;
        }
        printf("\nxit: ");
        FLEXCOM1_USART_Read(xit, (sizeof (xit)));
        xit_conv = convert(xit);
        
        if(xit_conv > pos_limit)
        {
            printf("Error Out of Range: xit > 360 degrees\n");
            state_cur->next = stand_by;
            state_cur->prev = stand_by;
            return;
        }
        printf("\n");
        printf("Enter s to enter operation and start movement: ");
        FLEXCOM1_USART_Read(buff, sizeof(buff) - 1);
        printf("\n");
        
        if(buff[0] == 's')
        {
            printf("Enabling actuator drivers\n");
            state_cur->next = operation;
            state_cur->prev = stand_by;
        }
        else
        {
            error_flag = true;
            printf("Error: Start command not received\n");
            state_cur->next = error;
            state_cur->prev = stand_by;
        }
        printf("Enter duty cycle divider (2 or 4) ");
        FLEXCOM1_USART_Read(duty_buff, sizeof(duty_buff));
        if(duty_buff[0] == '2')
        {
            uint8_t txdata[] = {0x2};
            FLEXCOM3_SPI_Write(txdata, sizeof (txdata));
        }
        
        else if(duty_buff[0] == '4')
        {
            uint8_t txdata[] = {0x4};
            FLEXCOM3_SPI_Write(txdata, sizeof (txdata));
        }
        
        FLEXCOM3_SPI_Write(duty, sizeof(duty));
        printf("\n");
    }

    else if (buff[0] == 'm')
    {
        
        state_cur->next = maintenance;
        state_cur->prev = stand_by;
    }
}

// OPERATION state: Simulates timed operation with user control                 
void operation(struct state * state_cur)
{
    //First: put given czt, zetat, and xit into kinematics formula
    //PBSS_Controller
    //Second: write given actuator values to FPGA
    //FLEXCOM3_SPI_Write(void* pTransmitData, size_t txSize);
    
    
    printf("Current State: %s\n", __func__);
    printf("Enter p to set new point\n");
    printf("Enter s to stop\n");
    if (FLEXCOM1_USART_ReceiverIsReady())
    {
        FLEXCOM1_USART_Read(buff, (sizeof(buff) - 1));
    }


    int total_seconds = 10;

    for (int cur_second = 1; cur_second <= total_seconds; cur_second++)
    {
        TC0_CH1_TimerStart();


        while (TC0_CH1_TimerCounterGet() < timer)
        {

            if (FLEXCOM1_USART_ReceiverIsReady()) 
            {
                FLEXCOM1_USART_Read(buff, 1);
  
                if (buff[0] == 'p')
                {
                    printf("\n");
                    printf("Set new point command received \n");
                    state_cur->next = stand_by;
                    state_cur->prev = operation;
                    buff[0] = 0;
                    return;
                }


                if (buff[0] == 's')
                {
                    printf("\n");
                    printf("Stopping command received \n");
                    state_cur->next = stand_by;
                    state_cur->prev = operation;
                    buff[0] = 0;
                    return;
                }
            }
        }

        TC0_CH1_TimerStop();
        printf("Setting point: %d\n", cur_second);
    }


    state_cur->next = stand_by;
    state_cur->prev = operation;
}

// MAINTENANCE state: Enter back to previous state after performing maintenance
void maintenance(struct state * state_cur)
{
    printf("Current State: %s\n", __func__);
    printf("Enter 1 to Perform Self-test | Enter 2 to Perform Homing | Enter 3 to go to Reference Position | "
            "Enter 4 to Update FWSW | Enter 5 to Upload Config | Enter 6 to Download Config\n");
    printf("Command: ");
    FLEXCOM1_USART_Read(buff, (sizeof(buff) - 1));
    printf("\n");


    if (buff[0] == '1')
    {
        printf("Self Test Performed\n");
        printf("Press r to Re-Enter maintenance\n");
        printf("Press e to exit maintenance\n");
        FLEXCOM1_USART_Read(buff, (sizeof(buff) -1));
        if(buff[0] == 'r')
        {
            state_cur->next = maintenance; 
            state_cur->prev = state_cur->prev;
        }
        else if(buff[0] == 'e')
        {
           state_cur->next = state_cur->prev; 
           state_cur->prev = state_cur->prev; 
        }
        else
        {
            state_cur->next = error;
            state_cur->prev = maintenance;
        }
    }
    
    else if (buff[0] == '2')
    {
        printf("Homing Performed\n");
        printf("Press r to Re-Enter maintenance\n");
        printf("Press e to exit maintenance\n");
        FLEXCOM1_USART_Read(buff, (sizeof (buff) - 1));
        if (buff[0] == 'r') 
        {
            state_cur->next = maintenance;
            state_cur->prev = state_cur->prev;
        } 
        else if (buff[0] == 'e') 
        {
            state_cur->next = state_cur->prev;
            state_cur->prev = state_cur->prev;
        }
        else
        {
            state_cur->next = error;
            state_cur->prev = maintenance;
        }
    }
    
    else if (buff[0] == '3')
    {
        printf("Reference Position received\n");
        printf("Press r to Re-Enter maintenance\n");
        printf("Press e to exit maintenance\n");
        FLEXCOM1_USART_Read(buff, (sizeof (buff) - 1));
        if (buff[0] == 'r') 
        {
            state_cur->next = maintenance;
            state_cur->prev = state_cur->prev;
        } 
        else if (buff[0] == 'e') 
        {
            state_cur->next = state_cur->prev;
            state_cur->prev = state_cur->prev;
        }
        else
        {
            state_cur->next = error;
            state_cur->prev = maintenance;
        }
    }
    
    else if (buff[0] == '4')
    {
        printf("FWSW Updated\n");
        printf("Press r to Re-Enter maintenance\n");
        printf("Press e to exit maintenance\n");
        FLEXCOM1_USART_Read(buff, (sizeof (buff) - 1));
        if (buff[0] == 'r') 
        {
            state_cur->next = maintenance;
            state_cur->prev = state_cur->prev;
        } 
        else if (buff[0] == 'e') 
        {
            state_cur->next = state_cur->prev;
            state_cur->prev = state_cur->prev;
        }
        else
        {
            state_cur->next = error;
            state_cur->prev = maintenance;
        }
    }
    
    else if (buff[0] == '5')
    {
        printf("Config Uploaded\n");
        printf("Press r to Re-Enter maintenance\n");
        printf("Press e to exit maintenance\n");
        FLEXCOM1_USART_Read(buff, (sizeof (buff) - 1));
        if (buff[0] == 'r') 
        {
            state_cur->next = maintenance;
            state_cur->prev = state_cur->prev;
        } 
        else if (buff[0] == 'e') 
        {
            state_cur->next = state_cur->prev;
            state_cur->prev = state_cur->prev;
        }
        else
        {
            state_cur->next = error;
            state_cur->prev = maintenance;
        }
    }
    
    else if (buff[0] == '6')
    {
        printf("Config Downloaded\n");
        printf("Press r to Re-Enter maintenance\n");
        printf("Press e to exit maintenance\n");
        FLEXCOM1_USART_Read(buff, (sizeof (buff) - 1));
        if (buff[0] == 'r') 
        {
            state_cur->next = maintenance;
            state_cur->prev = state_cur->prev;
        } 
        else if (buff[0] == 'e') 
        {
            state_cur->next = state_cur->prev;
            state_cur->prev = state_cur->prev;
        }
        else
        {
            state_cur->next = error;
            state_cur->prev = maintenance;
        }
    }
    
    else
    {
        state_cur->next = maintenance; 
        state_cur->prev = state_cur->prev;
    }
}

// ERROR state: Handles error recovery
void error(struct state * state_cur)
{
    printf("Current State: %s\n", __func__);
    printf("Enter r restart\n");
    printf("Enter m to enter next maintenance\n");
    printf("Command: ");
    FLEXCOM1_USART_Read(buff, (sizeof(buff) - 1));
    printf("\n");
    
    error_flag = false;

    


    if ((buff[0] == 'r') && (error_flag == false))
    {
        state_cur->next = start_up; 
        state_cur->prev = start_up;
    }
    

    else if (buff[0] == 'm')
    {
        state_cur->next = maintenance;
        state_cur->prev = error;
    }
    
}



// *****************************************************************************
// Section: Main Function
// *****************************************************************************

int main(void)
{
    // Initialize all system modules
    SYS_Initialize(NULL);

    char start_buff[6];
    
    // Initial state is START_UP
    struct state state_cur = {
        start_up,
        start_up
    };

    
    printf("Enter start to begin state machine\n");
    printf("Command: ");
    FLEXCOM1_USART_Read((uint16_t*)start_buff, 5);
    start_buff[5] = '\0';
    printf("\n");
    

    while (true)
    {
        // Wait for power-on button press
        if (strcmp(start_buff, "start") == 0)
        {
            
            // Currently runs infinitely
            while (state_cur.next)
            {
                state_cur.next(&state_cur);
            }
        }

        // Maintain Harmony system tasks
        SYS_Tasks();
    }

    // Should never reach here
    return EXIT_FAILURE;
}

/*******************************************************************************
 End of File
*/
