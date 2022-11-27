/*
 * File:   
 * Author: JA
 *
 * 
 * 
 * It reads periodically 1 sensor, and writes the AD conversions results.
 * 
 */


// DSPIC33FJ128MC802 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Mode (Primary Oscillator (XT, HS, EC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h"
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include "AuK_v1_1_8.h"

/*-----------------------------------------------------------------
 AuK variables and routines
 ------------------------------------------------------------------*/



// Used to inform of the internal scheduler period in seconds
float s_tic_period; 

/* Returns the number of seconds between two tics of scheduler timer 
 The tic period is correctly initialized when init_tmr1_int_frequency is
 invoqued */
float tic_period()
{
   return(s_tic_period); 
}

void init_uart(void)
{
    /* Specified pins for UART1 */
    RPINR18bits.U1RXR = 6; /* Pin RP6 asigned to UART1 RX */
    RPOR3bits.RP7R = 0b00011; /* UART1 TX asigned to RP7 */
    
    U1MODEbits.USIDL = 1;   // Stop on idle mode
    U1MODEbits.IREN = 0;    // disable IrDA operation
    U1MODEbits.UEN = 0;     // Only RX and TX are used (non CTS, RTS and BCLK)
    U1MODEbits.WAKE = 0;    // Wake up on start bit is disabled
    U1MODEbits.LPBACK = 0;  // Loopback mode disabled
    U1MODEbits.ABAUD = 0;   // No automatic baud rate
    U1MODEbits.URXINV = 0;  // Non polarity inversion. Idle state is 1
    U1MODEbits.BRGH = 0;    // High baude rate disabled
    U1MODEbits.PDSEL = 0;   // 8 bit data with no parity
    U1MODEbits.STSEL = 0;   // One stop bit.
    
    U1STAbits.URXISEL = 0;  // Interrupt on each character received
    
    U1BRG = 257; // 9600 Baudios. 39.6288*10**6/(16*9600) - 1
    
    /* In this configuration uart interrupts are not allowed */
    IPC2bits.U1RXIP = 0; 
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 0;
    
    U1MODEbits.UARTEN = 1; // Enable UART operation
    U1STAbits.UTXEN = 1;    // Enable uart1 TX (must be done after UARTEN)
    
    /* It is needed to wait one transmision bit to ensure start bit detection 
     When 9600 Baud rate is selected it is necessary to wait 104 us */
    __delay32(4122);
}

void init_micro(void)
{
    RCONbits.SWDTEN = 0; // Disable Watchdog Timer

    // Configure Oscillator to operate the device at 40 Mhz
    // Fosc = Fin*M/(N1*N2), Fcy = Fosc/2
    // Fosc = 7.3728*43/(2*2) = 79.2576 Mhz
    // Fcy = Fosc/2 = 39.6288 MHz

    // Configure PLL prescaler, PLL postscaler and PLL divisor

    PLLFBDbits.PLLDIV = 41; // M = PLLDIV + 2 = 43 -> PLLDIV = 43 - 2 = 41
    CLKDIVbits.PLLPOST = 0; // N2 = 2 (Output/2)
    CLKDIVbits.PLLPRE = 0; // N1 = 2 (Input/2)

    // clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {}; // Wait for PLL to lock (If LOCK = 1 -> PLL start-up timer is satisfied)

}

void init_ports(void)
{
    /* All possible analog bits are configured as digital */
    AD1PCFGL = 0xFFFF;
    
    /* RA0 = AN0
       RA1 = AN1
       RB0 = AN2 */
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISBbits.TRISB0 = 1;
    AD1PCFGLbits.PCFG0 = 0;
    AD1PCFGLbits.PCFG1 = 0;
    AD1PCFGLbits.PCFG2 = 0;

    /* Set TRISx registers for uart */
    TRISBbits.TRISB6 = 1; /* RB6 is configured as input (UART1 RX) */
    TRISBbits.TRISB7 = 0; /* RB7 is configured as output (UART1 TX) */
    
}

int init_tmr1_int_frequency(float Fcyc, float scheduler_period)
{
    float PR1_count; //Expected timer1 instruction counts to interrupt.
        
    PR1_count = Fcyc * scheduler_period;
    if(PR1_count < 65535.0)
    {
        T1CONbits.TCKPS = 0;
        PR1 = (unsigned int)PR1_count;
    }
    else if((PR1_count/8.0) < 65535.0)
        {
            T1CONbits.TCKPS = 0b01;
            PR1 = (unsigned int)(PR1_count/8);
        }
        else if((PR1_count/64.0) < 65535.0)
            {
                T1CONbits.TCKPS = 0b10;
                PR1 = (unsigned int)(PR1_count/64);
            }
            else if((PR1_count/256.0) < 65535.0)
                {
                    T1CONbits.TCKPS = 0b11;
                    PR1 = (unsigned int)(PR1_count/256);
                }
                else
                    return(-1); //Incompatible data
        
    switch(T1CONbits.TCKPS)
    {
        case 0: s_tic_period = 1.0/(Fcyc / (float)PR1);
                break;
        case 1: s_tic_period = 1.0/((Fcyc / 8.0) / (float)PR1);
                break;
        case 2: s_tic_period = 1.0/((Fcyc / 64.0) / (float)PR1);
                break;
        case 3: s_tic_period = 1.0/((Fcyc / 256.0) / (float)PR1);
                break;
        default: s_tic_period = scheduler_period;
                break;
    }
    
    return(0);    
}

void init_tmr1(void)
{
    TMR1 = 0;    
    
    /* One interrupt each 10 ms if Fcy = Fosc/2 = 59.904 MHz */
    //PR1 = 2340; 
    
    T1CONbits.TSIDL = 1; /* Stop on idle mode */
    T1CONbits.TGATE = 0; /* Internal clock source */
    T1CONbits.TCS = 0;
    //T1CONbits.TCKPS = 0b11; /* 256 prescaler */

    IPC0bits.T1IP = 7; /* Priority level */
    IFS0bits.T1IF = 0; /* Clear interrupt flag */
    /* Disable T1 interrupts. It'll be enabled when idle_task will be invoked 
     to turn on uKernel */
    IEC0bits.T1IE = 0; 

    T1CONbits.TON = 0; /* Start timer */
}

void init_int2(void)
{
    RPINR1 = 0; // INT2 tied to vss
    IFS1bits.INT2IF = 0;
    
    // It could be set here because it is called only when AuK is working.
    IEC1bits.INT2IE = 1;
    
    IPC7bits.INT2IP = 7;
    
}

/* Variables for managin process memory */
/****************************************************************************/
unsigned int mem[max_memory_size_words];  // Word array for Task stacks
int total_mem_free_words;
unsigned int indx_free_mem = 0;
/****************************************************************************/

/* Variables for managing process state */
/****************************************************************************/

/* Task states */
enum T_state{running, blocked, ready, free_slot};

/* Time control*/
unsigned long current_tic;

/* Task control block */
struct TCB
{
    int stack_pointer;
    int state;
    int priority;
    unsigned long tics_to_wake_up;
    int delayed;
};

/* Task descriptor.
 Last slot (max_TCB_indx) will be used for Idle task. */
struct TCB TCB_Descriptor[max_TCB_indx + 1];

/* Maneuver variables used in asm code */

unsigned int temp_stack_pointer;
unsigned int temp_W0;

unsigned int task_stack_address;
unsigned int new_task_stack_pointer;
unsigned int task_code_address_page;
unsigned int task_code_address_offset;

unsigned int adr_current_task_stack_pointer;

/* Could be used if it is necessary to return from AuK */
unsigned int system_stack_pointer;


/* Task management variables */
int current_TCB_indx;

/* Asm functions */
extern void asmStackFrame(void);
/****************************************************************************/

/* Return address of the memory space whith the specified words.
 * If there is no space available to satisfy the request then -1 is returned. */
int GiveMeMemory(int num_words)
{
    if((total_mem_free_words - num_words) < 0)
        return(-1);
    else
        total_mem_free_words -= num_words;
    
    int return_address = (unsigned int)&mem[indx_free_mem];
    
    indx_free_mem += num_words;
    
    return(return_address);
}



void init_task_manager()
{
    int x;
    for (x=0; x<= max_TCB_indx; x++)
    {
        TCB_Descriptor[x].stack_pointer = 0;
        TCB_Descriptor[x].state = free_slot;
    }
    
    /* ATTENTION 
     * Avoid interrupt nesting. Priority interrupts are only used to 
     * resolve conflicts between simultaneous pending interrupts. */
    INTCON1bits.NSTDIS = 1;
    
    init_tmr1(); //It begins to work when start_AuK() is invoqued.
    init_int2();
}


int init_AuK(float Fcyc, float scheduler_period)
{
    int x;
    
    
    total_mem_free_words = max_memory_size_words;
            
    x = init_tmr1_int_frequency(Fcyc, scheduler_period);
    
    if(x == 0)
    {
        init_task_manager();
        return(0);
    }
    else
        return( x );
}

int give_me_my_id(void)
{
    return(current_TCB_indx);
}

/* Count tics. This function will be called from T1 timer interrupt routine.
 * It is responsible for waking up dormant processes on a timed basis. */

void count_tic()
{
    int x;
    
    current_tic++;
    
    for(x = 0; x < max_TCB_indx; x++)
    {
        if(TCB_Descriptor[x].delayed)
        {
            TCB_Descriptor[x].tics_to_wake_up--;
            if(TCB_Descriptor[x].tics_to_wake_up == 0)
            {
                TCB_Descriptor[x].delayed = FALSE;
                TCB_Descriptor[x].state = ready;
            }
        }
    }
}

/* Set adr_new_task_stack_pointer to the new active task */
void scheduler()
{
    int TCB_indx, indx_count, highest_priority, selected_task;
    
    
    if (TCB_Descriptor[current_TCB_indx].state == running)
        TCB_Descriptor[current_TCB_indx].state = ready;
    
    if(current_TCB_indx == max_TCB_indx)
        TCB_indx = 0;
    else
        TCB_indx = (current_TCB_indx +1) % max_TCB_indx;
    
    
    
    highest_priority = 0;
    
    for(indx_count = 1; indx_count <= max_TCB_indx; indx_count++)
    {
        if(TCB_Descriptor[TCB_indx].state == ready)
            if(TCB_Descriptor[TCB_indx].priority > highest_priority)
            {
                highest_priority = TCB_Descriptor[TCB_indx].priority;
                selected_task = TCB_indx;
            }
        TCB_indx = (TCB_indx + 1) % max_TCB_indx;
    }
    
    if(highest_priority == 0)    
        /* There's no ready task, then select idle task */
        TCB_indx = max_TCB_indx;
    else
        TCB_indx = selected_task;
        
    TCB_Descriptor[TCB_indx].state = running;
    
    adr_current_task_stack_pointer = 
            (unsigned int)&TCB_Descriptor[TCB_indx].stack_pointer;
    current_TCB_indx = TCB_indx;
    
}

/*-------------------------------------*/
/* Code_address must contain address of the new task code.
 * Code address_page could be obtained by means of __builtin_tblpage(func)
 * Code address_offset could be obtained by means of __builtin_tbloffset(func)
 * Where "func" is the name of the function which contains the code of 
 * the new task.
 * Mem_needed must contain  the nedded words for the task stack 
 * priority must contain the task priority. The larger the number 
 * the higher the priority. Priority must be greather than 0 */
int create_task(unsigned int code_address_page, 
                unsigned int code_address_offset,
                unsigned int mem_needed,
                unsigned int priority)
{
    int TCB_indx, answer;
    
    /* Allowable priorities are greather than 0 */
    if(priority == 0)
        return(-2);
    
    /* Find a free TCB */
    TCB_indx = 0;
    while((TCB_Descriptor[TCB_indx].state != free_slot) 
            && TCB_indx < max_TCB_indx)
        TCB_indx++;
    
    if (TCB_indx == max_TCB_indx) // max_TCB_indx is reserved for idle task
        /* There is no free TCB */
        return(-1);
    
    answer = GiveMeMemory(mem_needed);
    if(answer < 0)
        return(-3);
    task_stack_address = (unsigned int)answer;
    task_code_address_page = code_address_page;
    task_code_address_offset = code_address_offset;
    
    /* Create new task stack frame */
    asmStackFrame();
    
    /* Save new task Stack pointer from asm routine */
    TCB_Descriptor[TCB_indx].stack_pointer = new_task_stack_pointer;
    TCB_Descriptor[TCB_indx].state = ready;
    TCB_Descriptor[TCB_indx].priority = priority;
    return(TCB_indx);
}

/* This routine launches AuK and also contains the idle process code. */

void start_AuK(void)
{
    current_TCB_indx = max_TCB_indx; // Idle task
    
    /* Idle task only needs 82 words of stack to correctly save its state
     * when Timer 1 interrupt happens. 100 words are reserved to add a security
     * margin */
    
    TCB_Descriptor[current_TCB_indx].stack_pointer = GiveMeMemory(100);
    TCB_Descriptor[current_TCB_indx].state = running;
    TCB_Descriptor[current_TCB_indx].priority = 0;
    
    /* Change to idle task stack */
    system_stack_pointer = WREG15; /* This could be used to return from AuK */
    WREG15 = TCB_Descriptor[current_TCB_indx].stack_pointer;
    adr_current_task_stack_pointer = 
            (unsigned int) &TCB_Descriptor[current_TCB_indx].stack_pointer;
    
    /* Activate timer 1 interrupt */
    IEC0bits.T1IE = 1; /* Enable T1 interrupts */
    T1CONbits.TON = 1; /* Start timer */
    
    while(1); /* Idle task code */
    
    /* In case it should be necessary to return from AuK. 
     * 
     * WREG15 = system_stack_pointer;  
     * 
     */
    
}


/* Semaphores (Brinch Hansen inspired but including priorities) */
/*
   This function makes a TEST and SET operation on the file register 
   addressed by its first parameter (W0). Test and set operation is ever
   produced in bit 0 of the file register pointed by the first parameter.
   If this bit was 0 (non locked) the function
   replays with a 0 in its second parameter (W1), otherwise it replays with 
   a 1.
   The result of calling this function is set to 1 (locked in 
   any circumstance) the bit 0 of file register pointed by the first 
   parameter.
    
   ¡¡¡ATTENTION!!!
   TEST and SET operation is executed in only one instruction cycle.
   Therefore, it is uninterrumpible, hence it is impossible for race
   conditions to appear in relation to the bit in which TEST and SET 
   operates.
    
 */
extern void TSlock(int address_of_lock_variable, int address_of_answer);

/* This function writes a 0 on the less significant bit of a file register
   used as a lock with a test a set instruction.
   This action should be interpreted as unlock a resource.
   The file register used as a lock is passed as a parameter (W0).
 */
extern void TSunlock(int address_of_lock_variable);

/* The semaphore is initialized and its counter is set with the value of
 the second parameter. */
void init_semaphore(Tsemaphore *sem, int init_count)
{
    sem->semaphore_counter = init_count;
    sem->in = 0;
    sem->out = 0;
    sem->blocked_counter = 0;
}

/* It decerements the counter of the semaphore. If the counter value was 
 * already zero, the task is blocked and the counter remains at zero. 
 * Tasks blocked in the semaphore are ordered by priority and FIFO 
 * within prioiries. */
void wait(Tsemaphore *sem)
{
    unsigned int answer; // to be used in TSlock
    int x;
    int dir_last, dir_last_1;
    int temp;
    
    do
    {
        TSlock((unsigned int)&sem->lock, (unsigned int)&answer);
        if(answer == 1)
            if((PR1 - TMR1) > 5)
            {
                // Change context to benefit other task
                IFS1bits.INT2IF = 1;
                Nop();
                Nop();
            }
    }
    while(answer == 1);
    
    if(sem->semaphore_counter > 0)
    {
        sem->semaphore_counter--;
        TSunlock((unsigned int)&sem->lock);
    }
    else
    {        
        sem->BlockedTasksList[sem->in] = current_TCB_indx;
        sem->in = (sem->in + 1) % max_TCB_indx;
        sem->blocked_counter++;
                
        // Task must be ordered by priority and FIFO within priorities.
        
        /* The last blocked process enters at the end of the queue. 
         * Comparing with all the previous ones and reordering in each case 
         * if necessary, the queue is sorted by priority and FIFO within 
         * each priority. */
        
        
        if(sem->blocked_counter > 1)
        {
            for(x=sem->blocked_counter -1; x > 0; x--)
            {
                dir_last = (sem->out + x) % max_TCB_indx;
                dir_last_1 = (sem->out + x -1) % max_TCB_indx;               
                
                if(TCB_Descriptor[sem->BlockedTasksList[dir_last]].priority >
                   TCB_Descriptor[sem->BlockedTasksList[dir_last_1]].priority)
                {
                    // Swap components
                    temp = sem->BlockedTasksList[dir_last_1];
                    sem->BlockedTasksList[dir_last_1] = 
                            sem->BlockedTasksList[dir_last];
                    sem->BlockedTasksList[dir_last] = temp;
                }    
            }
        }
        
        /* These two critical instructions must be executed without 
         * the possibility of a context switch while they have not 
         * been completed. */
        
        IEC0bits.T1IE = 0; /* Disable T1 interrupts */
        TCB_Descriptor[current_TCB_indx].state = blocked;        
        TSunlock((unsigned int)&sem->lock);
        IEC0bits.T1IE = 1; /* Enable T1 interrupts */
        
        // Change context
        IFS1bits.INT2IF = 1;
        Nop();
        Nop();
    }
}

/* If there are no blocked tasks in the semaphore, signal increments the 
 semaphore counter. If there exist almost a blocked Task, signal unblock
 the first task in the semaphore blocked task queue. */
void signal(Tsemaphore *sem)
{
    unsigned int answer; // to be used in RClock
    
    do
    {
        TSlock((unsigned int)&sem->lock, (unsigned int)&answer);
        if(answer == 1)
            if((PR1 - TMR1) > 5)
            {
                // Change context to allow other task to change lock
                IFS1bits.INT2IF = 1;
                Nop();
                Nop();
            }
    }
    while(answer == 1);
    
    if(sem->semaphore_counter > 0)
    {
        sem->semaphore_counter++;
        TSunlock((unsigned int)&sem->lock);
    }
    else
    {   //semaphore_counter == 0
        if(sem->blocked_counter == 0)
        {
            sem->semaphore_counter++;
            TSunlock((unsigned int)&sem->lock);
        }
        else
        {
            TCB_Descriptor[sem->BlockedTasksList[sem->out]].state = ready;
            sem->out = (sem->out + 1) % max_TCB_indx;
            sem->blocked_counter--;
            TSunlock((unsigned int)&sem->lock);
        }
    }
}

/* Mutex */
/* Mutex are objects of exclusive access by tasks, with priority according 
 * to the immediate priority ceiling real time scheduler protocol */

/* Initialize a mutex and assigns to it the priority provided in the 
 * second parameter. */
void mutex_init (Tmutex *new_mutex, int priority)
{
    //new_mutex->sem.semaphore_counter = 1;
    init_semaphore(&new_mutex->sem, 1);
    new_mutex->mutex_priority = priority;
    new_mutex->task_old_priority = 0;
    new_mutex->holder_task = max_TCB_indx; /* Assigned to idle task */
}

/* If the mutex is locked by other task, the calling task is blocked in a 
 * semaphore local to the mutex (according to its current priority).
 * If the mutex is free, then it is blocked and the task adquires de priority 
 * of the mutex if it is higher than its current priority. */
void mutex_lock(Tmutex *mutex)
{
    wait(&mutex->sem);
    
    mutex->holder_task = current_TCB_indx;
    if(mutex->mutex_priority > TCB_Descriptor[current_TCB_indx].priority)
    {
        mutex->task_old_priority = TCB_Descriptor[current_TCB_indx].priority;
        TCB_Descriptor[current_TCB_indx].priority = mutex->mutex_priority;
    }
}

/* The mutex is unlocked and the task adquires its previous priority. 
 * If there exists blocked tasks in the same mutex, there will awaken one 
 * of them according to their priorities and arrival order within priorities. 
 *
 * It is no allowed to unlock a mutex by a task that no has this mutex locked.*/
void mutex_unlock(Tmutex *mutex)
{
    /* Unlock only allowed to the holder task */
    if(mutex->holder_task == current_TCB_indx)
    {
        mutex->holder_task = max_TCB_indx; /* Assigned to idle task */
    
        if(mutex->task_old_priority > 0)
        {
            TCB_Descriptor[current_TCB_indx].priority = mutex->task_old_priority;
            mutex->task_old_priority = 0;
        }

        signal(&mutex->sem);    
    }
}

/* Time services */

/* Returns current tic (see init_tmr1 in code) */
unsigned long clock()
{
    return current_tic;
}

/* Blocks current task until global tic.  */
void delay_until(unsigned long tic)
{
    int temp_IPL;
    unsigned long tics_to_wake_up;
    
    tics_to_wake_up = tic - current_tic;
    
    temp_IPL =  SRbits.IPL;

    /* Scheduler critical region */
    SRbits.IPL = 7;
    TCB_Descriptor[current_TCB_indx].tics_to_wake_up = tics_to_wake_up;
    TCB_Descriptor[current_TCB_indx].delayed = TRUE;
    TCB_Descriptor[current_TCB_indx].state = blocked;
    SRbits.IPL = temp_IPL;

    // Change context
    IFS1bits.INT2IF = 1;
    Nop();
    Nop();
    
}
/*-----------------------------------------------------------------
 End of AuK variables and routines
 ------------------------------------------------------------------*/


/*******************************************************/
/*******************************************************/
/*******************************************************/
/* ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ ATTENTION !!!!!!!!!!!!!!!!!!!!!!! */

/* This set of routines is provided to facilitate the development 
 * of more abstract tools than those currently provided in the microkernel, 
 * but is not recommended to use them in ordinary systems.*/


/* This routine block current active task. */
void block_me(void)
{
    TCB_Descriptor[current_TCB_indx].state = blocked;
    
    // Change context
    IFS1bits.INT2IF = 1;
    Nop();
    Nop();
}

/* This routine returns whether task_id is blocked. */
int is_blocked(int task_id)
{
    if(TCB_Descriptor[task_id].state == blocked)
        return(TRUE);
    else
        return(FALSE);
}

/* This routine unblock task_id. */
void unblock(int task_id)
{
    TCB_Descriptor[task_id].state = ready;
}

/*******************************************************/
/*******************************************************/
/*******************************************************/

// Array que representa los tenedores de los filósofos.
// 0 -> tenedor libre (sin usar)
// 1 -> tenedor ocupado (usado))
Tsemaphore semaforos [6];

void proceso(void)
{
    while(1)
    {
        meditar();
        comer();
        dormir();
    }
}


void meditar(void)
{
    int id = give_me_my_id();
    
    wait(&semaforos[5]);
    printf("\r\nMeditando %d", id);
    signal(&semaforos[5]);
    
}

void comer(void)
{
    int id = give_me_my_id();
    
    cogerTenedor();
    
    wait(&semaforos[5]);
    printf("\r\nComiendo %d", id);
    signal(&semaforos[5]);
    
    soltarTenedor();
    
}

void dormir(void)
{
    int id = give_me_my_id();
    
    wait(&semaforos[5]);
    printf("\r\nDurmiendo %d", id);
    signal(&semaforos[5]);
}


void cogerTenedor(void)
{
    int tenedorA = give_me_my_id();
    int tenedorB = getIdTenedorB (); 
    

    if (tenedorA < tenedorB){
        wait(&semaforos[tenedorA]);
        wait(&semaforos[tenedorB]);
    }else{
        wait(&semaforos[tenedorB]);
        wait(&semaforos[tenedorA]);
    }
    
}

int getIdTenedorB(void){
    int myId = give_me_my_id();
    int tenedorId;
    if(myId == 4){
        tenedorId = 0;
    }else {
        tenedorId = myId + 1;
    }
    
    return tenedorId;
}

void soltarTenedor(void)
{
    int tenedorA = give_me_my_id();
    int tenedorB = getIdTenedorB (); 
    
        if (tenedorA < tenedorB){
        signal(&semaforos[tenedorA]);
        signal(&semaforos[tenedorB]);
    }else{
        signal(&semaforos[tenedorB]);
        signal(&semaforos[tenedorA]);
    }
}

void inizializarSemaforos(void){
    for(int i=0; i<6; i++){ 
        init_semaphore(&semaforos[i],1);
    }
}

int main(void)
{
    init_micro();
    init_ports();
    init_uart();
    
    init_AuK(39.6288E+6,0.01);
    
    inizializarSemaforos();
            
    for(int i=0; i<5; i++) 
    {
        create_task(__builtin_tblpage(proceso),
                __builtin_tbloffset(proceso),300,1);
    }
    
    start_AuK();
    Sleep();
    return(0);
}
 

