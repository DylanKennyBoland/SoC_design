#line 1 "main-CMSIS.c"















 
#line 1 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 18 "main-CMSIS.c"
#line 1 "ARMCM0.h"
 








 

























 










 

typedef enum IRQn
{
 
  NonMaskableInt_IRQn           = -14,       
  HardFault_IRQn                = -13,       



  SVCall_IRQn                   =  -5,       

  PendSV_IRQn                   =  -2,       
  SysTick_IRQn                  =  -1,       

 
  WDT_IRQn                      =   0,       
  UART_IRQn                     =   1,       
  TIM0_IRQn                     =   2,       
  TIM2_IRQn                     =   3,       
  MCIA_IRQn                     =   4,       
  MCIB_IRQn                     =   5,       
  UART0_IRQn                    =   6,       
  UART1_IRQn                    =   7,       
  UART2_IRQn                    =   8,       
  UART4_IRQn                    =   9,       
  AACI_IRQn                     =  10,       
  CLCD_IRQn                     =  11,       
  ENET_IRQn                     =  12,       
  USBDC_IRQn                    =  13,       
  USBHC_IRQn                    =  14,       
  CHLCD_IRQn                    =  15,       
  FLEXRAY_IRQn                  =  16,       
  CAN_IRQn                      =  17,       
  LIN_IRQn                      =  18,       
  I2C_IRQn                      =  19,       
  CPU_CLCD_IRQn                 =  28,       
  UART3_IRQn                    =  30,       
  SPI_IRQn                      =  31,       
} IRQn_Type;


 
 
 

 





#line 1 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"
 







 

























 
























 




 


 

 













#line 110 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"


 







#line 145 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "C:\\Keil_v4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 147 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 



#line 292 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmInstr.h"


#line 684 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 148 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 271 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 307 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 634 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 

#line 149 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"








 
#line 174 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"

 






 
#line 190 "C:\\Keil_v4\\ARM\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








#line 100 "ARMCM0.h"
#line 1 "system_ARMCM0.h"
 








 

























 









extern uint32_t SystemCoreClock;      










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 101 "ARMCM0.h"


 
 
 

 

  #pragma push
  #pragma anon_unions
#line 124 "ARMCM0.h"



 
 
 
typedef struct
{
  volatile const  uint32_t ID;                
  volatile uint32_t MEMCFG;            
  volatile const  uint32_t SW;                
  volatile uint32_t LED;               
  volatile const  uint32_t TS;                
  volatile uint32_t CTRL1;             
       uint32_t RESERVED0[2];
  volatile uint32_t CLKCFG;            
  volatile uint32_t WSCFG;             
  volatile uint32_t CPUCFG;            
       uint32_t RESERVED1[3];
  volatile uint32_t BASE;              
  volatile uint32_t ID2;               
} ARM_CPU_SYS_TypeDef;


 
 
 
typedef struct
{
  volatile const  uint32_t ID;                
  volatile uint32_t PERCFG;            
  volatile const  uint32_t SW;                
  volatile uint32_t LED;               
  volatile uint32_t SEG7;              
  volatile const  uint32_t CNT25MHz;          
  volatile const  uint32_t CNT100Hz;          
} ARM_DUT_SYS_TypeDef;


 
 
 
typedef struct
{
  volatile uint32_t Timer1Load;        
  volatile const  uint32_t Timer1Value;       
  volatile uint32_t Timer1Control;     
  volatile  uint32_t Timer1IntClr;      
  volatile const  uint32_t Timer1RIS;         
  volatile const  uint32_t Timer1MIS;         
  volatile uint32_t Timer1BGLoad;      
       uint32_t RESERVED0[1];
  volatile uint32_t Timer2Load;        
  volatile const  uint32_t Timer2Value;       
  volatile uint32_t Timer2Control;     
  volatile  uint32_t Timer2IntClr;      
  volatile const  uint32_t Timer2RIS;         
  volatile const  uint32_t Timer2MIS;         
  volatile uint32_t Timer2BGLoad;      
} ARM_TIM_TypeDef;


 
 
 
typedef struct
{
  volatile uint32_t DR;                
  union {
  volatile const  uint32_t RSR;               
  volatile  uint32_t ECR;               
  };
       uint32_t RESERVED0[4];
  volatile uint32_t FR;                
       uint32_t RESERVED1[1];
  volatile uint32_t ILPR;              
  volatile uint32_t IBRD;              
  volatile uint32_t FBRD;              
  volatile uint32_t LCR_H;             
  volatile uint32_t CR;                
  volatile uint32_t IFLS;              
  volatile uint32_t IMSC;              
  volatile uint32_t RIS;               
  volatile uint32_t MIS;               
  volatile  uint32_t ICR;               
  volatile uint32_t DMACR;             
} ARM_UART_TypeDef;


 

  #pragma pop
#line 229 "ARMCM0.h"




 
 
 
 








 





#line 258 "ARMCM0.h"


 
 
 
 



 
#line 275 "ARMCM0.h"






#line 19 "main-CMSIS.c"
#line 1 "DES_M0_SoC.h"



 

	




typedef unsigned       char uint8;
typedef   signed       char  int8;
typedef unsigned short int  uint16;
typedef   signed short int   int16;
typedef unsigned       int  uint32;
typedef   signed       int   int32;

#pragma anon_unions



typedef struct 
{
	union  
	{
		volatile uint8   RxData;		
		volatile uint32  reserved0; 
	};
	union  
	{
		volatile uint8   TxData;
		volatile uint32  reserved1;
	};
	union 
	{
		volatile uint8   Status;
		volatile uint32  reserved2;
	};
	union 
	{
		volatile uint8   Control;
		volatile uint32  reserved3;
	};
} UART_block;
















struct TwoByte		
{									
	uint8 Lo;
	uint8 Hi;
};

typedef struct 		
{
	union 					
	{
		volatile uint16  Out0;		
		struct TwoByte  OUT0;			
		volatile uint32  reserved0;		
	};
	union 
	{
		volatile uint16  Out1;
		struct TwoByte  OUT1;
		volatile uint32  reserved1;
	};
	union 
	{
		volatile uint16  In0;
		struct TwoByte  IN0;
		volatile uint32  reserved2;
	};
	union 
	{
		volatile uint16  In1;
		struct TwoByte  IN1;
		volatile uint32  reserved3;
	};
} GPIO_block;


typedef struct 		
{
	union 					
	{
		volatile uint8 spi_ctrl; 		
		volatile uint32  reserved0;		
	};
	union 
	{
		volatile uint8 spi_tx_data; 		
		volatile uint32  reserved1;		
	};
	union 
	{
		volatile uint8 spi_rx_data; 		
		volatile uint32  reserved2;		
	};
} SPI_block;


typedef struct
{
	volatile uint32 rawLow;
	volatile uint32 rawHigh;
	volatile uint32 hexData;
	volatile uint32 control;
	
} SEV_SEG_block;


#line 133 "DES_M0_SoC.h"






















typedef struct
{
	volatile uint32	CTRL;		
	volatile uint32	LOAD;			
	volatile uint32	VAL;			
} SysTick_block;















typedef struct {
	volatile uint32	SETENA;				
	volatile uint32	reserved1[0x20-1];  
	volatile uint32	CLRENA;				
} NVIC_block;









#line 196 "DES_M0_SoC.h"


#line 20 "main-CMSIS.c"















volatile uint8  RxBuf[100];	
volatile uint8  counter  = 0; 		
volatile uint8  BufReady = 0; 		





void UART_ISR()		
{
	char c;
	c = (((UART_block *)0x51000000)->RxData);	 				
	RxBuf[counter]  = c;  
	counter++;            
	(((UART_block *)0x51000000)->TxData) = c;  				

	


 
	if (counter == 100-1 || c == '\r')  
	{
		counter--;							
		RxBuf[counter] = 0;  
		BufReady       = 1;	    
	}
}





void SysTick_ISR()	
{
	(((GPIO_block *)0x50000000)->OUT0 . Hi) ^= 0x80;		
}







void delay (uint32 n) 
{
	volatile uint32 i;
		for(i=0; i<n; i++);		
}





int main(void) 
{
	uint8 i;		
	uint8 TxBuf[(sizeof(RxBuf)/sizeof(RxBuf[0]))];		
	uint32 timeStart, timeEnd;	
	
	delay(1000000);										
	
	printf("\nWelcome to Cortex-M0 SoC with CMSIS functions\n");			
	printf("CPU ID %x SysTick Calibration %x\n", ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CPUID, ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CALIB);

	
	(((UART_block *)0x51000000)->Control) = (1 << 3);		
  NVIC_EnableIRQ(UART_IRQn);													
	
	
	if ( SysTick_Config(0x1000000) )	
	{
		printf("Failed to configure SysTick timer\n");
	}

	
	timeStart = ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL;		
	delay( 1000 );				
	timeEnd = ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL;			
	printf("delay( %d ) used %d clock cycles\n", 1000, 
					(timeStart - timeEnd) & 0x00ffffff);  
	
	
	while(1){			
			
		
		(((GPIO_block *)0x50000000)->Out0)	= (((GPIO_block *)0x50000000)->In0); 			
		delay(1000000);				
		((((GPIO_block *)0x50000000)->OUT0 . Hi) ^= 0xff);						
		delay(1000000);				
		((((GPIO_block *)0x50000000)->OUT0 . Hi) ^= 0xff);						
		delay(1000000);				

		

 
		if ( (((GPIO_block *)0x50000000)->In1) & (0x08) ) 		
			((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL &= ~0x0002;	
		else 															
			((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL |= 0x0002;		
		
		
		printf("\nSysTick: Control %x Reload %x Current %x\n", 
								((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL, ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD, ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL);		
		printf("\nNVIC: Enable %x Pending %x Priority %x\n", 
								((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0], ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0], ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[0]);			

		
		printf("\nType some characters: ");
		
		while (BufReady == 0)		
		{			
			__wfi();  
			
			if ( counter > 0 )	
				(((GPIO_block *)0x50000000)->OUT0 . Lo) = RxBuf[counter-1];  
		}

		
 
		
		NVIC_DisableIRQ(UART_IRQn);		
		
		for (i=0; i<=counter; i++)		
		{
			if (RxBuf[i] >= 'A') {						
				TxBuf[i] = RxBuf[i] ^ ('A' ^ 'a'); 
			}
			else {
				TxBuf[i] = RxBuf[i];            
			}
		} 
		
		BufReady = 0;			
		counter  = 0; 		
		
		NVIC_EnableIRQ(UART_IRQn);		
		
		
		printf("\n:--> |%s|\n", TxBuf);  
		
	} 

}  


