    ;EXPORT  OSStartHighRdy  
    ;EXPORT  OSCtxSw
    ;EXPORT  OSIntCtxSw

    ;EXTERN  OSRunning
    ;EXTERN  OSPrioCur
    ;EXTERN  OSPrioHighRdy
    ;EXTERN  OSTCBCur
    ;EXTERN  OSTCBHighRdy
    ;EXTERN  OSIntExit
    ;EXTERN  OSTaskSwHook
    ;;EXTERN  OS_CPU_ExceptStkBase
    ;;EXTERN  OS_KA_BASEPRI_Boundary
;OS_CPU_ExceptStkBase		EQU	0x00	
;OS_KA_BASEPRI_Boundary		EQU 0x00
;NVIC_INT_CTRL   EQU     0xE000ED04                              
;NVIC_SYSPRI14   EQU     0xE000ED22                              
;NVIC_PENDSV_PRI EQU           0xFF                             
;NVIC_PENDSVSET  EQU     0x10000000  	
	

	;AREA Init, CODE, READONLY

;OSStartHighRdy
    ;CPSID   I                                                   
    ;LDR     R0, =NVIC_SYSPRI14                                 
    ;LDR     R1, =NVIC_PENDSV_PRI
    ;STRB    R1, [R0]

    ;MOVS    R0, #0                                             
    ;MSR     PSP, R0

    ;LDR     R0, =OS_CPU_ExceptStkBase                           
    ;LDR     R1, [R0]
    ;MSR     MSP, R1

    ;BL      OSTaskSwHook                                        

    ;LDR     R0, =OSRunning                                     
    ;MOVS    R1, #1
    ;STRB    R1, [R0]

    ;LDR     R0, =OSPrioCur                                      
    ;LDR     R1, =OSPrioHighRdy
    ;LDRB    R2, [R1]
    ;STRB    R2, [R0]

    ;LDR     R0, =OSTCBCur                                      
    ;LDR     R1, =OSTCBHighRdy
    ;LDR     R2, [R1]
    ;STR     R2, [R0]

    ;LDR     R0, [R2]                                          
    ;MSR     PSP, R0                                            

    ;MRS     R0, CONTROL
    ;ORR     R0, R0, #2
    ;MSR     CONTROL, R0
    ;ISB                                                        

    ;LDMFD    SP!, {R4-R11, LR}                                  
    ;LDMFD    SP!, {R0-R3}                                      
    ;LDMFD    SP!, {R12, LR}                                     
    ;LDMFD    SP!, {R1, R2}                                      
    ;CPSIE    I
    ;BX       R1
	
	
;OSCtxSw
;OSIntCtxSw
    ;LDR     R0, =NVIC_INT_CTRL                                  
    ;LDR     R1, =NVIC_PENDSVSET
    ;STR     R1, [R0]
    ;BX      LR
	
	
;OS_CPU_PendSVHandler
    ;CPSID   I                                                   
    ;MOV32   R2, OS_KA_BASEPRI_Boundary                         
    ;LDR     R1, [R2]
    ;MSR     BASEPRI, R1
    ;DSB
    ;ISB
    ;CPSIE   I

    ;MRS     R0, PSP                                            
    ;STMFD   R0!, {R4-R11, R14}                                  

    ;LDR     R5, =OSTCBCur                                      
    ;LDR     R1, [R5]
    ;STR     R0, [R1]                                           

                                                                
    ;MOV     R4, LR                                             
    ;BL      OSTaskSwHook                                        

    ;LDR     R0, =OSPrioCur                                      
    ;LDR     R1, =OSPrioHighRdy
    ;LDRB    R2, [R1]
    ;STRB    R2, [R0]

    ;LDR     R1, =OSTCBHighRdy                                   
    ;LDR     R2, [R1]
    ;STR     R2, [R5]

    ;ORR     LR,  R4, #0x04                                     
    ;LDR     R0,  [R2]                                           
    ;LDMFD   R0!, {R4-R11, R14}                                
    ;MSR     PSP, R0                                             

    ;MOV32   R2, #0                                            
    ;CPSID   I
    ;MSR     BASEPRI, R2
    ;DSB
    ;ISB
    ;CPSIE   I
    ;BX      LR                                                 

    ;ALIGN                                                      

    END
