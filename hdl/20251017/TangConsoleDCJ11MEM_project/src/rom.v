//---------------------------------------------------------------
// Bootstrap Loader for Hard Disk and Tape
//---------------------------------------------------------------
// rom.v
// to be included from the top module at the compile

//---------------------------------------------------------------
// This ROM contains several boot programs
// - Magnetic Tape boot loader to install unix
// - Unix v1 boot ROM
// - BM873-YA restart/loader
// - Bootstrap Loader for paper tape
//
// Boot program entry addresses
//
// 100 000: TM11
// 157 744: PC11 (load absolute loader)
// 773 000: RF11 (not for unix v1, use 773700)
// 773 010: RK11 (unix v6, RT-11)
// 773 050: TM11
// 773 100: RP11
// 773 312: PC11
// 773 700: Unix V1 (RF)
//---------------------------------------------------------------

`define MEM(x, y) {mem_hi[(x)>>1], mem_lo[(x)>>1]}=y
`define ROM(x, y) rom[x>>1]=y

initial
begin
//---------------------------------------------------------------
// SWR (Console Switches Register, 777570)
//---------------------------------------------------------------
REG_SWR = 'o173700; // UNIX V1 multi user mode
//REG_SWR = 'o073700; // UNIX V1 single user mode
//REG_SWR = 'o173030; // UNIX V6 single user mode

//---------------------------------------------------------------
// Bootstrap Loader for magnetic tape TM11/TU10
//---------------------------------------------------------------
`MEM('o100000,'o012700);// mov #172526, r0     ; r0=MTBRC+2
`MEM('o100002,'o172526);//                     ;
`MEM('o100004,'o012701);// mov #-^D512, r1     ;
`MEM('o100006,'o177000);//                     ;
`MEM('o100010,'o010140);// mov r1, -(r0)       ; MTBRC=-512
`MEM('o100012,'o012740);// mov #READ+GO, -(r0) ; MTC=READ+GO;
`MEM('o100014,'o060003);//                     ;
`MEM('o100016,'o005000);// clr r0              ; unit number
`MEM('o100020,'o012701);// mov #172522, r1     ; CSR addr of TM11
`MEM('o100022,'o172522);//
`MEM('o100024,'o000000);// halt                ; halt
//`MEM('o100024,'o000777);// br .                ; jump to .
//`MEM('o100024,'o005007);// clr pc              ; jump to 0

// set 040 to SWR
`MEM('o100040,'o012737); // mov #000040, @#777570
`MEM('o100042,'o000040); // 
`MEM('o100044,'o177570); // 
`MEM('o100046,'o000000); // halt
// set 045 to SWR
`MEM('o100050,'o012737); // mov #000045, @#777570
`MEM('o100052,'o000045); // 
`MEM('o100054,'o177570); // 
`MEM('o100056,'o000000); // halt
// set 070 to SWR
`MEM('o100070,'o012737); // mov #000070, @#777570
`MEM('o100072,'o000070); // 
`MEM('o100074,'o177570); // 
`MEM('o100076,'o000000); // halt
   
//---------------------------------------------------------------
// boot loader for RF disk (UNIX V1)
//---------------------------------------------------------------
`ROM('o3700,'o012700); // mov #177472,r0
`ROM('o3702,'o177472);
`ROM('o3704,'o012740); // mov #3,-(r0)     // DAE=3
`ROM('o3706,'o000003);
`ROM('o3710,'o012740); // mov #14000,-(r0) // DAR=14000
`ROM('o3712,'o140000);
`ROM('o3714,'o012740); // mov #54000,-(r0) // CMA=54000
`ROM('o3716,'o054000);
`ROM('o3720,'o012740); // mov #-2000,-(r0) // WC=-2000
`ROM('o3722,'o176000);
`ROM('o3724,'o012740); // mov #5, -(r0)    // DCS=5
`ROM('o3726,'o000005);
`ROM('o3730,'o105710); // tstb (r0)
`ROM('o3732,'o002376); // bge .-2
`ROM('o3734,'o000137); // jmp @#54000
`ROM('o3736,'o054000);

//---------------------------------------------------------------
// BM873-YA restart/loader
//---------------------------------------------------------------
`ROM('o3000,'o010702);//   RF11: mov pc,r2 ;
`ROM('o3002,'o000464);//         br DOSIMP ; 
`ROM('o3004,'o177462);//                   ; RFWC
`ROM('o3006,'o000005);//                   ; READ+GO
`ROM('o3010,'o010702);//   RK11: mov pc,r2 ;
`ROM('o3012,'o000460);//         br DOSIMP ;
`ROM('o3014,'o177406);//                   ; RKWC
`ROM('o3016,'o000005);//                   ; READ+GO
`ROM('o3020,'o013707);//  CONSW: mov @#177570, pc ; jump to comsole register
`ROM('o3022,'o177570);//                   ;
`ROM('o3024,'o173776);//  vector
`ROM('o3026,'o000340);//
`ROM('o3030,'o010702);//   TC11: mov pc,r2 ;
`ROM('o3032,'o000426);//         br DOEXT  ;
`ROM('o3034,'o177344);//                   ; TCWC
`ROM('o3036,'o004003);//
`ROM('o3040,'o100000);//
`ROM('o3042,'o024000);//
`ROM('o3044,'o000445);//
`ROM('o3046,'o000005);//
`ROM('o3050,'o010702);//   TM11: mov pc,r2 ;
`ROM('o3052,'o000416);//         br DOEXT  ;
`ROM('o3054,'o172524);//                   ; TMBCR
`ROM('o3056,'o060017);//
`ROM('o3060,'o000200);//
`ROM('o3062,'o100000);//
`ROM('o3064,'o000413);//
`ROM('o3066,'o060011);//
`ROM('o3070,'o000200);//
`ROM('o3072,'o100000);//
`ROM('o3074,'o000431);//
`ROM('o3076,'o060003);//
`ROM('o3100,'o010702);//   RP11: mov pc,r2 ;
`ROM('o3102,'o000424);//         br DOSIMP ;
`ROM('o3104,'o176716);//                   ; RPWC
`ROM('o3106,'o000005);//                   ; READ+GO
`ROM('o3110,'o010200);//  DOEXT: mov r2,r0
`ROM('o3112,'o005720);//         tst (r0)+
`ROM('o3114,'o000005);//         reset
`ROM('o3116,'o005720);//         tst (r0)+
`ROM('o3120,'o016201);//         mov 2(r2),r1
`ROM('o3122,'o000002);//
`ROM('o3124,'o005311);//         dec (r1)
`ROM('o3126,'o012041);//         mov (r0)+,-(r1)
`ROM('o3130,'o031011);//  1$:    bit (r0),(r1)
`ROM('o3132,'o001776);//         beq 1$
`ROM('o3134,'o005720);//         TST (r0)+
`ROM('o3136,'o032041);//         bit (r0)+,-(r1)
`ROM('o3140,'o001063);//         bne XXX
`ROM('o3142,'o000110);//         jmp (r0)
`ROM('o3144,'o010702);//   RC11: mov pc,r2
`ROM('o3146,'o000402);//         br DOSIMP
`ROM('o3150,'o177450);//
`ROM('o3152,'o000005);//
`ROM('o3154,'o010200);// DOSIMP: mov r2,r0    ; Get command pointer
`ROM('o3156,'o005720);//         tst(r0)+
`ROM('o3160,'o005720);//         tst(r0)+
`ROM('o3162,'o000005);//         reset        ; Also clears device BAR
`ROM('o3164,'o016201);//         mov 2(r2),r1 ; Get addr of WC reg
`ROM('o3166,'o000002);//
`ROM('o3170,'o012711);//         mov #-1000,(r1) ; Set word count
`ROM('o3172,'o177000);//
`ROM('o3174,'o011041);//         mov (r0),-(r1)  ; Command
`ROM('o3176,'o105711);//     1$: tstb (r1)       ; Wait for done
`ROM('o3200,'o100376);//         bpl 1$
`ROM('o3202,'o005711);//         tst (r1)        ; Check error bit
`ROM('o3204,'o100441);//         bmi XXX         ; error
`ROM('o3206,'o005007);//         clr pc          ; jump  to 0
`ROM('o3210,'o012704);//   KL11: mov #177560,r4  ;
`ROM('o3212,'o177560);//
`ROM('o3214,'o000440);//         br RDBOOT
`ROM('o3216,'o017640);//
`ROM('o3220,'o002415);//
`ROM('o3222,'o112024);//
`ROM('o3224,'o173776);//   vector
`ROM('o3226,'o000340);//
`ROM('o3230,'o005004);//   TA11: clr r4
`ROM('o3232,'o012700);//         mov #177500,r0
`ROM('o3234,'o177500);//
`ROM('o3236,'o000005);//  RETRY: reset
`ROM('o3240,'o010410);//         mov r4,(r0)
`ROM('o3242,'o012701);//         mov #173216,r1
`ROM('o3244,'o173216);//
`ROM('o3246,'o012702);//         mov #375,r2
`ROM('o3250,'o000375);//
`ROM('o3252,'o112103);//         movb (r1)+,r3
`ROM('o3254,'o112110);//     XX: movb (r1)+,(r0)
`ROM('o3256,'o100407);//         bmi 1$
`ROM('o3260,'o130310);//     2$: bitb r3,(r0)
`ROM('o3262,'o001776);//         beq 2$
`ROM('o3264,'o105202);//         incb r2
`ROM('o3266,'o100772);//         bmi XX
`ROM('o3270,'o116012);//         movb 2(r0),(r2)
`ROM('o3272,'o000002);//
`ROM('o3274,'o000771);//         br 2$
`ROM('o3276,'o005710);//     1$: tst (r0)
`ROM('o3300,'o100756);//         bmi RETRY
`ROM('o3302,'o005002);//         clr r2
`ROM('o3304,'o120312);//         cmpb r3,(r2)
`ROM('o3306,'o001377);//         bne .
`ROM('o3310,'o000112);//    XXX: jmp (r2)
`ROM('o3312,'o012704);//   PC11: mov #177550,r4
`ROM('o3314,'o177550);//
`ROM('o3316,'o000005);// RDBOOT: reset
`ROM('o3320,'o012701);//         mov #160000,r1
`ROM('o3322,'o160000);//
`ROM('o3324,'o012702);//         mov #6,r2
`ROM('o3326,'o000006);//
`ROM('o3330,'o012712);//         mov #340,(r2)
`ROM('o3332,'o000340);//
`ROM('o3334,'o010742);//         mov pc,-(r2)
`ROM('o3336,'o012706);//         mov #24,sp
`ROM('o3340,'o000024);//
`ROM('o3342,'o010441);//         mov r4,-(r1)
`ROM('o3344,'o040601);//         bic sp,r1
`ROM('o3346,'o010111);//         mov r1,(r1)
`ROM('o3350,'o011102);//     2$: mov (r1),r2
`ROM('o3352,'o005214);//         inc (r4)
`ROM('o3354,'o105714);//     1$: tstb (r4)
`ROM('o3356,'o100376);//         bpl 1$
`ROM('o3360,'o116412);//         movb 2(r4),(r2)
`ROM('o3362,'o000002);//
`ROM('o3364,'o005211);//         inc (r1)
`ROM('o3366,'o120227);//         cmpb r2,#375
`ROM('o3370,'o000375);//
`ROM('o3372,'o001366);//         bne 2$
`ROM('o3374,'o105222);//         incb (r2)+
`ROM('o3376,'o000142);//         jmp -(r2)
   
//---------------------------------------------------------------
// Bootstrap Loader for paper tape
//
//  LOAD=xx7400         ; Buffer start address
// .=LOAD+0344          ; Start address of bootstrap loader (xx7744)
//---------------------------------------------------------------
                          // START:
`MEM('o157744,'o016701);// MOV DEVICE, R1    ; Get reader CSR address
`MEM('o157746,'o000026);//                   ; 157750+000026=157776
                        // LOOP:
`MEM('o157750,'o012702);// MOV #.-LOAD+2, R2 ; Get buffer pointer
`MEM('o157752,'o000352);//                   ; (<--- pointer to buffer)
`MEM('o157754,'o005211);// INC @R1           ; Enable the paper tape reader
                        // WAIT:
`MEM('o157756,'o105711);// TSTB @R1          ; Wait until data available
`MEM('o157760,'o100376);// BPL WAIT
`MEM('o157762,'o116162);// MOVB 2(R1), LOAD(R2) ;Transfer byte to buffer
`MEM('o157764,'o000002);//
`MEM('o157766,'o157400);//  xx7400
`MEM('o157770,'o005267);//  INC LOOP+2       ; Increment pointer to buffer
`MEM('o157772,'o177756);//
`MEM('o157774,'o000765);//  BR LOOP          ; Continue reading
                        //  (modified branch instruction)
`MEM('o157776,'o177550);//  DEVICE:        ; Paper tape reader CSR address

end
