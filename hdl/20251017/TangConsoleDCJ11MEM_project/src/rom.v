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
`MEM('o100000,'o012700);// mov #172526, r0     ; r0 = MTBRC+2
`MEM('o100002,'o172526);//                     ;
`MEM('o100004,'o010040);// mov r0, -(r0)       ; MTBRC=-05252
`MEM('o100006,'o012740);// mov #READ+GO, -(r0) ; MTC  =READ+GO;
`MEM('o100010,'o060003);//                     ;
`MEM('o100012,'o000000);// halt                ; halt
//`MEM('o100012,'o000777);// br .                ; jump to .
//`MEM('o100012,'o005007);// clr pc              ; jump to 0

// set 040 to SWR
`MEM('o100040,'o012737); // mov #000040, @#777570
`MEM('o100042,'o000040); // 
`MEM('o100044,'o177570); // 
`MEM('o100046,'o000000); // halt
// set 040 to SWR
`MEM('o100050,'o012737); // mov #000050, @#777570
`MEM('o100052,'o000050); // 
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
`MEM('o773700,'o012700); // mov #177472,r0
`MEM('o773702,'o177472);
`MEM('o773704,'o012740); // mov #3,-(r0)     // DAE=3
`MEM('o773706,'o000003);
`MEM('o773710,'o012740); // mov #14000,-(r0) // DAR=14000
`MEM('o773712,'o140000);
`MEM('o773714,'o012740); // mov #54000,-(r0) // CMA=54000
`MEM('o773716,'o054000);
`MEM('o773720,'o012740); // mov #-2000,-(r0) // WC=-2000
`MEM('o773722,'o176000);
`MEM('o773724,'o012740); // mov #5, -(r0)    // DCS=5
`MEM('o773726,'o000005);
`MEM('o773730,'o105710); // tstb (r0)
`MEM('o773732,'o002376); // bge .-2
`MEM('o773734,'o000137); // jmp @#54000
`MEM('o773736,'o054000);

//---------------------------------------------------------------
// BM873-YA restart/loader
//---------------------------------------------------------------
`MEM('o773000,'o010702);//   RF11: mov pc,r2 ;
`MEM('o773002,'o000464);//         br DOSIMP ; 
`MEM('o773004,'o177462);//                   ; RFWC
`MEM('o773006,'o000005);//                   ; READ+GO
`MEM('o773010,'o010702);//   RK11: mov pc,r2 ;
`MEM('o773012,'o000460);//         br DOSIMP ;
`MEM('o773014,'o177406);//                   ; RKWC
`MEM('o773016,'o000005);//                   ; READ+GO
`MEM('o773020,'o013707);//  CONSW: mov @#177570, pc ; jump to comsole register
`MEM('o773022,'o177570);//                   ;
`MEM('o773024,'o173776);//  vector
`MEM('o773026,'o000340);//
`MEM('o773030,'o010702);//   TC11: mov pc,r2 ;
`MEM('o773032,'o000426);//         br DOEXT  ;
`MEM('o773034,'o177344);//                   ; TCWC
`MEM('o773036,'o004003);//
`MEM('o773040,'o100000);//
`MEM('o773042,'o024000);//
`MEM('o773044,'o000445);//
`MEM('o773046,'o000005);//
`MEM('o773050,'o010702);//   TM11: mov pc,r2 ;
`MEM('o773052,'o000416);//         br DOEXT  ;
`MEM('o773054,'o172524);//                   ; TMBCR
`MEM('o773056,'o060017);//
`MEM('o773060,'o000200);//
`MEM('o773062,'o100000);//
`MEM('o773064,'o000413);//
`MEM('o773066,'o060011);//
`MEM('o773070,'o000200);//
`MEM('o773072,'o100000);//
`MEM('o773074,'o000431);//
`MEM('o773076,'o060003);//
`MEM('o773100,'o010702);//   RP11: mov pc,r2 ;
`MEM('o773102,'o000424);//         br DOSIMP ;
`MEM('o773104,'o176716);//                   ; RPWC
`MEM('o773106,'o000005);//                   ; READ+GO
`MEM('o773110,'o010200);//  DOEXT: mov r2,r0
`MEM('o773112,'o005720);//         tst (r0)+
`MEM('o773114,'o000005);//         reset
`MEM('o773116,'o005720);//         tst (r0)+
`MEM('o773120,'o016201);//         mov 2(r2),r1
`MEM('o773122,'o000002);//
`MEM('o773124,'o005311);//         dec (r1)
`MEM('o773126,'o012041);//         mov (r0)+,-(r1)
`MEM('o773130,'o031011);//  1$:    bit (r0),(r1)
`MEM('o773132,'o001776);//         beq 1$
`MEM('o773134,'o005720);//         TST (r0)+
`MEM('o773136,'o032041);//         bit (r0)+,-(r1)
`MEM('o773140,'o001063);//         bne XXX
`MEM('o773142,'o000110);//         jmp (r0)
`MEM('o773144,'o010702);//   RC11: mov pc,r2
`MEM('o773146,'o000402);//         br DOSIMP
`MEM('o773150,'o177450);//
`MEM('o773152,'o000005);//
`MEM('o773154,'o010200);// DOSIMP: mov r2,r0    ; Get command pointer
`MEM('o773156,'o005720);//         tst(r0)+
`MEM('o773160,'o005720);//         tst(r0)+
`MEM('o773162,'o000005);//         reset        ; Also clears device BAR
`MEM('o773164,'o016201);//         mov 2(r2),r1 ; Get addr of WC reg
`MEM('o773166,'o000002);//
`MEM('o773170,'o012711);//         mov #-1000,(r1) ; Set word count
`MEM('o773172,'o177000);//
`MEM('o773174,'o011041);//         mov (r0),-(r1)  ; Command
`MEM('o773176,'o105711);//     1$: tstb (r1)       ; Wait for done
`MEM('o773200,'o100376);//         bpl 1$
`MEM('o773202,'o005711);//         tst (r1)        ; Check error bit
`MEM('o773204,'o100441);//         bmi XXX         ; error
`MEM('o773206,'o005007);//         clr pc          ; jump  to 0
`MEM('o773210,'o012704);//   KL11: mov #177560,r4  ;
`MEM('o773212,'o177560);//
`MEM('o773214,'o000440);//         br RDBOOT
`MEM('o773216,'o017640);//
`MEM('o773220,'o002415);//
`MEM('o773222,'o112024);//
`MEM('o773224,'o173776);//   vector
`MEM('o773226,'o000340);//
`MEM('o773230,'o005004);//   TA11: clr r4
`MEM('o773232,'o012700);//         mov #177500,r0
`MEM('o773234,'o177500);//
`MEM('o773236,'o000005);//  RETRY: reset
`MEM('o773240,'o010410);//         mov r4,(r0)
`MEM('o773242,'o012701);//         mov #173216,r1
`MEM('o773244,'o173216);//
`MEM('o773246,'o012702);//         mov #375,r2
`MEM('o773250,'o000375);//
`MEM('o773252,'o112103);//         movb (r1)+,r3
`MEM('o773254,'o112110);//     XX: movb (r1)+,(r0)
`MEM('o773256,'o100407);//         bmi 1$
`MEM('o773260,'o130310);//     2$: bitb r3,(r0)
`MEM('o773262,'o001776);//         beq 2$
`MEM('o773264,'o105202);//         incb r2
`MEM('o773266,'o100772);//         bmi XX
`MEM('o773270,'o116012);//         movb 2(r0),(r2)
`MEM('o773272,'o000002);//
`MEM('o773274,'o000771);//         br 2$
`MEM('o773276,'o005710);//     1$: tst (r0)
`MEM('o773300,'o100756);//         bmi RETRY
`MEM('o773302,'o005002);//         clr r2
`MEM('o773304,'o120312);//         cmpb r3,(r2)
`MEM('o773306,'o001377);//         bne .
`MEM('o773310,'o000112);//    XXX: jmp (r2)
`MEM('o773312,'o012704);//   PC11: mov #177550,r4
`MEM('o773314,'o177550);//
`MEM('o773316,'o000005);// RDBOOT: reset
`MEM('o773320,'o012701);//         mov #160000,r1
`MEM('o773322,'o160000);//
`MEM('o773324,'o012702);//         mov #6,r2
`MEM('o773326,'o000006);//
`MEM('o773330,'o012712);//         mov #340,(r2)
`MEM('o773332,'o000340);//
`MEM('o773334,'o010742);//         mov pc,-(r2)
`MEM('o773336,'o012706);//         mov #24,sp
`MEM('o773340,'o000024);//
`MEM('o773342,'o010441);//         mov r4,-(r1)
`MEM('o773344,'o040601);//         bic sp,r1
`MEM('o773346,'o010111);//         mov r1,(r1)
`MEM('o773350,'o011102);//     2$: mov (r1),r2
`MEM('o773352,'o005214);//         inc (r4)
`MEM('o773354,'o105714);//     1$: tstb (r4)
`MEM('o773356,'o100376);//         bpl 1$
`MEM('o773360,'o116412);//         movb 2(r4),(r2)
`MEM('o773362,'o000002);//
`MEM('o773364,'o005211);//         inc (r1)
`MEM('o773366,'o120227);//         cmpb r2,#375
`MEM('o773370,'o000375);//
`MEM('o773372,'o001366);//         bne 2$
`MEM('o773374,'o105222);//         incb (r2)+
`MEM('o773376,'o000142);//         jmp -(r2)
   
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
