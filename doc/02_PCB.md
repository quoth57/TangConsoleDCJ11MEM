# 基板 rev.2.0
![](../images/rev20.jpg)
- TangNano20K版とは違い，レベル変換ICをCPUボードに搭載してみました．(20K版の方でもCPUボード上に搭載した基板(rev3)を作りました．)
- CPUの入力信号(FPGA→CPU)はレベル変換せずに直接接続しています．
- ABORT_nはオープンコレクタなので，外付でそれ用の素子を用意しようかとも思いましたが，FPGA側で下記のように実装すれば良いようなのでそれで済ませました．
```
  assign ABORT_n = bus_error ? 1'b0 : 1'bz; // simulate open collector output
```
#### BOM
|Reference          |Qty| Value          |Size |Memo |
|-------------------|---|----------------|-----|-----|
|C1,C2              |2  |68pF            |||
|C3,C5,C6,C7,C8     |5  |0.1uF           |||
|C4                 |1  |1uF           |||
|C9,C10             |2	|0.33uF	         |||
|C11                |1  |47uF            |||
|D1                 |1  |LED             || |
|J1                 |1  |pin socket|2x20|TangConsole接続用．基板背面に実装．|
|J2,J3              |2  |pin header or socket|1x30|任意．テストや観測，実験用．|
|J4                 |1  |pin header      |1x06 L字|UART用|
|J5                 |1  |pin header or socket |2x03 |不要．将来使うかもしれない実験用．|
|R1                 |1  |1M              |||
|R2,R3,R4,R5,R6,R7,R8,R9,R10   |9  |10k            || |
|R11                |1  |33             ||CLK2(出力)のダンピング抵抗．|
|R12                |1  |100k            || 値はLEDに合わせて任意．|
|R13,R14            |2  |1k              |||
|SW1,SW2            |2  |tactile SW      |6mmxH4.3mm|例: https://akizukidenshi.com/catalog/g/g103647/ |
|U1                 |1  |DCJ11           |60pin DIP 1300mil| 1x30 の丸ピンソケット2列|
|U2,U3,U4,U5        |4  |SN74CB3T3245DWR (又はDW) |SOIC-20|https://mou.sr/3URN55f https://www.digikey.jp/short/9485r0f0  |
|U6                 |1  |NJM12888F33     |SOT-23-5 | https://akizukidenshi.com/catalog/g/g110675/|
|Y1                 |1  |18MHz           |HC49|例: https://mou.sr/3WcWExh ．|

- SN74CB3T3245はパッケージサイズに注意．SOIC-20はDWかDWRです．PWやPWRではありません．
- R11は"33k"ではなく"33"です．ダンピング抵抗なので．
- R12，最近のLEDは明るいので100kぐらいでちょうど良かったりします．使うLEDに応じて適切な値を選んで下さい．
- C9,C10はDECのプロセッサボードで0.33uFを使っていたのでそうしましたが，間違って0.1uFを載せても動きました．

## デバッグ用端子
- 基板の右上にあるBS[1:0], MAP_n, STRB_nは将来もしかしたら使うかもと思って用意している信号です．3.3Vに変換済みです．

