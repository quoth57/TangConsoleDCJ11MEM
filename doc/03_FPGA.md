# ビルド時の注意点
## デバイスのバージョンについて
- 2025年7月から138KのバージョンがDevice Cになったようです．私が2025年10月時点で開発に使用しているのはDevice Bなのですが，Cを使用する場合はGowin EDAのバージョンを最新にしたり，デバイスの設定を'C'にする必要があるようです．詳細については下記あたりを参照して下さい．
  - https://wiki.sipeed.com/hardware/en/tang/common-doc/get_started/install-the-ide.html
  - https://wiki.sipeed.com/hardware/en/tang/common-doc/questions

## config
- Configulationはimpl/の*.config.jsonに保存されているようなので，特に何もする必要は無いはずですが，もしエラーが出た場合はConfigurationのdual-purpose pinで，SSPI, READY, DONE, CPUをチェックします．
![](../images/DualPurposePins.png)

## warning
- 下記のwarningが出ますが，対処方法がいまいちわからないので放置しています．
```
WARN  (PR1014) : Generic routing resource will be used to clock signal 'sys_clk50_d' by the specified constraint. And then it may lead to the excessive delay or skew
WARN  (PR1014) : Generic routing resource will be used to clock signal 'CLK2_d' by the specified constraint. And then it may lead to the excessive delay or skew
WARN  (PR1014) : Generic routing resource will be used to clock signal 'ALE_n_d' by the specified constraint. And then it may lead to the excessive delay or skew
```

## program
- UARTのポートが複数あるので，Gowin programmerでは適切なポートを選択する必要があります．
![](../images/FPGA-programport.png)
- SRAMではなくflashメモリに書き込みます．1〜2分かかります．
  - Access Mode: External Flash Mode 5A
  - Operation: exFlash Erace, Program thru GAO-Bridge 5A
![](../images/FPGA-flash.png)

## reconfigボタン
- Tang Consoleになって付いたreocnfigボタンがかなり便利です．FPGAをflashメモリからreconfigしてBSRAMやレジスタを初期化してくれます．Tang Nano 20Kのときには付いていなかった(それ用のパッドはある)ので毎回書き込み直していました．
![](../images/FPGA-reconfig.jpg)

# FPGAに実装した機能
- コンソール入出力用UART
- Initialization Sequence時のPower-Up Configuration Register設定
- メモリ
  - 256KB RAM (760000-777777はI/O空間とROM用なので使えるのは248KB)
  - ブート用ROM
- ハードディスクドライブ RF11, RK11 (sdメモリによるエミュレーション)
- 外部演算装置 KE11-A (unix v1に必須)
- 紙テープリーダパンチャ PC11
- クロック KW11-L
- BS0, BS1は見ていません．DAL[15:0]とAIO[3:0]を見ればとりあえず十分だったので．
- DAL[21:18]も見ていません．
- 磁気テープドライブ TM11 (TU10)
- 大容量ハードディスクドライブRP11 (RP03)
