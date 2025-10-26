# TangConsoleDCJ11MEM
DEC DCJ11 processor test board using memory systems and peripherals implemented  on Tang Console 138K.

This document is written mostly in Japanese. If necessary, please use a translation service such as DeepL or Google.

![](images/title.jpg)
# 概要
- PDP-11の命令セットを持つCPU「DEC DCJ11」のメモリシステムと周辺装置をFPGAボード(Tang Console 138K)上に実装する試みです．
- TangNano20Kを用いた同様のプロジェクト[TangNanoDCJ11MEM](https://github.com/ryomuk/TangNanoDCJ11MEM)の続編です．

各種説明は[doc/](./doc/)にあるファイルに記載しています．

## 最近の話題
- 以前の話題は[更新ログ](doc/01_updatelog.md)参照
- 2025/10/17
  - 磁気テープリーダ(TM11)等を実装してunix v7や2.9BSDをインストール可能なバージョン(20251017)を公開しました．わりと大幅なアップデートです．
  - ブートローダーのアドレスを実物のROM(BM873-YA)に準拠させるように変更しました．主なアドレスは下記のようになっています．詳細は[doc/06-1-bootloader.md](doc/06-1-bootloader.md)を参照して下さい．
    - RK(unix v1): 773700
    - RP(unix v6, RT-11等): 773010
  - 説明文書を[doc/](./doc/)にまとめました．まだログファイルそのままだったりソースのコメントのコピペばかりですが，動かすのに必要な情報は揃っていると思います．気が向いたらもう少し書き直します．

## 主要なファイル一覧
```
.
├── diskimage : SDメモリ用データ
│   ├── Caldera-license.pdf : UNIXのライセンス条項
│   ├── sd-unix-v1.dsk      : unix v1用disk image
│   └── sd-unix-v6.dsk      : unix v6用disk image
├── doc                      : 説明用文書
│   ├── 01_updatelog.md
│   ├── 02_PCB.md
│   ├── 03_FPGA.md
│   ├── 04_Interface.md
│   ├── 05_SDmemory.md
│   ├── 06-0_applications.md
│   ├── 06-1_bootloader.md
│   ├── 06-2_papertapebasic.md
│   ├── 06-3_unix-v1.md
│   ├── 06-4_unix-v6.md
│   ├── 06-5_unix-v7.md
│   ├── 06_6_unix-2.9BSD.md
│   ├── 06_7_RT-11v4.md
│   ├── 07_debugtool
│   └── 08_references.md
├── hdl                      : Gowin EDA用プロジェクト
│   ├── old                 : 旧版のバックアップ(あれば)
│   └── TangConsoleDCJ11MEM_project.xxxxxxxx
│       └── src
│             ├── rom.v      : ブートローダー
│             ├── sdhd.v     : HDエミュレータモジュール
│             ├── sdtape.v   : 紙テープエミュレータモジュール
│             ├── tc138k.cst : 物理制約(ピンアサイン)
│             ├── top.v      : メインプログラム
│             ├── uart.v     : uartモジュール
│             └── ws2812.v   : WS2812モジュール
├── images                    : 文書用画像ファイル
├── pcb
│   └── rev2.0 : 回路図，基板データ等(KiCAD 8用)
└── README.md
```
- diskimageフォルダ内にあるSDメモリ用のイメージファイルはUNIXのオリジナルソースからの派生物なので，ライセンス条件は Caldera-license.pdf (昔のBSD?)に従います．
- その他の部分についてはMITライセンスです．

## 動作状況
- unix v1
  - multi userで起動，rootでログインできました．
  - edでASCIIARTのプログラムを書いてccでセルフコンパイルして実行できました．
- unix v6
  - multi userで起動，rootでログインできました．
  - /usr/games/ にあるchessやbj, tttなどが動きました．
  - 実機でカーネルの再構築ができました．
- paper tape
  - paper tape BASICが起動して簡単なプログラムが動作しました．
  - unix v1, unix v6のデバイスとして読むことができました．
- rt-11 v4
  - 起動してHELP, DIRが動きました．(top.vのオプションを変更してビルドする必要あり)
- unix v7  
  - ディストリビューションテープからインストールとブートができました．
- 2.9BSD
  - ディストリビューションテープからインストールとブートができました．
  
## 既知の問題
- 電源投入直後はブートに失敗することが多い気がします．Tang Console のreconfigボタン(pmodコネクタのあたりにあるやつ)を押してリセットしてからリトライすると直ることが多いです．
- v6のファイルシステムは壊れやすいような気がします．電源断前にはsync3〜4回のおまじないが必要かも．
- v7でハードディスクにddしたbootloaderがちゃんと動かないような気がします．tapeのブートローダーでブートできます．

## 開発環境
- Tang Console 138K Device B (Device Cについては[doc/03_FPGA](doc/03_FPGA.md)参照．)
- Windows 11
  - KiCAD 8.0.5
  - GOWIN FPGA Designer V1.9.11.02(64-bit)
  - VMware Workstation 17 Player
    - Ubuntu 22.04.4
      - dd
      - simh
  - TeraTerm
  - PDP11GUI

# 動画
- [UNIX V6 on DEC DCJ-11 (PDP-11) with Tang Console 138K](https://www.youtube.com/watch?v=6rK0t8tJp9Y)

# 更新履歴
- 2025/09/03: 初版(20250922)公開
- 2025/09/06: unix v6でもccが動いたのでREADME修正．
- 2025/09/08: READMEに軽微な情報追加．
- 2025/09/22: 20250922版(pmod paper tape実装)暫定公開．(説明は未)
- 2025/09/26: 20250926版公開．(RT-11 v4起動)
- 2025/10/19: 20251017版公開．(磁気テープ実装, unix v7, 2.9BSD起動)
- 2025/10/19: ドキュメント構成見直し
- 2025/10/26: [doc/03_FPGA](doc/03_FPGA.md)にDevice Cに関するメモを追加．
