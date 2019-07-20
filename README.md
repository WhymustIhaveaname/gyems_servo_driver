# 上海光毓机电 RMD-S 系列步进电机驱动

首先强调一下重点：
## 如果你不幸买了光毓机电的电机，就赶快退货吧！及时止损，换别的电机，因为问题太多了。同时淘宝上还有一家叫 海泰机电 的卖的也是 光毓 的电机，请大家务必避开。
之后是仓库介绍：

使用pySerial调用串口，通过一个 Serial转RS485 模块控制光毓机电的 RMD-S 系列步进电机。光毓机电的电机 __非常难用__ ，具体地说：

* 电机的行为与文档上 __描述不符并且难以理解__ 。比如文档上的“位置闭环控制 4”，命令的第6个字节“转动方向字节”，描述是“0 表示顺时针, 1 表示逆时针”，然而根本不是，电机的方向与这个字节没有任何关系并且完全无法控制，完全无!法!控!制!
* 电机的 __响应极慢__ 。使用波特率为115200的串口询问电机位置，来回十几个字节应当在1ms内就能返回，然而光毓机电的电机竟然需要大约15ms才能返回，这个控制芯片锝有多慢？
* 电机 __经常会作出匪夷所思的行为__ ，令人无法理解，并且常常会导致严重后果。比如命令字节为a4的指令在归零时往往会多转一圈再归零，而且找不到规律什么时候多转什么时候不多转，并且，不幸的是，多转一圈就会导致我的机械的毁坏。
* 电机的 __角度约定混乱__ ，有时以顺时针为正方向，有时以逆时针为正，为编程带来很大困难。
* 技术文档 __不全并且敷衍__ ，甚至很多功能号称有的在技术文档中根本没写!
* 电机力量很小，并且 __经常抖动__ 。
* 电机正反面使用的螺丝型号不同，不方便安装，同时其中一面的螺丝孔很浅无法承受太大的力也体现出了他们工程师的不专业。
* 电机的尺寸图也有 __尺寸漏标__ ，更让人相信他们工程师的垃圾。
* 客服完全没有，技术智商掉线。
    
    
本人是由于任务时间紧没有时间购买别的电机才不得不使用的 光毓 的电机， __使用感觉极差__ ，电机各种与手册不符的行为和不听控制 __令人抓狂__ ，也浪费了我很多时间。与朋友交流后大家也一致同意，这样的电机就是 __白送都不会有人捡__ 。而最气人的是，这种电机竟然买好!几!百!，感觉自己花了1000买到了20的电机，简直 __太坑了__ 。

## 如果你不幸买了光毓机电的电机，就赶快退货吧！及时止损，换别的电机，否则气也把人气死。

~~但是光毓电机的老板还有他们的工程师还是有值得我们学习的地方的，就比如说，如果是我做出了这么垃圾的东西，我会羞愧难当无法继续生活甚至选择自尽，然而光毓机电呢？不但没有半点愧疚之情，反而说自己是“专业直流无刷伺服系统服务商”，这种乐观的精神，这种坚韧不拔的意志是值得我们学习的。不过我仍然认为，如果光毓机电的老板和工程师能够呆在更适合他们的地方——垃圾场，把做电机这种事情交给上过学的人来做，更有助于人类的发展。同时我在此善意的提醒光毓机电的老板和工程师一定要去有垃圾分类的城市的垃圾场，因为你们属于有害垃圾，不适合和其他垃圾呆在一起。~~
