# tx-utils

这一个nvidia jetson tegra x系列的工具包。主要用来记录tx板子的状态，并将其写入excel表格，便于生成图表。

## 使用

### tegrastats2

Nvidia自带了一个tegrastats工具，默认位于home目录下。可以用来查看cpu和gpu的一些状态信息，我写了这个脚本，在其输出结果中加入了时间。

 ```bash
 sudo python3 tegrastats2.py 
 ```

你应该用sudo来执行这个脚本，因为sudo权限才能让tegrastats获得到gpu的状态。
`--bin`是tegrastats的路径，`--output`是log日志输出的路径。

`--params`是附加给`tegrastats`的参数，自从jetpack3.2以后，nvidia提供的tegrastats工具越来越强大，给了更多了运行参数。

### visualize

将原始日志内容格式化到excel文件里，并自动生成cpu占用率和gpu占用率的折线图。


```bash
python3 visualize.py
```

