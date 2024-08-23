# coding: utf8
import argparse
import signal
import subprocess
import time

LOG_FILE = None

# 捕获Ctrl C退出
def exit_pro(signum, frame):
    if LOG_FILE:
        LOG_FILE.close()
    exit()

signal.signal(signal.SIGINT, exit_pro)
signal.signal(signal.SIGTERM, exit_pro)

# Default values
DEFAULT_BIN_PATH = '/usr/bin/tegrastats'
DEFAULT_LOG_FILE_PATH = './a.log'
DEFAULT_PARAMS = '--interval 500'  # 修改为传递间隔参数

def work(write_to_log=False, your_args=''):
    """将tegrastats加上时间戳
    @:arg write_to_log 是否写入log文件"""
    global LOG_FILE
    cmds = [BIN_PATH]
    cmds += your_args.split()
    p = subprocess.Popen(cmds, stdout=subprocess.PIPE)
    if write_to_log:
        LOG_FILE = open(LOG_FILE_PATH, 'a')
    while 1:
        current_stat = p.stdout.readline().decode().strip()
        if current_stat == '':
            print("tegrastats error")
            break
        text = "%s:\n%s" % (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), current_stat)
        print(text)
        if write_to_log:
            LOG_FILE.write(text + '\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='调用tegrastats，并记录输出到log文件里')
    parser.add_argument('-b', '--bin', metavar='where tegrastats is', required=False, dest='bin_path', action='store',
                        default=DEFAULT_BIN_PATH)
    # 如果没有手动指定输出文件路径，默认输出到 ./a.log
    parser.add_argument('-o', '--output', metavar='write the log file to here', required=False, dest='log_file_path',
                        action='store', default=DEFAULT_LOG_FILE_PATH)
    parser.add_argument('-p', '--params', metavar='additional arguments of tegrastats', required=False, dest='your_args',
                        action='store', default=DEFAULT_PARAMS)
    args = parser.parse_args()

    # 使用传入或默认的路径
    LOG_FILE_PATH = args.log_file_path
    write_to_log = bool(LOG_FILE_PATH)

    # 使用传入或默认的 tegrastats 路径
    BIN_PATH = args.bin_path
    
    # 使用传入或默认的参数
    PARAMS = args.your_args

    work(write_to_log, PARAMS)
