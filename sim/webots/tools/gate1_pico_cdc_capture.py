#!/usr/bin/env python3
"""Explicit Pico CDC RX-only capture with bounded same-identity reopen. No UART."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import select
import signal
import sys
import termios
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.gate1 import RealClock
from pil.gate1_reset_capture import capture_cdc, CaptureStopped
from pil.gate1_trial import labels


class PicoCDC:
    def __init__(self,path):
        if not path.is_symlink():raise FileNotFoundError('Pico CDC by-id not present')
        if not re.fullmatch(r'/dev/ttyACM[0-9]+',str(path.resolve())):
            raise OSError('Pico CDC target is not a ttyACM')
        self.fd=os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            fcntl.ioctl(self.fd,termios.TIOCEXCL)
            a=termios.tcgetattr(self.fd);a[0]=a[1]=a[3]=0
            a[2]=termios.CLOCAL|termios.CREAD|termios.CS8|termios.HUPCL
            a[4]=a[5]=termios.B115200
            a[6][termios.VMIN]=a[6][termios.VTIME]=0
            termios.tcsetattr(self.fd,termios.TCSANOW,a)
            print('PICO_USB_CAPTURE_READY; RX only; TRIAL / NOT GATE 1 PASS',flush=True)
        except BaseException:
            os.close(self.fd);raise
    def read(self,n):
        if not select.select([self.fd],[],[],0)[0]:raise BlockingIOError()
        return os.read(self.fd,n)
    def close(self):os.close(self.fd)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hardware-authorized',action='store_true')
    p.add_argument('--device',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration-seconds',type=int,default=700)
    a=p.parse_args()
    if not a.hardware_authorized:p.error('operator permission required; no device opened')
    name=a.device.name
    mtk3='usb-micro_T-Kernel_Forum_MTK3_Thread-Metric_RP2040_MTK3PICO-if00'
    forbidden=any(token in name.casefold()for token in ('debug_probe','cmsis-dap'))
    if (a.device.parent!=Path('/dev/serial/by-id')or forbidden or
            not (name==mtk3 or name.startswith('usb-Raspberry_Pi_Pico_'))):
        p.error('exact MTK3PICO/Pico CDC by-id required; Debug Probe/CMSIS-DAP/UART forbidden')
    if not 1<=a.duration_seconds<=700:p.error('duration1..700 seconds')
    if a.output.exists():p.error('fresh output directory required')
    print('PICO_CDC_CAPTURE_WATCH_READY; RX/reopen only; TRIAL / NOT GATE 1 PASS',flush=True)
    interrupted=False
    def stop(signum,frame):
        nonlocal interrupted
        if not interrupted:
            interrupted=True  # Repeated signals must not interrupt close/export.
            raise CaptureStopped(signum)
    previous={sig:signal.getsignal(sig)for sig in (signal.SIGTERM,signal.SIGINT)}
    try:
        for sig in previous:signal.signal(sig,stop)
        r=capture_cdc(lambda:PicoCDC(a.device),a.output,RealClock(),
                      duration_us=a.duration_seconds*1000000,labels=labels(True))
    finally:
        for sig,handler in previous.items():signal.signal(sig,handler)
    print(json.dumps(r,indent=2))
    return 2  # Capture completion never means Gate1 PASS.

if __name__=='__main__':raise SystemExit(main())
