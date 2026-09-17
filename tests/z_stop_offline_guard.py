"""Offline runner guard: private namespace plus denied hardware discovery."""
import os
import sys
import tempfile
os.environ['BIOXP_OEM_RUNTIME_STATE_ROOT'] = tempfile.mkdtemp(prefix='z-stop-offline-')
os.environ.pop('BIOXP_OEM_RUNTIME_ROOT', None)

def guard(event, args):
    if event in ('socket.connect', 'socket.connect_ex', 'socket.getaddrinfo'):
        raise RuntimeError('offline qualification denies network: ' + event)
    if event == 'open' and args and isinstance(args[0], str) and args[0].startswith(('/dev/bus/usb/', '/dev/video', '/var/lib/bioxp-oem-runtime/')):
        raise RuntimeError('offline qualification denies live path: ' + args[0])
sys.addaudithook(guard)
import usb.core

def denied(*args, **kwargs):
    raise RuntimeError('offline qualification denies hardware discovery/initialization')
usb.core.find = denied
from bioxp.usb_driver import BioXpTester
BioXpTester.__init__ = denied
