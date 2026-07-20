from bioxp.usb_driver import BioXpTester
import bioxp.api

t = BioXpTester.__new__(BioXpTester)
x = t._motion_oem_axis_profile('x', startup=True)
y = t._motion_oem_axis_profile('y', startup=True)
print('IMPORT_OK')
print('X disable_right=%s Y disable_right=%s' % (x.get('disable_right'), y.get('disable_right')))
assert x.get('disable_right') is True
assert y.get('disable_right') is True
print('PROFILE_PARITY_OK')
