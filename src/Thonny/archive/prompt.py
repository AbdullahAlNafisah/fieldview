import os, sys, machine

# Which files are present?
os.listdir('/')             # expect to see boot.py, main.py, lib/, etc.

# Where modules are searched:
sys.path

# Why did we reboot?
machine.reset_cause()       # compare to machine.PWRON_RESET, SOFT_RESET, DEEPSLEEP_RESET, etc.

# Quick peek at a file:
print(open('/boot.py').read())
print(open('/main.py').read())
