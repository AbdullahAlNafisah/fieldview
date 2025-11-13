import machine, sys
from app import main as app_main

print(">> main.py start", machine.reset_cause())

try:
    app_main()
except Exception as e:
    print("Fatal error in main:")
    sys.print_exception(e)
finally:
    # Always reset to ensure a clean restart after exit/exception.
    machine.reset()
