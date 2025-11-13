# ─────────────────────────────────────────────────────────
# Project layout
#
# /boot.py                # minimal Wi‑Fi + hostname; prints IP
# /main.py                # tiny entrypoint – calls app.main()
# /lib/app/__init__.py    # app.main(): start HTTP+SSE server
# /lib/app/config.py      # user settings (Wi‑Fi, pins, rate, debug, HOSTNAME)
# /lib/app/bmm350.py      # sensor driver + BMM350Node
# /lib/app/nodes.py       # create sensor nodes from pin list
# /lib/app/server.py      # HTTP index + SSE handler (prints clickable URLs)
# ─────────────────────────────────────────────────────────
