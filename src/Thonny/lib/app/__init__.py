import uasyncio as asyncio
from .server import serve_forever


def main():
    try:
        asyncio.run(serve_forever())
    finally:
        asyncio.new_event_loop()
