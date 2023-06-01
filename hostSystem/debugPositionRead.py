"""
Code authored by Keegan Kelly
"""
import asyncio
import aiohttp
from ast import Pass
address = "http://192.168.0.181:3000/"


async def get_data():
    # put the data in r1, r2, and r3 into the server
    async with aiohttp.ClientSession() as session:
        async with session.get(address + "agentsLocal/") as resp:
            data = await resp.json()
            return data

while True:
    data = asyncio.run(get_data())
    print("(" + str(data[0]['position'][0]) + ", " + str(data[0]['position'][1]) + ", " + str(data[0]['position'][2]) + ") (" + str(data[1]['position'][0]) + ", " + str(data[1]['position'][1]) + ") (" + str(data[1]['position'][0]) + ", " + str(data[1]['position'][1]) + ")", end="\r")
