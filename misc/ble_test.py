import asyncio
from bleak import BleakClient

address = "EC:94:CB:4D:7F:7A"
MODEL_NBR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

async def run(address):
    async with BleakClient(address) as client:
        model_number = await client.read_gatt_char(MODEL_NBR_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))
        while True:
            for i in range(0,100,10):
                if i == 100:
                    speed = str(i)
                elif i<10:
                    speed = '00' + str(i)
                else:
                    speed = '0' + str(i)
            
                motor1 = speed + 'M1'
                print(motor1)
                motor1bytes = b'%b' % motor1.encode('utf8')
                motor2 = speed + 'M2'
                print(motor2)
                motor2bytes = b'%b' % motor2.encode('utf8')
                await client.write_gatt_char(MODEL_NBR_UUID, motor1bytes)
                await client.write_gatt_char(MODEL_NBR_UUID, motor2bytes)
                await asyncio.sleep(0.005)
            
            
            for i in range(100, -1, -10):
                if i == 100:
                    speed = str(i)
                elif i<10:
                    speed = '00' + str(i)
                else:
                    speed = '0' + str(i)
            
                motor1 = speed + 'M1'
                print(motor1)
                motor1bytes = b'%b' % motor1.encode('utf8')
                motor2 = speed + 'M2'
                print(motor2)
                motor2bytes = b'%b' % motor2.encode('utf8')
                await client.write_gatt_char(MODEL_NBR_UUID, motor1bytes)
                await client.write_gatt_char(MODEL_NBR_UUID, motor2bytes)
                await asyncio.sleep(0.005)

loop = asyncio.get_event_loop()
loop.run_until_complete(run(address))
