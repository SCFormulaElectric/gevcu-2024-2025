import random

# Settings
start_time = 1.0
steps = 50
interval = 0.1

with open("test_log.txt", "w") as f:
    for i in range(steps):
        t = round(start_time + i * interval, 6)
        voltage = random.randint(300, 500)
        bamocar_temp = round(random.uniform(90.0, 130.0), 4)
        motor_temp = random.randint(70, 110)

        f.write(f'I({t}) Voltage reading : {voltage}\n')
        f.write(f'I({t}) BAMOCAR temp : {bamocar_temp}\n')
        f.write(f'I({t}) motor temp : {motor_temp}\n')

print("✅ test_log.txt generated.")
