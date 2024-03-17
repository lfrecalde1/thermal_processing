import sensor, image, time, ustruct
from pyb import USB_VCP
usb = USB_VCP()
import machine


min_temp_in_celsius = 0.0
max_temp_in_celsius = 150.0

print("Resetting Lepton...")
# These settings are applied on reset
sensor.reset()
print(
    "Lepton Res (%dx%d)"
    % (
        sensor.ioctl(sensor.IOCTL_LEPTON_GET_WIDTH),
        sensor.ioctl(sensor.IOCTL_LEPTON_GET_HEIGHT),
    )
)
print(
    "Radiometry Available: "
    + ("Yes" if sensor.ioctl(sensor.IOCTL_LEPTON_GET_RADIOMETRY) else "No")
)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=5000)
clock = time.clock()
while(True):
    clock.tick()
    cmd = usb.recv(4, timeout=5000)
    if (cmd == b'snap'):
        img = sensor.snapshot().compress()
        usb.send(ustruct.pack("<L", img.size()))
        usb.send(img)
    elif (cmd == b'close'):
        break
        machine.reset()
    else:
        None
