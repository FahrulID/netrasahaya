import math
def remap(value, leftMin, leftMax, rightMin, rightMax):
    if value < leftMin or value > leftMax:
        print(f'Value {value} is out of range {leftMin} - {leftMax}')
        return 0

    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

vx = -1.0
vy = -1.0
r = math.atan(vy/vx)
print(f'r: {r}')

kiri = remap(r, math.pi/4, math.pi/2, 50, 255)
depan_kiri = remap(r, math.pi/6, 5/12*math.pi, 255, 50) or remap(r, -1/12*math.pi, math.pi/6, 50, 255)
depan_kanan = remap(r, -5/12*math.pi, -math.pi/6,  50, 255) or remap(r, -math.pi/6, 1/12*math.pi, 50, 255)
kanan = remap(r, -math.pi/2, -1/4*math.pi, 255, 50)

print(f'kiri: {kiri}, depan_kiri: {depan_kiri}, depan_kanan: {depan_kanan}, kanan: {kanan}')