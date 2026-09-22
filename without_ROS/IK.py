import math as m
import pyfirmata
import time

board = pyfirmata.Arduino('COM6')
print("success")
iter8 = pyfirmata.util.Iterator(board)
iter8.start()

s1 = board.get_pin('d:11:s') #spalla
s2 = board.get_pin('d:5:s') #gomito
s3 = board.get_pin('d:7:s') #polso
s4 = board.get_pin('d:8:s')  #base
s5 = board.get_pin('d:10:s') #pinza

# j_x = board.get_pin('a:0:i')
# j_y = board.get_pin('a:1:i')
# j_x_1 = board.get_pin('a:2:i')
# j_y_1 = board.get_pin('a:3:i')

# s5.write(50)
# s3.write(90)





def ik(x1, y1, z1, ang3):
    l_1 = 12
    l_2 = 12
    l_3 = 12

    ang4 = m.degrees(m.atan(x1/y1))
    new_y = m.sqrt(m.pow(x1,2)+m.pow(y1,2))



    y = new_y - (l_3 * m.cos(m.radians(ang3)))
    z = z1 - (l_3 * m.sin(m.radians(ang3)))

    s = (m.pow(z, 2) + m.pow(y, 2) - m.pow(l_1, 2) - m.pow(l_2, 2)) / (2 * l_1 * l_2)

    q_2 = - round(m.acos(s), 2)

    q_1 = (m.atan(z / y)) - (m.atan((l_1 * m.sin(q_2)) / (l_1 + l_2 * m.cos(q_2))))

    q_1 = q_1 * 57.3
    q_2 = q_2 * 57.3
    q_3 = ang3-q_1-q_2

    print('----')
    print('Q1: ', round(q_1, 0), '\nQ2: ', round(q_2, 0))
    print("Q3: ",round(q_3))


    ang1 = 95 - 90 + q_1
    s1.write(ang1)
    ang2 = 25 - (q_2)
    s2.write(ang2)
    s3.write(q_3 + 90) # già in ang3
    s4.write(ang4+90)

# print("hgojrwh")


bo = 0.001
ik(0,15,15,bo)

# val = None
# z = 0
# y = 0
# x = 0
# b = 0
# while True:
#     pin_int = str(j_x.read())
#     pin_float = str(j_y.read())
#     pin = str(j_x_1.read())
#     pim = str(j_y_1.read())
#     if pin_int == '0.0':
#         val = 'alto'
#         z = z + 1
#         ik((0+x),(12+y),(12+z),bo)
#         print(val)
#     if pin_int == '1.0' or pin_int == '0.999':
#         val = 'basso'
#         z = z - 1
#         ik((0+x),(12 + y), (12 + z), bo)
#         print(val)
#     if pin_float == '1.0' or pin_float == '0.999' :
#         val = 'sinistra'
#         y = y - 1
#         ik((0+x),(12 + y), (12 + z), bo)
#         print(val)
#     if pin_float == '0.0':
#         val = 'destra'
#         y = y + 1
#         ik((0+x),(12 + y), (12 + z), bo)
#         print(val)
#     if pim == '0.0':
#         val = 'dx'
#         x = x + 1
#         ik((0+x),(12 + y), (12 + z), bo)
#         print(val)
#     if pim == '1.0' or pim == '0.999':
#         val = 'sx'
#         x = x - 1
#         ik((0+x),(12 + y), (12 + z), bo)
#         print(val)
#     # if pin == '0.0':
#     #     val = 'chiudi'
#     #     b = 20
#     #     s5.write(b)
#     # if pin == '1.0' or pin == '0.999':
#     #     val = 'apri'
#     #     b = 180
#     #     s5.write(b)
#
#     #print(pin_int)
#     time.sleep(0.02)



# ik(0,30,0,0.0001)
# board.pass_time(1.5)
# ik(0,20,0,0.0001)
# board.pass_time(1.5)
# ik(10,20,0,0.0001)
# board.pass_time(1.5)
# ik(10,30,0,0.0001)
# board.pass_time(1.5)
# ik(0,30,0,0.0001)
# board.pass_time(1.5)
# i = 0
# while i < 10:
#     ik(i,20, 15, 0.0001)
#     time.sleep(0.01)
#     i=i+0.1
# board.pass_time(1.5)
# while i < 0:
#     ik(i,20, 15, 0.0001)
#     time.sleep(0.01)
#     i=i+0.1

# ik(12,12,0.0001)
#
# i = 12
# while i < 21:
#     ik(i, 12, 0.0001)
#     time.sleep(0.01)
#     i=i+0.1
# board.pass_time(1.5)
# while i > 12:
#     ik(i, 12, 0.0001)
#     time.sleep(0.01)
#     i=i-0.1
#

a = 0
while a<3:
    for i in range(-31,31,2):
        ik(0,15,15,1)
        time.sleep(0.05)
    for j in range(31,-31,-2):
        ik(0,15,15,j)
        time.sleep(0.05)
    a=a+1
