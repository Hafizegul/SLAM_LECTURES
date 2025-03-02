# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file

from math import sin, cos, pi  # Matematiksel işlemler için gerekli fonksiyonlar içe aktarılır.
from lego_robot import *  # Lego robotuyla ilgili fonksiyonları içeren modül içe aktarılır.

def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    # Robot tekerlekleri eşit miktarda döndüğünde düz hareket eder.
    if motor_ticks[0] == motor_ticks[1]:
        lx = (motor_ticks[0]*ticks_to_mm)  # Sol tekerin hareket ettiği mesafe hesaplanır.
        ly = (motor_ticks[1]*ticks_to_mm)  # Sağ tekerin hareket ettiği mesafe hesaplanır.

        x, y, theta = old_pose  # Mevcut pozisyon ve yön açısı alınır.
        x = x + (lx*cos(theta))  # X ekseninde yeni pozisyon hesaplanır.
        y = y + (ly*sin(theta))  # Y ekseninde yeni pozisyon hesaplanır.
        
        return (x, y, theta)  # Yeni pozisyon döndürülür.

    else:
        # Tekerlek hareketleri eşit değilse, robot bir yay hareketi çizer.
        l = (motor_ticks[0]*ticks_to_mm)  # Sol tekerlek mesafesi hesaplanır.
        r = (motor_ticks[1]*ticks_to_mm)  # Sağ tekerlek mesafesi hesaplanır.

        alpha = (r-l)/robot_width  # Dönüş açısı hesaplanır.
        R = l/alpha  # Dönüş yarıçapı hesaplanır.

        x, y, theta = old_pose  # Mevcut pozisyon ve yön açısı alınır.

        # Tarayıcı kaydırması uygulanarak yeni başlangıç noktası belirlenir.
        x = x - scanner_displacement * cos(theta)
        y = y - scanner_displacement * sin(theta)

        # Dönüş merkezinin koordinatları hesaplanır.
        cx = x - ((R + (robot_width/2))*sin(theta))
        cy = y - ((R + (robot_width/2))*-cos(theta))

        theta = (theta + alpha) % (2*pi)  # Yeni yön açısı hesaplanır.

        # Yeni robot konumu hesaplanır.
        x = cx + ((R + (robot_width/2))*sin(theta))
        y = cy + ((R + (robot_width/2))*-cos(theta))

        # Tarayıcı kaydırması geri eklenir.
        x = x + scanner_displacement * cos(theta)
        y = y + scanner_displacement * sin(theta)
        
        return (x, y, theta)  # Yeni pozisyon döndürülür.

if __name__ == '__main__':
    # Tarayıcının robot merkezine göre kaydırılması (mm cinsinden).
    scanner_displacement = 30.0

    # Tekerlek hareketlerini mm'ye çevirme katsayısı.
    ticks_to_mm = 0.349

    # Robotun tekerlekleri arasındaki mesafe (mm cinsinden).
    robot_width = 150.0

    # Ölçülen başlangıç pozisyonu.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Motor verilerini içeren dosya okunur.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Motor hareketlerine göre pozisyon hesaplanır.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,scanner_displacement)
        filtered.append(pose)

    # Hesaplanan pozisyonlar dosyaya yazılır.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print("F %f %f %f" % pose, file=f)  # Formatlanmış çıktı dosyaya yazılır.
    f.close()  # Dosya kapatılır.
