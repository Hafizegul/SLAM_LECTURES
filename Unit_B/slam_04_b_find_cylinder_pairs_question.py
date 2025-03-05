# Her silindirin kartesyen koordinatlarını dünya koordinat sistemi içinde bulun,
# ardından referans silindir veri setinde en yakın noktayı bul ve çıktıyı al.
# 04_b_find_cylinder_pairs

import time
import numpy as np
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders

# Silindirler (nokta listesi) ve referans_silindirler listesi verildiğinde:
# Her bir silindir için en yakın referans_silindiri bul ve 
# (i, j) index çiftini, burada i silindirin indeksi, j ise referans_silindirin
# indeksi olacak şekilde, sonuç listesine ekle.
def compute_distance(i,j):
    x = i[0] - j[0]
    y = i[1] - j[1]
    return np.sqrt(x*x + y*y) 

'''
Bu fonksiyon, robotun LIDAR tarayıcısı ile tespit ettiği silindirleri (konum olarak) 
önceden bilinen referans silindirleriyle eşleştirerek, 
hangi tespit edilen silindirin haritadaki hangi silindire karşılık geldiğini bulmaya çalışır.
'''
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Buraya kodunuzu yazın.
    # Tüm silindirler ve referans_silindirler üzerinde döngü yapın.
    # Döngüde, eğer cylinders[i] en yakınsa reference_cylinders[j]'ye,
    # ve mesafe max_radius'tan küçükse, o zaman 
    # (i,j) çiftini cylinder_pairs listesine ekleyin, yani cylinder_pairs.append( (i,j) ).
    
    # list = [a,b,c,d,e]
    # for i,j in enumerate(list) ----> i 0 j a | i 1 j b 
    
    for i, cyl in enumerate(cylinders):
        for j, rcyl in enumerate(reference_cylinders):
            if compute_distance(cyl,rcyl) < max_radius:
                cylinder_pairs.append((i,j))
    
    return cylinder_pairs


if __name__ == '__main__':
    # filter_step için kullandığımız sabitler.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # Taramada kullandığımız silindir tespiti sabitleri.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Silindir eşleştirmeleri için izin verilen maksimum mesafe.
    max_cylinder_distance = 300.0

    # Mucizevi bir şekilde elde ettiğimiz başlangıç pozisyonu.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Tüm taramaları içeren log dosyasını oku.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Ayrıca referans silindirlerini (haritayı) oku.
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Tüm pozisyonlar üzerinde döngü kur.
    out_file = open("find_cylinder_pairs.txt", "w")
    for i in range(len(logfile.scan_data)):
        # Yeni pozisyonu hesapla.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Silindirleri çıkar, ayrıca bunları dünya koordinatlarına çevir.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # Her bir silindir için en yakın referans silindirini bul.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Dosyaya yaz.
        # Pozisyonu.
        out_file.write("F %f %f %f\n" % pose)
        # Tarayıcı koordinat sistemindeki tespit edilen silindirler.
        write_cylinders(out_file, "D C ", cartesian_cylinders)
        # Bir silindir çifti olan referans silindirleri.
        write_cylinders(out_file, "W C ",
            [reference_cylinders[j[1]] for j in cylinder_pairs])

    out_file.close()
