# Her bir taranan silindir için, dünya koordinat sisteminde 
# Kartezyen koordinatlarını bulun.
# Tarayıcıdan gelen silindirler ile referans silindirler arasındaki en yakın
# çiftleri bulun ve bunları hizalayacak en uygun dönüşümü hesaplayın.
# Daha sonra, taranan silindirleri bu dönüşümü kullanarak çıktıya yazdırın.
# 04_c_estimate_transform


import numpy as np
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt

def compute_distance(i,j):
    x = i[0] - j[0]
    y = i[1] - j[1]
    return np.sqrt(x*x + y*y) 

# Silindirler ile referans silindirleri arasındaki en yakın çiftleri bul.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    for i, cyl in enumerate(cylinders):
        for j, rcyl in enumerate(reference_cylinders):
            if compute_distance(cyl,rcyl) < max_radius:
                cylinder_pairs.append((i,j))

    return cylinder_pairs

'''compute_center fonksiyonu, bir nokta listesinin merkez noktasını hesaplayar.'''
def compute_center(point_list):
    # Boş liste kontrolü.
    if not point_list:
        return (0.0, 0.0)
    # Liste boş değilse, değerleri topla ve böl.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))

# İki nokta kümesi arasındaki dönüşümü tahmin et.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Sol ve sağ kümenin merkezini hesapla.
    lc = compute_center(left_list) # dünya koordinatındaki (görünen) merkez
    rc = compute_center(right_list) # referans koordinatlarındaki merkez

    # Merkezden kaydırılmış noktaları hesapla.
    l_prime = [(l[0]-lc[0],l[1]-lc[1])for l in left_list]  
    r_prime = [(r[0]-rc[0],r[1]-rc[1])for r in right_list]  

    cs,ss,rr,ll = 0,0,0,0

    for i in range(len(left_list)):
        l , r = l_prime[i] , r_prime[i] # İlk indeks silindiri, ikinci indeks koordinatı verir

        cs += ((r[0] * l[0]) + (r[1] * l[1]))
        ss += ((-1 * (r[0] * l[1])) + (r[1] * l[0]))
        rr += ((r[0] * r[0]) + (r[1] * r[1]))
        ll += ((l[0] * l[0]) + (l[1] * l[1]))
        
    # Lambda, c, s, tx ve ty hesapla.
    if ((ll - 0.0) < 0.00001):
        return None
    # Ölçek sabitlenmişse lambda=1, aksi halde hesaplanır.
    if fix_scale:
        la = 1
    else:
        la = np.sqrt(rr/ll)

    cs_sum = np.sqrt((cs*cs)+(ss*ss))
    c = cs / cs_sum
    s = ss / cs_sum

    tx = rc[0] - (la * (c*lc[0] - s*lc[1]))
    ty = rc[1] - (la * (s*lc[0] + c*lc[1]))

    return la, c, s, tx, ty

# Verilen bir dönüşümü uygula.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

if __name__ == '__main__':
    # Filtre adımı için kullanılan sabitler.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # Tarama sırasında silindir tespiti için kullanılan sabitler.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Silindir eşleştirme için maksimum mesafe.
    max_cylinder_distance = 300.0

    # Başlangıç pozisyonu (öngörülen değerler).
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Tüm taramaları içeren günlük dosyasını oku.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Referans silindirleri oku (haritamız).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = open("estimate_transform.txt", "w")
    for i in range(len(logfile.scan_data)):
        # Yeni pozisyonu hesapla.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Silindirleri tespit et ve dünya koordinatlarına çevir.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # Her bir silindir için en yakın referans silindirini bul.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Silindir çiftlerini kullanarak dönüşümü tahmin et.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)

        # Tahmin edilen dönüşümü kullanarak silindirleri dönüştür.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]            

        # Dosyaya yaz.
        # Pozisyonu yaz.
        out_file.write("F %f %f %f\n" % pose)
        # Tarayıcı koordinat sistemindeki tespit edilen silindirleri yaz.
        write_cylinders(out_file, "D C ", cartesian_cylinders)
        # Tahmin edilen dönüşüm uygulanmış silindirleri yaz.
        write_cylinders(out_file, "W C ", transformed_world_cylinders)

    out_file.close()
