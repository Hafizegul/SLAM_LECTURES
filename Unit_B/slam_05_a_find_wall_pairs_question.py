# Taramayı alt örnekleme yap. Her nokta için arenanın duvarında en yakın noktayı bul.
# 05_a_find_wall_pairs

from lego_robot import *
from slam_b_library import filter_step, compute_cartesian_coordinates
from slam_04_a_project_landmarks import write_cylinders

# Bir tarama alır ve ölçümleri alt örnekler, böylece her sampling'inci nokta alınır.
# Tarayıcının koordinat sisteminde (x, y) noktalarından oluşan bir liste döner.
def get_subsampled_points(scan, sampling=10):
    # Taramadan alt örnekleme yap
    index_range_tuples = []
    for i in range(0, len(scan), sampling):  # xrange yerine range
        index_range_tuples.append((i, scan[i]))
        
    return compute_cartesian_coordinates(index_range_tuples, 0.0)

# Verilen bir nokta kümesi için, her bir noktanın p, sol, sağ, üst veya alt duvara
# eps'ten daha yakın olup olmadığını kontrol eder. Eğer öyleyse, 
# noktayı left_list'e ve duvardaki en yakın noktayı right_list'e ekler.
def get_corresponding_points_on_wall(points,
                                     arena_left=0.0, arena_right=2000.0,
                                     arena_bottom=0.0, arena_top=2000.0,
                                     eps=150.0):
    left_list = []
    right_list = []
    
    for p in points:
        x = p[0]
        y = p[1]
        
        if abs(arena_left-x) <= eps:
            left_list.append(p)
            right_list.append((arena_left,y))

        elif abs(arena_right-x) <= eps:
            left_list.append(p)
            right_list.append((arena_right,y))

        elif abs(arena_bottom-y) <= eps:
            left_list.append(p)
            right_list.append((x,arena_bottom))

        elif abs(arena_top-y) <= eps:
            left_list.append(p)
            right_list.append((x,arena_top))

    return left_list, right_list

if __name__ == '__main__':
    # filter_step için kullandığımız sabitler.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Tüm taramaları içeren log dosyasını oku.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Tüm pozisyonlar üzerinde dolaş.
    with open("find_wall_pairs.txt", "w") as out_file:  
        for i in range(len(logfile.scan_data)):  
            # Yeni pozu hesapla.
            pose = filter_step(pose, logfile.motor_ticks[i],
                               ticks_to_mm, robot_width,
                               scanner_displacement)

            # Noktaları alt örnekle.
            subsampled_points = get_subsampled_points(logfile.scan_data[i])
            world_points = [LegoLogfile.scanner_to_world(pose, c)
                            for c in subsampled_points]

            # Duvar üzerindeki karşılık gelen noktaları al.
            left, right = get_corresponding_points_on_wall(world_points)

            # Dosyaya yaz.
            # Poz.
            out_file.write("F %f %f %f\n" % pose)
            # Tarayıcı noktalarını ve karşılık gelen noktaları yaz.
            write_cylinders(out_file, "W C ", left + right)
