# Her bir tarama (scan) içerisindeki silindirlerin
# dünya koordinat sistemindeki Kartezyen koordinatlarını bulur.
# Sonuçları, tüm taramalara ait tüm silindirleri içeren bir dosyaya yazar.
# 04_a_project_landmarks


from lego_robot import *
from slam_b_library import filter_step, compute_derivative,\
     find_cylinders, compute_cartesian_coordinates

# Bu fonksiyon, tarama verisinden silindir tespiti ve konum bulma işlemlerini yapar.
def compute_scanner_cylinders(scan, jump, min_dist, cylinder_offset):
    # Taramadan türev (derivative) hesaplanır. Türev, mesafe değişimlerini gösterir.
    der = compute_derivative(scan, min_dist)

    # Türevden faydalanarak silindirler tespit edilir.
    cylinders = find_cylinders(scan, der, jump, min_dist)

    # Tespit edilen silindirler, sensör koordinat sisteminden Kartezyen koordinatlara çevrilir.
    scanner_cylinders = compute_cartesian_coordinates(cylinders, cylinder_offset)
    return scanner_cylinders

# Bir silindir listesini belirtilen dosyaya yazar (her satırda bir silindir listesi).
# Satır başlığı (line header), satırın başlangıcında yer alır ve hangi tip veri olduğunu belirtir.
# Örneğin "D C" algılanan silindiri, "W C" ise dünya koordinatındaki silindiri gösterir.
def write_cylinders(file_desc, line_header, cylinder_list):
    line = line_header  # Satırın başına belirtilen başlığı ekle.
    for c in cylinder_list:
        # Her bir silindirin x ve y koordinatlarını yaz.
        line += "%.1f %.1f " % c
    file_desc.write(line+"\n")  # Satırı dosyaya yaz.

if __name__ == '__main__':
    # Hareket (motor) hesaplamaları için kullanılan sabitler.
    scanner_displacement = 30.0  # Robotun tarayıcısının robot merkezine olan mesafesi.
    ticks_to_mm = 0.349          # Tekerlek enkoderinden alınan her bir tick'in milimetre cinsinden karşılığı.
    robot_width = 150.0          # Robotun iki teker arasındaki mesafesi.

    # Silindir tespiti için kullanılan sabitler.
    minimum_valid_distance = 20.0  # Silindir olarak kabul edilecek minimum mesafe.
    depth_jump = 100.0             # Derinlik değişiminde silindir olarak algılanacak eşik.
    cylinder_offset = 90.0         # Silindirlerin merkezini lazer okumasına göre ayarlamak için ofset.

    # Başlangıç pozisyonu (x, y, theta), bu pozisyon deneysel veya harici olarak elde edilmiş olabilir.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Tüm taramaları (scan) içeren log dosyasını oku.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")  # Motor hareketleri verisi.
    logfile.read("robot4_scan.txt")    # Tarama verisi (lazer ölçümleri).

    # Tüm taramalar boyunca iterasyon yap.
    out_file = open("project_landmarks.txt", "w")  # Sonuçları yazacağımız dosyayı aç.
    for i in range(len(logfile.scan_data)):
        # Yeni pozisyonu hesapla (hareket modeline göre bir adım at).
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Bu pozisyondaki taramadan silindirleri tespit et ve dünya koordinatlarına dönüştür.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        
        # Silindirleri, tarayıcı koordinat sisteminden dünya koordinat sistemine dönüştür.
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # Sonuçları dosyaya yaz.
        # Robotun yeni pozisyonunu yaz.
        out_file.write("F %f %f %f\n" % pose)
        # Tarayıcı koordinat sistemindeki algılanan silindirleri yaz.
        write_cylinders(out_file, "D C ", cartesian_cylinders)
        # Dünya koordinat sistemindeki silindirleri yaz.
        write_cylinders(out_file, "W C ", world_cylinders)

    out_file.close()  # Dosyayı kapat.
