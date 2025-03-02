from lego_robot import *  # Robot tarama verisi işleme için gerekli fonksiyonları içe aktar
from math import sin, cos  # Koordinat dönüşümü için trigonometrik fonksiyonları içe aktar

# Tarama verisinin türevini hesapla, geçersiz ölçümleri göz ardı et
def compute_derivative(scan, min_dist):
    jumps = [0]
    for i in range(1, len(scan) - 1):  # Tarama verisi üzerinde gezin
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:  # Geçersiz ölçümleri (minimum mesafeden küçük) göz ardı et
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

# Tarama verisinde türev (jump) eşiğini aşan silindirleri tespit et
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []  # Silindirlerin listesi
    on_cylinder = False  # Silindirin başında olup olmadığımızı takip et
    sum_ray, sum_depth, rays = 0.0, 0.0, 0  # Ortalama hesaplaması için geçici değişkenler

    for i in range(len(scan_derivative)):
        current_der = scan_derivative[i]
        
        if abs(current_der) > jump:  # Eğer türev eşiği aşılırsa, potansiyel silindir var
            if on_cylinder:
                if current_der < 0:  # Silindirin sonu
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0
                    continue
                else:
                    on_cylinder = False
                    average_ray = sum_ray / rays  # Ortalama ray (ışın)
                    average_depth = sum_depth / rays  # Ortalama derinlik
                    cylinder_list.append((average_ray, average_depth))  # Silindirin ray ve derinliğini listeye ekle
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0

            elif not on_cylinder and current_der < 0:  # Silindirin başı
                on_cylinder = True

        if scan[i] <= min_dist and not on_cylinder:  # Geçersiz veriyi atla
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
            continue

        if on_cylinder and scan[i] > min_dist:  # Silindir verilerini biriktir
            rays += 1
            sum_ray += i  # Işın indeksini biriktir
            sum_depth += scan[i]  # Derinliği biriktir

    return cylinder_list

# Silindirin kutupsal koordinatlarını Kartezyen koordinatlara dönüştür
def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:  # (beam_index, range) her bir silindiri işle
        # Beam index'ini açıya dönüştürmek için LegoLogfile.beam_index_to_angle fonksiyonunu kullan
        theta = LegoLogfile.beam_index_to_angle(c[0])
        # Kartezyen koordinatları (x, y) hesapla
        x = (c[1] + cylinder_offset) * cos(theta)
        y = (c[1] + cylinder_offset) * sin(theta)
        result.append((x, y))  # Hesaplanan (x, y) koordinatlarını sonuca ekle
    return result

if __name__ == '__main__':
    minimum_valid_distance = 20.0  # Geçerli ölçüm için minimum mesafe
    depth_jump = 100.0  # Silindiri tanımlamak için türev eşiği
    cylinder_offset = 90.0  # Silindir ofseti

    # Tüm taramaları içeren log dosyasını oku
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Çıktı dosyasını aç ve silindir verilerini yaz
    out_file = open("cylinders.txt", "w")
    for scan in logfile.scan_data:
        # Mevcut taramayı işleyerek silindirleri bul
        der = compute_derivative(scan, minimum_valid_distance)
        cylinders = find_cylinders(scan, der, depth_jump, minimum_valid_distance)
        print(cylinders)  # Bulunan silindirleri yazdır (hata ayıklama için)
        
        # Polar koordinatları Kartezyen koordinatlara dönüştür
        cartesian_cylinders = compute_cartesian_coordinates(cylinders, cylinder_offset)
        
        # Sonuçları çıktı dosyasına yaz
        print("D C", file=out_file, end=" ")
        for c in cartesian_cylinders:
            print("%.1f %.1f" % c, file=out_file, end=" ")
        print("", file=out_file)  # Her tarama için satır sonu ekle
    out_file.close()
