# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders

from pylab import *  # pylab kütüphanesini içe aktar, genellikle matplotlib ve numpy işlevlerini içerir.
from lego_robot import *  # lego_robot modülünü içe aktar, robotla ilgili işlevler sağlar.

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):  # Derivatif hesaplamak için fonksiyon, 'scan' (tarama verisi) ve 'min_dist' (geçerli mesafe) parametrelerini alır.
    jumps = [ 0 ]  # Derivatiflerin saklanacağı listeyi başlat, ilk değeri 0 olarak belirle.
    for i in range(1, len(scan) - 1):  # Tarama verilerinin başından sonuna kadar, ilk ve son eleman hariç her bir elemanı kontrol et.
        l = scan[i-1]  # 'i' konumundan bir önceki değeri al (sol komşu).
        r = scan[i+1]  # 'i' konumundan bir sonraki değeri al (sağ komşu).
        if l > min_dist and r > min_dist:  # Sol ve sağ komşu değerler geçerli mesafeden büyükse, geçerli bir ölçüm yapılıyor demektir.
            derivative = (r - l) / 2.0  # Derivatif hesaplamak için (sağ - sol) / 2 formülünü kullan.
            jumps.append(derivative)  # Hesaplanan derivatifi 'jumps' listesine ekle.
        else:
            jumps.append(0)  # Eğer bir komşu geçersizse, derivatifi 0 olarak ayarla.
    jumps.append(0)  # Son eleman için derivatifi 0 olarak ayarla.
    return jumps  # Hesaplanan derivatifleri içeren listeyi döndür.

def find_cylinders(scan, scan_derivative, jump, min_dist):  # Silindirleri bulan fonksiyon, 'scan' (tarama verisi), 'scan_derivative' (derivatif verisi), 'jump' (türev eşiği) ve 'min_dist' (geçerli mesafe) parametrelerini alır.
    '''
        Eğer türev jumpı aşıyorsa bu bir silindirdir deriz. eğer bu düşen kenarsa lefttir ve bundan sonraki taramalar ortalama için takip edilmelidir. Eğer düşen kenardan
        sonra tekrar düşen kenar gelirse ikinci bir silindir algılanmıştır. Bu hesaba dahil edilmez ve hesap 0 lanır, yakında olan hesaplanır.
    '''

    cylinder_list = []  # Silindirlere ait verilerin saklanacağı listeyi başlat.
    on_cylinder = False  # Başlangıçta silindirin tespiti yapılmamış.
    sum_ray, sum_depth, rays = 0.0, 0.0, 0  # Ray ve derinliklerin toplamı ile ray sayısını başlat.

    for i in range(len(scan_derivative)):  # Tarama verisinin türevini gez.
        current_der = scan_derivative[i]  # Mevcut türev değerini al.

        if abs(current_der) > jump:  # Türev değeri, belirlenen jump eşik değerinden büyükse, silindir başlıyor demektir.
            if on_cylinder:  # Eğer bir silindirin üzerindeysek.
                if current_der < 0:  # Düşen kenar tespit edilmişse.
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0  # Verileri sıfırla, yeni silindire geç.
                    continue  # Bir sonraki iterasyona geç.
                else:
                    on_cylinder = False  # Silindir tespiti bitmiş, silindir takibi sonlanıyor.
                    average_ray = sum_ray / rays  # Raylerin ortalamasını hesapla.
                    average_depth = sum_depth / rays  # Derinliklerin ortalamasını hesapla.
                    cylinder_list.append((average_ray, average_depth))  # Silindirin ortalama ray ve derinlik verisini listeye ekle.
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0  # Yeni silindir için verileri sıfırla.

            elif not on_cylinder and current_der < 0:  # Eğer bir silindirin üzerindeysek ve düşen kenar tespit edilmişse.
                on_cylinder = True  # Silindire geçildi.

        if scan[i] <= min_dist and not on_cylinder:  # Eğer mesafe geçerli değilse ve silindir tespiti yapılmadıysa.
            sum_ray, sum_depth, rays = 0.0, 0.0, 0  # Silindire ait verileri sıfırla.
            continue  # Bir sonraki iterasyona geç.

        if on_cylinder and scan[i] > min_dist:  # Eğer silindire ulaşıldıysa ve mesafe geçerli ise.
            rays += 1  # Ray sayısını artır.
            sum_ray += i  # Rayin pozisyonunu topla.
            sum_depth += scan[i]  # Derinliği topla.

    return cylinder_list  # Silindirlere ait ray ve derinlik verilerini içeren listeyi döndür.

if __name__ == '__main__':  # Eğer bu kod ana programda çalışıyorsa (modül olarak değilse).
    
    minimum_valid_distance = 20.0  # Geçerli mesafe olarak kabul edilecek minimum değeri belirle.
    depth_jump = 100.0  # Derivatifteki değişim eşiğini belirle (silindir tespiti için).

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()  # LegoLogfile sınıfından bir nesne oluştur.
    logfile.read("robot4_scan.txt")  # Tarama verilerini içeren log dosyasını oku.

    # Pick one scan.
    scan = logfile.scan_data[7]  # Kullanılacak tarama verisinin numarasını belirle (bu durumda 8. tarama).
    
    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)  # Derivatif hesapla.
    cylinders = find_cylinders(scan, der, depth_jump, minimum_valid_distance)  # Silindirleri tespit et.

    # Plot results.
    plot(scan)  # Tarama verisini çiz.
    # plot(der)  # (İsteğe bağlı) Derivatif verisini çiz.
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],  # Silindirlerin ray ve derinliklerini scatter ile çiz.
            c='r', s=200)  # Kırmızı renk ve büyük boyutlu noktalarla göster.
    show()  # Grafiği ekranda göster.
